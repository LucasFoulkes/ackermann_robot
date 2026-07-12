# Stack map — every node, what it consumes, produces, and does

*2026-07-04. The default `drive.launch.py` stack, as actually wired.*

## Data flow in one picture

```
RPLIDAR C1 ──/scan──┬─> icp_odometry ──/odom_icp──> EKF ──/odom + odom→base_link TF
                    ├─> scan_throttle(5Hz) ─> speckle_filter ─/scan_slam─> slam_toolbox ─> /map + map→odom TF
                    └─> local & global costmaps (raw, 10 Hz)

D435i ──depth cloud──> depth_floor_scan ──/camera/scan──> costmaps (low obstacles)
      └─IMU 200Hz──> madgwick ─> imu_throttle(30Hz) ─/imu/data_ekf─> EKF (gyro yaw rate)
                                                          └────────> cmd_vel_to_effort (vibration + accel)

Nav2:  bt_navigator (behavior tree) ─> planner_server (Smac Hybrid, global costmap)
                                    └> controller_server (RPP, local costmap) ─/cmd_vel_nav─┐
       behavior_server (wait/drive_on_heading) ─/cmd_vel_nav────────────────────────────────┤
       retrace_recovery (custom BackUp) ─/cmd_vel_nav───────────────────────────────────────┤
                                                                                            v
auto_calib <──/ackermann/cmd_effort + /odom──  cmd_vel_to_effort  <─/odom, /imu/data_ekf, /auto_calib/params
    └──/auto_calib/params (learned breakaway fwd/rev, steering gains)──^      │
                                                              /ackermann/cmd_effort [steer, throttle]
                                                                              v
                                                                    ackermann_driver ─I2C─> PCA9685 ─PWM─> servo + ESC
```

## Nodes, by layer

### Sensing
| Node | In → Out | Job |
|---|---|---|
| `sllidar_node` (c1) | USB serial → `/scan` 10 Hz | RPLIDAR C1 driver |
| `realsense2_camera_node` | USB → depth cloud, color, raw IMU 200 Hz | D435i driver. Depth 480x270@6 (lowest valid profile). IMU = "Motion Module", known to wedge until USB power-cycle |
| `imu_filter_madgwick` | raw IMU → `/imu/data` | orientation filter |
| `imu_throttle` | `/imu/data` 200 Hz → `/imu/data_ekf` 30 Hz | rate-limit so the EKF isn't flooded |
| `scan_throttle` | `/scan` → `/scan_slam_pre` 5 Hz | SLAM-only decimation (costmaps/odom keep 10 Hz) |
| `scan_speckle_filter` (laser_filters) | `/scan_slam_pre` → `/scan_slam` | removes phantom dots before they bake into the map |
| `depth_floor_scan` | depth cloud + TF → `/camera/scan` | IMU-leveled floor removal; marks low obstacles (≥ ~4 cm) the lidar can't see. Ring-artifact suppression, temporal floor filter with divergence recovery. Respawns |

### State estimation
| Node | In → Out | Job |
|---|---|---|
| `icp_odometry` (rtabmap_odom) | `/scan` → `/odom_icp` | scan-to-map lidar odometry (replaced rf2o; anti-freeze params per icp-null-guess notes) |
| `ekf_node` (robot_localization) | `/odom_icp` (x,y,yaw,vx) + `/imu/data_ekf` (vyaw) → `/odom` + odom→base_link TF, 30 Hz | fuses ICP + gyro; 2.5σ innovation gate against phantom velocity spikes. BOTH yaw sources required (they cancel each other's drift) |
| `async_slam_toolbox_node` | `/scan_slam` + odom TF → `/map`, map→odom TF | live mapping + loop closure |

### Navigation (Nav2)
| Node | In → Out | Job |
|---|---|---|
| `bt_navigator` | goals → orchestrates | runs `ackermann_nav_to_pose.xml`: 0.4 s settle, 0.33 Hz replan, recovery ladder (clear costmaps → retrace → drive forward → wait) |
| `planner_server` (Smac Hybrid, REEDS_SHEPP) | global costmap → `/plan` | Ackermann-feasible paths, R=1.3 m, reverse_penalty 4.5 (forward preference — floor camera faces forward), ALL_DIRECTION goals |
| `controller_server` (RPP) | path + local costmap + `/odom` → `/cmd_vel_nav` | follows the path at 0.25 m/s; collision-checks ~1 s ahead (halts, doesn't swerve — swerving is the queued MPPI experiment) |
| `behavior_server` | recovery actions → `/cmd_vel_nav` | stock wait / drive_on_heading |
| `retrace_recovery` (ours) | `/odom` breadcrumbs; BackUp action "retrace" → `/cmd_vel_nav` | recovery reverse ALONG the recorded forward path (only ground known safe); escapes forward if stuck while reversing. Respawns |
| `lifecycle_manager_navigation` | — | brings up exactly the 4 Nav2 servers |
| costmaps (in planner/controller) | `/scan` + `/camera/scan` | global: static map + lidar + camera (raytrace 0.9 = REMEMBERS low obstacles); local: rolling 4 m, lidar + camera (raytrace 1.5 = stays fluid) + denoise + inflation |

### Actuation + learning (ours)
| Node | In → Out | Job |
|---|---|---|
| `cmd_vel_to_effort` | `/cmd_vel_nav`, `/odom`, `/imu/data_ekf`, `/auto_calib/params` → `/ackermann/cmd_effort` [steer, throttle] | THE control node: per-direction feedforward maps (F/R-calibrated), speed PI (stall-hold integral), yaw PI, learned-breakaway stiction kick (seeded per direction, explorer starts), conditional launch cooldown, direction interlock (1.8 s max hold), odometry-distrust watchdog, ZUPT gate (vibration vs phantom velocity), learned per-side steering gain correction. Respawns |
| `ackermann_driver` | `/ackermann/cmd_effort` → I2C to PCA9685 @0x40 | PWM out at 100 Hz frames (2x tick resolution): servo ch15, ESC ch14 (F/R mode, esc_inverted true). Servo idle-relax (no pulses when parked = no buzz). Stale-command watchdog → neutral. SIGINT-only kills (a SIGKILL leaves the chip free-running its last PWM). Respawns |
| `auto_calib` | `/ackermann/cmd_effort` + `/odom` → `/auto_calib/params` | THE learner: breakaway forward + reverse (event-based, anti-ratchet, recency window), steering gain per side (effort→curvature pairs), steady pairs (awaits M3 dither). Persists to `~/.ros/auto_calib_state.json` every 30 s. Respawns |

### Diagnostics
| Node | Job |
|---|---|
| `drive_logger` | 10 Hz CSV (`logs/drive_log_*.csv`): cmd/odom twist, efforts, curvatures, trims, ctl_state, vib, map pose, CPU. The evidence base for everything |
| `system_stats` | 5 s CPU/mem/temp samples |
| `drive_score` (on demand) | `ros2 run ackermann_robot drive_score` — 7-component 0-100 score, history in `logs/scores.csv` |
| `robot_state_publisher` | URDF → static TFs (base_link↔laser↔camera) |

## The three learned parameters (persist across restarts/reboots/builds)
1. Forward breakaway → seeds the kick (starts)
2. Reverse breakaway → seeds the reverse kick
3. Steering gain per side → corrects the steering feedforward

## Known live issues / next work
- Odometry phantom at the ICP/EKF source (Nav2's costmap shakes with it) — top of next session
- D435i Motion Module wedge = silent gyro loss (yaw measurement flatlines) — needs sensor-health watchdog
- M3 dither (speed-gain learning) — retires the last hand-set maps
- M4: mechanism deletion pass (7 control mechanisms → ≤4)
