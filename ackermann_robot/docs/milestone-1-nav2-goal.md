# Milestone 1 — Nav2 Goal: the robot learns to drive itself

## Definition of done

Open RViz2, click a Goal Pose across the room, and the robot drives there—
avoiding obstacles seen by lidar and camera, mapping as it goes, obeying
commanded velocities, and measurably improving its driving with every run.

The novel component is the adaptive controller that learns signal-to-motion
while driving. Everything else should use existing ROS 2 Jazzy packages and
configuration. Once `cmd_vel` maps honestly to real motion, later behaviors
(follow-me, patrol, coverage, and farm work) only need to publish goals.

## Scope

In scope: RViz goal navigation, obstacle avoidance including camera-derived
obstacle heights, online self-calibration, simultaneous mapping, e-stop, and
continuous data recording.

Deferred: follow-me, leg detection, graph/topological navigation, GPS,
coverage patterns, multi-zone maps, and hole detection.

## Architecture

```text
RViz Goal Pose -> Nav2 -> twist_mux -> collision_monitor
                                      -> adaptive_ackermann_controller
                                      -> PCA9685 -> ESC + steering servo

RPLIDAR C1 /scan -> ICP odometry -> odom->base_link -> controller/SLAM/Nav2
D435i IMU -> Madgwick -> /imu/data ------------------^ 
D435i depth -> ground_filter -> pointcloud_to_laserscan -> /scan_camera
/scan + odometry -> slam_toolbox -> /map + map->odom
rosbag records every session
```

TF tree:

```text
map -> odom -> base_link -> {laser, camera_link, imu_link}
```

Static transforms come from the URDF and must use physical ruler
measurements.

## Camera obstacle pipeline

The lidar observes one horizontal plane and can miss low objects and
overhangs. The D435i fills that vertical gap. Milestone 1 uses a fully 2D
pipeline:

```text
D435i depth
  -> ground_filter (floor removal and gravity-aligned height band)
  -> pointcloud_to_laserscan (min height 0.02 m, max height 0.45 m)
  -> /scan_camera
  -> Nav2 obstacle layer + collision monitor
```

```yaml
obstacle_layer:
  observation_sources: scan scan_camera
  scan:
    topic: /scan
    marking: true
    clearing: true
  scan_camera:
    topic: /scan_camera
    marking: true
    clearing: true
    obstacle_max_range: 3.0
inflation_layer:
  inflation_radius: 0.45
  cost_scaling_factor: 3.0
```

Short-lived ghosts from collapsing the whole height band are acceptable at
Milestone 1 speeds. If they become troublesome, the named upgrade is STVL
(Spatio-Temporal Voxel Layer). Mount the camera pitched approximately 10–15°
down so it sees floor-to-robot-height space 0.5–3 m ahead. Use 424x240 at
15 fps initially.

### Ground-filter behavior

The custom node handles three tilt cases:

1. Static mount error: gravity-seeded RANSAC at startup estimates the floor,
   camera pitch, and camera height.
2. Dynamic chassis pitch: live Madgwick roll/pitch gravity-aligns the height
   band; marking may be gated during high pitch rate.
3. Inclined ground: continuous plane-aware segmentation is deferred because
   Milestone 1 assumes a flat house. The future implementation can also
   detect negative obstacles/holes.

After initial detection, the floor plane is tracked deterministically each
frame using the previous plane and IRLS/PCA-style refitting. RANSAC runs again
only after loss of track. Named alternatives are organized-cloud normal
segmentation and v-disparity; Patchwork++ is reserved for future outdoor 3D
lidar use.

Lidar tilt cannot be corrected from a 2D LaserScan. Level it mechanically and
cap costmap scan range below the predicted floor-strike distance. Mapping may
retain a longer range.

## Nodes and packages

| Node | Package/approach | Initial configuration |
|---|---|---|
| Robot transforms | `robot_state_publisher` | Measured URDF offsets |
| Lidar | `sllidar_ros2` official source driver | C1, 460800 baud, `laser` frame; delay consumers ~2 s |
| Camera | `realsense2_camera` | Depth 424x240x15, IMU, point cloud, direct USB 3 |
| IMU filter | `imu_filter_madgwick` | `use_mag: false`, output `/imu/data` |
| Odometry | Swappable backend | Default RTAB-Map `icp_odometry`, IMU seeded |
| SLAM | `slam_toolbox` | Async, 12 m max range, 0.05 m resolution |
| Navigation | `nav2_bringup` | Smac Hybrid-A* + Regulated Pure Pursuit |
| Command mux/e-stop | `twist_mux`, `teleop_twist_keyboard` | Keyboard priority above Nav2 |
| Safety | `nav2_collision_monitor` | After mux, before adaptive controller |
| Ground filter | Small custom node | Floor calibration and filtered cloud |
| Camera scan | `pointcloud_to_laserscan` | Output `/scan_camera` |
| Controller | Custom adaptive controller | Model learning, PI feedback, safety |
| Recorder | `rosbag2` | Always record core topics |

The design originally named `rplidar_ros`; implementation uses SLAMTEC's
official `sllidar_ros2` because its supported-device list explicitly includes
the RPLIDAR C1 and provides a C1 launch configuration.

## Collision monitor

Command chain:

```text
Nav2 -> twist_mux -> collision_monitor -> adaptive controller
```

It brakes but does not steer. Nav2 handles evasion.

```yaml
collision_monitor:
  base_frame_id: base_link
  cmd_vel_in_topic: cmd_vel_muxed
  cmd_vel_out_topic: cmd_vel
  polygons: [PolygonStop, PolygonSlow, FootprintApproach]
  PolygonStop:
    type: polygon
    action_type: stop
    points: "[[0.30,0.15],[0.30,-0.15],[-0.15,-0.15],[-0.15,0.15]]"
  PolygonSlow:
    type: polygon
    action_type: slowdown
    slowdown_ratio: 0.4
    points: "[[0.55,0.25],[0.55,-0.25],[-0.20,-0.25],[-0.20,0.25]]"
  FootprintApproach:
    type: polygon
    action_type: approach
```

The approach zone must project the current Ackermann arc forward. Reverse is
either given a short rear zone and low cap or disabled until trustworthy.

## Odometry bake-off

- Default: RTAB-Map `icp_odometry`, LaserScan native and IMU seeded, with
  `Reg/Strategy: 1`, `Icp/PointToPlane: true`, `Icp/VoxelSize: 0.05`, and
  `wait_imu_to_init: true`.
- Alternative: KISS-ICP. It needs scan-to-cloud conversion and a wrapper to
  constrain planar z/roll/pitch drift; it intentionally does not use IMU.
- Later experiment: Kinematic-ICP using a learned-model plus gyro odometry
  prior. This is not a first-light dependency.

Downstream interfaces must remain unchanged when the backend changes.

## Ackermann Nav2 configuration

- Smac Hybrid-A* with `REEDS_SHEPP` initially; switch to Dubins if reversing
  proves unstable.
- Minimum turning radius comes from self-measurement.
- Regulated Pure Pursuit with `use_rotate_to_heading: false` and reversing
  allowed initially.
- Remove spin recovery; retain backup and wait.
- Begin with 0.25 m XY and 0.5 rad yaw goal tolerances, tightening later.

## Adaptive Ackermann controller

One custom Python node, initially around 300 lines.

Inputs: `/cmd_vel`, `/odom`, and `/imu/data`.

Outputs: PCA9685 I2C PWM and `/controller/debug` containing targets,
measurements, learned-curve samples, and PI terms.

Persistent model: `~/.robot/model.yaml`, loaded at boot and saved periodically
and during orderly shutdown.

Pipeline:

1. Bicycle model converts `(v, omega)` into target speed and steering angle:
   `delta = atan(L * omega / v)`. Clamp steering and implied radius.
2. Steering feed-forward maps target angle to PWM using a learned curve with
   independent left/right branches.
3. Throttle feed-forward uses the learned speed curve plus a learned pitch
   gravity term.
4. PI feedback trims PWM from odometry speed error with anti-windup.
5. Learned dead zones are jumped across, never commanded within.
6. Safety includes configurable speed caps, a <=1 s stall cutoff, a 0.5 s
   command watchdog, and ESC neutral/arming at startup.
7. Recursive least squares, forgetting factor near 0.995, updates feed-forward
   while feedback always remains active. Learning never extrapolates beyond
   the observed trusted speed range.

Self-measurements:

- Full-lock slow circle for approximately 10 s gives `R_min = v / omega`.
- Slow PWM ramps on first boot measure forward/reverse deadbands.

Bootstrap phases:

| Phase | Activity |
|---|---|
| 0 | Keyboard sanity check for steering, ESC arming, and rough deadband |
| 1 | Deadband ramps, full-lock circle, and camera ground self-calibration |
| 2 | Nav2 at 0.3 m/s cap using PI; goals begin producing training data |
| 3 | RLS feed-forward converges and response sharpens |
| 4 | Raise caps and tighten goal tolerances |

## Acceptance tests

1. RViz goal approximately 5 m away: curve around furniture and arrive within
   tolerance without collision or oscillation.
2. After learning, a 0.5 m/s command produces 0.5 +/- 0.05 m/s.
3. A low box and an overhang invisible to the lidar plane are marked from
   depth and avoided.
4. Hard braking on open flat floor creates no phantom obstacle marks.
5. A driven loop closes near its taped start without a smeared map.
6. Run 10 has visibly lower tracking error than run 1; the saved model reloads.
7. Blocking the car cuts throttle within 1 s; e-stop halts it within 0.5 s.
8. Every session automatically produces a rosbag.

## Known risks

- Hobby ESC low-speed jerk: deadband jump and PI first; add pulse-start only
  if necessary.
- Ignore initial lidar scans and delay consumers about 2 s.
- Use a dedicated USB 3 port for the D435i and perform Intel IMU calibration.
- Test RPP reversing early; use Dubins/no reverse if needed.
- ICP may weaken in feature-poor corridors; IMU seeding bridges short gaps.
- If CPU load is high, reduce camera cloud resolution/fps before lidar rate.

## Inputs required from the ruler session

1. Wheelbase, front-to-rear axle, in millimeters.
2. Lidar ground height, x/y offset from rear-axle center, and mounting yaw.
3. Camera position and downward pitch.
4. PCA9685 ESC/servo channels and known pulse widths.
5. Servo full-lock left/right pulse widths.
6. Maximum robot height, defining what it cannot drive under.

Known measurements retained from the previous implementation are documented
in `src/ackermann_robot/README.md`; estimates must be remeasured.

## Intended milestone deliverables

- `robot_bringup`: URDF, launch files, and configuration for SLAM Toolbox,
  Nav2, twist_mux, collision monitor, and rosbag.
- `adaptive_ackermann_controller`: controller node, persistent model, and
  debug topic.

Both packages must build with colcon on the Raspberry Pi.
