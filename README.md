# ackermann_robot

Ackermann robot on ROS 2 Jazzy: RPLIDAR C1, RF2O odometry, **SLAM Toolbox**, Nav2, PCA9685 drive.

## Drive to goal (main stack)

**On the Pi** (lidar + SLAM + Nav2 + motors):

```bash
cd /home/luky/ros2_ws
source install/setup.bash
ros2 launch ackermann_robot drive.launch.py
```

Wrong USB port:

```bash
ros2 launch ackermann_robot drive.launch.py serial_port:=/dev/ttyUSB1
```

**On a laptop** (same `ROS_DOMAIN_ID` / network):

```bash
source /opt/ros/jazzy/setup.bash
rviz2 -d $(ros2 pkg prefix ackermann_robot)/share/ackermann_robot/rviz/nav2.rviz
```

In RViz use **“Nav2 Goal”** (not plain “2D Goal Pose”). Drive the robot a few metres first so SLAM has a map, then click a goal.

### What `drive.launch.py` starts

| Component | Role |
|-----------|------|
| `robot_state_publisher` | `base_link`, `base_laser`, sensor TF |
| `c1` | `/scan` |
| `rf2o` + **IMU** + **EKF** | `/odom_rf2o` fused → `/odom` + TF (`use_ekf:=false` = raw rf2o) |
| **slam_toolbox** | `/map`, `map`→`odom` (live mapping) |
| Nav2 | plan + follow path → `/cmd_vel_nav` |
| `cmd_vel_to_effort` | Nav2 → `/ackermann/cmd_effort` (`cmd_vel_to_effort.yaml` deadband + `closed_loop`) |
| `ackermann_driver` | PWM servo + motor |

### Save the map

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_map'}}"
```

### Dependencies

```bash
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-nav2-bringup
```

(`nav2_* packages are pulled in via `package.xml` / apt as needed.)

### Checks

```bash
ros2 topic hz /scan
ros2 topic hz /map
ros2 lifecycle get /bt_navigator
ros2 run tf2_ros tf2_monitor
```

---

## Other launches

| Launch | Use |
|--------|-----|
| `bringup.launch.py` | Lidar + odom + driver only (no SLAM/Nav2) — bench tests |
| `odom_fusion_experiment.launch.py` | RF2O + IMU EKF experiment |
| `slam.launch.py` | SLAM only (included in `drive.launch.py`) |
| `navigation.launch.py` | Same as `drive.launch.py` (lower-level name) |

---

## Tuning

- **Motor deadband / cmd_vel scaling:** `config/cmd_vel_to_effort.yaml`
- **PWM ticks:** `config/ackermann_driver.yaml`
- **Nav2 speeds / turn radius:** `config/nav2_params.yaml`
- **Steering calibration:** `ros2 run ackermann_robot steer_calibration` while driving

---

## RF2O (C1 compatibility)

See package notes in `rf2o_laser_odometry/LD19_COMPAT.md` — circular scan `angle_min=0`, `angle_max≈2π` handling and empty `init_pose_from_topic` default.
