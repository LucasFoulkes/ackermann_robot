# ackermann_robot

This package exists to hold a single launch file that starts:
- LD19 driver (`ldlidar_stl_ros2_node`)
- Static TF `base_link -> base_laser`
- RF2O (`rf2o_laser_odometry_node`)

## Run

After building the workspace:

```bash
cd /home/luky/ros2_ws
source install/setup.bash
ros2 launch ackermann_robot ld19_rf2o.launch.py
```

This launch also starts `slam_toolbox` (online async) by default. To run only LD19 + TF + RF2O:

```bash
ros2 launch ackermann_robot ld19_rf2o.launch.py enable_slam_toolbox:=false
```

## PS4 teleop (prints sticks)

Install dependency:

```bash
sudo apt update
sudo apt install python3-evdev
```

Bluetooth tools (needed for pairing on Linux / Raspberry Pi):

```bash
sudo apt install bluez
sudo systemctl enable --now bluetooth
```

Pair DualShock 4 (Bluetooth):

1) Put controller in pairing mode: hold **SHARE + PS** until the light bar flashes fast.

2) Pair with `bluetoothctl`:

```bash
bluetoothctl
power on
agent on
default-agent
scan on
```

Wait until you see a `Wireless Controller` / `DualShock 4` device, then:

```bash
pair XX:XX:XX:XX:XX:XX
trust XX:XX:XX:XX:XX:XX
connect XX:XX:XX:XX:XX:XX
quit
```

Permissions (recommended):

```bash
sudo usermod -aG input $USER
```

Log out/in (or reboot) after changing groups.

Run:

```bash
cd /home/luky/ros2_ws
source install/setup.bash
ros2 run ackermann_robot ps4_teleop
```

By default `ps4_teleop` also tries to auto-connect over Bluetooth using `bluetoothctl` (best-effort). If you want to disable that and only read evdev:

```bash
ros2 run ackermann_robot ps4_teleop --ros-args -p bluetooth_auto_connect:=false
```

It prints:
- left stick X (`ABS_X`)
- right stick Y (`ABS_RY`, inverted by default)

## SLAM Toolbox (build a map)

Install:

```bash
sudo apt update
sudo apt install ros-jazzy-slam-toolbox
```

Run mapping (in a new terminal):

```bash
cd /home/luky/ros2_ws
source install/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=false \
  slam_params_file:=/home/luky/ros2_ws/install/ackermann_robot/share/ackermann_robot/config/slam_toolbox_online_async.yaml
```

Notes:
- The stock Jazzy config uses `base_frame: base_footprint`; our stack uses `base_link`, so we ship an override YAML.
- `scan_topic` must be an absolute path like `/scan`.

Save the map:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: 'my_map'}"
```

Optional overrides:

```bash
ros2 launch ackermann_robot ld19_rf2o.launch.py port_name:=/dev/ttyUSB0 port_baudrate:=230400 yaw:=0 z:=0.18
```

If you need to flip the laser frame:

```bash
ros2 launch ackermann_robot ld19_rf2o.launch.py yaw:=3.14159265
```

## RF2O changes we made (for LD19)

These changes were made in the `rf2o_laser_odometry` package so RF2O correctly handles LD19 scans that publish `angle_min=0` and `angle_max≈2π`.

### 1) Default `init_pose_from_topic` to empty

File: `rf2o_laser_odometry/src/CLaserOdometry2DNode.cpp`

- Before: default was a non-empty ground-truth topic (commonly not present).
- Now: `declare_parameter<std::string>("init_pose_from_topic", "");`
- Why: When that topic doesn’t exist, RF2O appears “stuck” waiting and won’t process scans outside launch.

### 2) Use `LaserScan.angle_min/angle_max` for index→angle mapping

Files:
- `rf2o_laser_odometry/include/rf2o_laser_odometry/CLaserOdometry2D.hpp`
- `rf2o_laser_odometry/src/CLaserOdometry2D.cpp`

What changed:
- Added members: `angle_min_` and `angle_span_`.
- In `CLaserOdometry2D::init()`:
  - `angle_min_ = scan.angle_min;`
  - `angle_span_ = scan.angle_max - scan.angle_min;`
  - `fovh = std::abs(angle_span_);`
- Replaced the old “centered FOV” angle assumption with:

$$
\theta(u) = \mathrm{angle\_min} + u\,\frac{\mathrm{angle\_span}}{N-1}
$$

Applied in:
- Point projection from ranges
- Normal computation
- Solver loops / warping

Why:
- LD19 publishes a circular scan (`0..2π`). RF2O originally assumed angles were centered around 0, which effectively rotates/reflects rays and can look like the sensor is “backwards”.

### 3) Warping wraps angles into scan range

File: `rf2o_laser_odometry/src/CLaserOdometry2D.cpp` (`performWarping()`)

- Wrap `atan2()` angle into `[angle_min_, angle_min_ + angle_span_)` (using `2π` wrapping).
- Compute warped index using `(tita_w - angle_min_) / angle_span_`.
- Guard: return early if `angle_span_ == 0`.

Why:
- For circular scans, consistent wrapping/indexing is required so warping samples the correct column.

### Notes

- If you see intermittent `Waiting for laser_scans....`, QoS compatibility between LD19 publisher and RF2O subscriber is the next thing to check (RF2O subscribes with `best_effort()` in code).
