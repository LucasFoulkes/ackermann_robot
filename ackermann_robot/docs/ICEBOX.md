# Icebox

Deferred work — not urgent, but worth doing before it bites.

## HARDWARE dead-man for the motor: PCA9685 ŌE pin -> GPIO (do before farm)

**Why (2026-07-04, it already bit twice):** the PCA9685 free-runs its last
PWM forever with no software alive — a SIGKILL'd driver + wedged I2C bus left
the motor running with nothing able to stop it. Software layers now in place
(driver stale-cmd neutral, SIGINT-only kills, respawn, systemd ExecStopPost
neutralizer via scripts/esc_neutral.py) cover every case EXCEPT a wedged bus.
**Only hardware covers that**: the chip's active-low ŌE pin disables all
outputs; a hobby ESC with no pulses does not drive.

**Fix:** find OE on the motor HAT (most boards tie it to GND — may need a
trace cut or solder jumper), wire to a Pi GPIO with a physical PULL-UP
resistor. Driver holds the GPIO low while alive (heartbeat); on Pi
reset/hang/driver death the pin floats HIGH -> outputs silent -> motor safe.
Verify: kill -9 the driver mid-drive (wheels off ground) — motor must stop
within the pull-up's reaction, regardless of bus state.

## RTAB-Map multi-sensor SLAM (lidar + camera together) — the farm-phase experiment

**What:** RTAB-Map's `rtabmap` node can consume BOTH the 2D lidar scan and the
RGB-D camera in one SLAM graph (`subscribe_depth: true` + `subscribe_scan:
true`), with EXTERNAL odometry (our EKF `/odom` — it does not need its own
visual odometry, which we already benchmarked as non-viable on this Pi).
Division of labor: camera images → bag-of-words VISUAL loop closure; lidar
scan → ICP link refinement (`Reg/Strategy: 1`) + proximity detection
(`RGBD/ProximityBySpace: true`) + the 2D occupancy grid for Nav2. The graph
node runs at `Rtabmap/DetectionRate` 1 Hz — a fraction of odometry-class CPU.

**Why it matters HERE:** SLAM Toolbox is lidar-only. The documented farm risk
(memory: crop rows = parallel-hall ambiguity at scale) is exactly the case
lidar loop closure fails and VISUAL loop closure shines — two identical-
looking rows to the lidar are visually distinct to the camera. This is the
strongest candidate fix, ahead of GPS/RTK fusion.

**Cost/risk:** replaces slam_toolbox (map format, Nav2 map topic, tuning all
change); rtabmap graph node CPU on Pi 5 unmeasured (1 Hz detection on
424x240 color is plausibly fine within our ~50% headroom, but measure);
needs the camera color stream enabled at a usable rate for BoW features.

**When:** before/at farm deployment, not during indoor tuning. Bench first:
run rtabmap in localization-off mapping mode alongside (not replacing)
slam_toolbox on a recorded bag, compare loop-closure recall and CPU.

Refs: RTAB-Map lidar+visual SLAM paper (arXiv 2403.06341 / J. Field Robotics
2019, Labbé), rtabmap_examples launch files in introlab/rtabmap_ros.

## Stable udev symlink for the RPLIDAR C1 serial port

**Problem:** [c1.launch.py](src/ackermann_robot/launch/c1.launch.py) hardcodes `serial_port=/dev/ttyUSB0`.
That index is assigned by enumeration order, so plugging in another USB-serial
device (or a different boot order) can make the C1 become `ttyUSB1` and bring-up
fails. The C1 connects via a CP2102N USB-UART adapter (Silicon Labs, usually
`10c4:ea60`).

**Fix:** add a udev rule that creates a stable `/dev/rplidar` symlink, then point
the launch file at it.

1. Find the adapter's identifiers (with the C1 plugged in):

   ```bash
   udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|serial'
   ```

2. Create `/etc/udev/rules.d/99-rplidar.rules`:

   ```
   # RPLIDAR C1 via CP2102N USB-UART adapter -> stable /dev/rplidar symlink
   SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar", MODE="0666"
   ```

   If more than one CP2102N device is ever connected, disambiguate by adding the
   per-device serial: `ATTRS{serial}=="<value from step 1>"`.

3. Reload and re-trigger:

   ```bash
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

4. Point the launch default at the symlink:

   ```python
   serial_port = LaunchConfiguration("serial_port", default="/dev/rplidar")
   ```

**Verify:** `ls -l /dev/rplidar` should point at the right `ttyUSB*`, and the
lidar should bring up regardless of which `ttyUSB` index it lands on.

## D435i IMU timestamp domain (HARDWARE_CLOCK reset)

**Problem:** at [d435i.launch.py](src/ackermann_robot/launch/d435i.launch.py) bring-up the
RealSense node warns:

    frame's time domain is HARDWARE_CLOCK. Timestamps may reset periodically.

IMU/frame timestamps come from the camera's own hardware clock, which can jump/reset.
If fused in robot_localization, this can surface as occasional "message too old / time
jumped backwards" warnings and EKF hiccups.

**Fix:** tell the wrapper to stamp messages with the host/global clock instead. In the
RealSense node params, set the timestamp domain to global time:

    "depth_module.global_time_enabled": true     # (and rgb_camera.global_time_enabled if color used)
    # the wrapper exposes "global_time_enabled" per-sensor; enable it for the motion module

i.e. add `global_time_enabled:=true` style args so frame stamps are converted to the
host clock domain (monotonic, no periodic reset). Re-run the smoke test and confirm
`/imu/data` header stamps advance monotonically with no backward jumps.

**Note:** only matters once the IMU is actually fused (robot_localization). Standalone
`/imu/data` works fine without it (verified: steady 200 Hz, valid orientation).

## D435i IMU factory calibration (manual, hands-on)

**Why:** smoke test logged `IMU Calibration is not available, default intrinsic and
extrinsic will be used` — this D435i has no accel/gyro calibration written to EEPROM.
Tolerable (Madgwick still produces orientation) but can leave a small persistent
tilt/bias. Worth doing once for cleaner roll/pitch.

**This is NOT automatic and has nothing to tune** — it's a guided physical procedure:
you place the camera on each of its 6 faces, hold it dead still ~5 s per pose on a
flat, level surface; the script records gravity in each pose, solves accel scale+bias,
and writes it to the camera EEPROM. Quality depends only on how still/level you hold it.

**Setup needed first (not currently installed):**
1. `pyrealsense2` Python bindings are missing, and the `rs-imu-calibration.py` script
   (a librealsense source-tree tool) is not installed — only the compiled `rs-*`
   binaries are present.
2. Get the script + bindings, e.g. `pip install pyrealsense2` and grab
   `tools/rs-imu-calibration/rs-imu-calibration.py` from the librealsense repo matching
   FW/lib v2.57.x. Make sure no other process (our launch) is using the camera.
3. Run `python3 rs-imu-calibration.py` and follow the 6-pose prompts; let it write to
   the device when done.

**Verify:** re-launch [d435i.launch.py](src/ackermann_robot/launch/d435i.launch.py) — the
"IMU Calibration is not available" warning should be gone, and `/imu/data` orientation
should settle closer to true level when the robot is stationary and flat.

## floor_scan / RANSAC cost — levers if the Pi gets CPU-starved

**Not urgent — measured cheap.** [depth_floor_scan.py](src/ackermann_robot/ackermann_robot/depth_floor_scan.py)
runs at `process_hz=8` via a timer that processes only the *latest* cloud (cloud
arrives ~18 Hz; we drop ~10/s on purpose). Measured on this Pi 5:

  * RANSAC alone (7000-pt subsample, 60 iters): ~6.7 ms
  * full numpy per-tick (RANSAC + floor removal + scan projection, 100k pts): ~11 ms
  * at 8 Hz => ~87 ms/s ≈ 9% of one core

RANSAC is NOT the bottleneck — it runs on a fixed 7000-pt subsample regardless of
cloud size (the `max_points` cap), so denser clouds don't slow it. The largest
single slice is the ROS PointCloud2 deserialization (`read_points_numpy`), which
scales with cloud size and is unrelated to the plane fit.

**If CPU-starved (Nav2 + SLAM + this together), in priority order:**
1. `process_hz: 5` in [floor_scan.launch.py](src/ackermann_robot/launch/floor_scan.launch.py)
   — 5 Hz is plenty for a Nav2 obstacle source; ~40% less CPU, no practical loss.
2. RealSense `decimation_filter` (e.g. 848x480 -> 424x240) — shrinks the cloud at
   the source, attacking the deserialization cost (the real one).
3. `max_points: 3000`, `ransac_iterations: 30` — halves the 6.7 ms, still robust
   for a single dominant plane.

Don't reach for Open3D/sklearn to "speed up RANSAC": sklearn RANSACRegressor is
numpy-underneath (not faster) and fits a biased vertical-residual plane (wrong);
Open3D's segment_plane is C++-fast but the 6.7 ms step isn't the bottleneck, and
it adds a heavy ARM dependency. Open3D only earns its place if we later need
multi-plane segmentation (floor + ramp/wall at once), per-point normals, or voxel
downsampling.
