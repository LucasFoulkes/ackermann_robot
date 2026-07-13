# ackermann_robot

Self-learning Ackermann robot: an encoder-less 1/10 crawler that learns to
drive from ordinary Nav2 goals, given only a per-vehicle "birth certificate"
(geometry, actuator channels/neutrals/safe bounds — no calibration, no
polarity). Amnesia exam passed 2026-07-13: memory wiped + config falsified,
24/26 goals across three from-scratch sessions.

Multi-package repository (nav2-style):

| Package | Role |
|---|---|
| [`adaptive_ackermann/`](adaptive_ackermann/) | The portable brain: learning controller + cusp dispatcher + offline refit tools. Hardware-free, effort-native — never sees a pulse unit. |
| [`adaptive_ackermann_msgs/`](adaptive_ackermann_msgs/) | The one interface: `EffortCommand` (normalized [-1, 1] per actuator, sign semantics learned not declared). Standalone so drivers depend on the message, never on the brain. |
| [`ackermann_robot/`](ackermann_robot/) | THIS vehicle's bringup: birth certificate, launch, Nav2 configs, URDF, `tf_odom_bridge` (MOLA TF → /odom), operating tools (`tools/preflight_session.sh`, `tools/auto_coverage_drive.py`, `tools/amnesia_exam.sh`), docs, and `history/` (archived µs-era experiments + data). |

The hardware driver lives outside this repo (`pca9685_effort_driver`):
drivers are per-substrate plugins, ~100 lines each — see the effort contract
in [`adaptive_ackermann/README.md`](adaptive_ackermann/README.md).

Start a session: `bash ackermann_robot/tools/preflight_session.sh` (kills
zombies, clears DDS shared memory, checks I2C, launches armed, health-gates,
prints READY).

## Vehicle measurements

Reference frame: rear axle center projected onto the ground; +X forward,
+Y left, +Z up.

| Measurement | Value |
|---|---:|
| Body length | 0.385 m |
| Body width | 0.300 m |
| Body height | 0.165 m |
| Ground clearance | 0.030 m (estimate) |
| Wheelbase | 0.2775 m |
| Wheel diameter | 0.120 m |
| Wheel width | 0.057 m |
| Track width, wheel center to wheel center | 0.247 m |
| Inner gap between wheels | 0.190 m |
| Approximate overall wheel width | 0.304 m |

| Sensor center | X | Y | Z | Yaw |
|---|---:|---:|---:|---:|
| RPLIDAR C1 | 0.237 m | 0 m | 0.165 m | 180 degrees |
| RealSense D435i prism | 0.2975 m | 0 m | 0.1275 m | 0 degrees |
