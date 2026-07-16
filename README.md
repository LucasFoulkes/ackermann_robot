# ackermann_robot

A self-learning Ackermann robot: an encoder-less 1/10 crawler (Pi 5, ROS 2
Jazzy) that learns its own actuation from driving. It is given only a
"birth certificate" — geometry, actuator channels, neutral/safe pulse
bounds — and discovers polarity, breakaway effort, throttle maps, steering
maps, steering lag, and its own trackable turning envelope from lidar
odometry while doing ordinary work. No encoders, no calibration scripts,
no battery gauge. It can also follow a person.

Amnesia exam (memory wiped + config falsified): 24/26 goals across three
from-scratch sessions, 2026-07-13.

## Hardware

Pi 5 (NVMe HAT) · PCA9685 servo HAT at i2c `0x40` (steering ch 12, ESC
ch 14) · RPLIDAR C1 (2D, mounted low at 16.5 cm — deliberate; the robot
stays low) · RealSense D435i (IMU used; depth not yet fused) · brushed ESC
with factory double-tap reverse. `i2cdetect -y 1` should show 36/40/70.

| Measurement | Value |
|---|---:|
| Wheelbase | 0.2775 m |
| Track width | 0.247 m |
| Wheel diameter | 0.120 m |
| Body footprint | front_x 0.40 / rear_x −0.10 / half-width 0.16 m |
| Lidar pose | x 0.237 m, z 0.165 m, yaw 180° |

Full authoritative values: `ackermann_robot/config/birth_certificate.yaml`.

## Packages

| Package | Role |
|---|---|
| `adaptive_ackermann/` | The portable brain: learning controller (`adaptive_ackermann_controller`) + Reeds-Shepp cusp dispatcher (`path_segment_dispatcher`) + shared model code. Hardware-free, effort-native `[-1, 1]` — never sees a pulse unit. |
| `adaptive_ackermann_msgs/` | `EffortCommand` — the one interface between brain and any driver. |
| `person_tracker/` | 2D-lidar person detection (leg clustering + persistence grid), DR-SPAAM neural referee, Lyapunov pursuit follower. |
| `ackermann_robot/` | THIS vehicle: birth certificate, launch files, Nav2 configs, URDF, `tf_odom_bridge`, operator tools. |

**The effort contract** (drivers): `EffortCommand.steering_effort` /
`.drive_effort` are dimensionless `[-1, 1]`; `0` = declared neutral, `±1` =
declared safe extremes, piecewise-affine per side (spans may be asymmetric).
The **sign carries no semantic** — polarity is learned, never declared. A
driver must clamp, apply its fixed affine map, hold neutral on stale
commands (`reference_timeout_s`, stamped), and go neutral-then-release on
any error/shutdown. ~100 lines per substrate. The PCA9685 driver is a
separate repo (`pca9685_effort_driver`) and adds brownout self-heal
(re-detects a power-cycled chip via its MODE1 register).

## How to run

```bash
# full stack, hardware armed, person following:
ros2 launch ackermann_robot bringup.launch.py arm_hardware:=true follow:=true

# navigation only (RViz goals / coverage):
ros2 launch ackermann_robot bringup.launch.py arm_hardware:=true

# checked session start (kills zombies, clears DDS shm, I2C check, health gate):
bash ackermann_robot/tools/preflight_session.sh
```

Always launch from your own terminal (keep Ctrl-C in your hands). Builds:
`colcon build` **only from `~/ros2_ws`** — building from a package directory
creates nested `build/install/log` and silently ships nothing; verify with
absolute `~/ros2_ws/install/...` paths and mtimes.

RViz markers from the tracker: **blue dots** = leg-shaped clusters,
**yellow cylinder** = unconfirmed candidate (never followed), **green** =
confirmed person, **red** = the person being followed.

### Operator tools

| Tool | Purpose |
|---|---|
| `ackermann_robot/tools/preflight_session.sh` | clean armed session start with health gate |
| `ackermann_robot/tools/auto_coverage_drive.py` | autonomous goal generator (survey, relocation, 3 m tether) |
| `ackermann_robot/tools/tracker_replay.py <bag>` | rerun recorded scans through the tracker offline (param overrides) |
| `ackermann_robot/tools/drspaam_bench.py <bag> <ckpt>` | score the neural referee against a recorded session |
| `ackermann_robot/tools/three_point_drill.py` | scripted steering-capability measurement |
| `ackermann_robot/tools/amnesia_exam.sh` | wipe memory + falsify config to re-certify from-scratch learning |
| `ackermann_robot/tools/drive_scorecard.py` | KPI report from a session's flight-recorder CSV |
| `adaptive_ackermann/tools/steering_map_challenger.py` | offline monotone steering-map refit from CSVs |

## How it learns (short version)

Everything is learned as *measured behavior*, gated by attribution rules
that decide whether a failure is really evidence:

- **Breakaway effort** per direction — EMA over launches, variance-damped,
  booked only once the robot demonstrably rolls (≥ 0.10 m/s; floor squirm
  is not breakaway). Adapts to carpet vs wood within a few launches.
- **Throttle maps + trims + effort scale** — feedforward-dominant speed
  control; effort scale is a Kalman gain tracking battery sag. Adaptation
  is odom-only and the robot never stops for "low battery" — a stop is
  earned only when max authority cannot move it.
- **Steering map** (per-side curvature) — RLS shadow + offline challenger
  refits, isotonic-projected at load, promoted only against ground truth;
  live steering-lag estimator budgets speed to keep the steering loop
  stable.
- **Planner trackability** — learned per-branch curvature the robot can
  actually track. Failures count only if tracking was genuinely bad, at
  demands near the believed limit, consolidated across sessions (each of
  those three gates exists because of a real poisoning incident).
- **Person tracker** — odom-frame persistence grid learns the room's
  static furniture in seconds; people confirm by sustained walking; the
  DR-SPAAM referee (runs from `~/venvs/drspaam`, checkpoint in
  `~/opt/drspaam_ckpts/`) ranks person-ness and arbitrates identity
  relatively — it outranks itself, never absolutely.

Learned state lives in `~/.robot/*.yaml` (runtime memory, steering map,
trackability). Deleting them = amnesia; the robot re-learns. Flight
recorder CSVs land in `~/.robot/drive_logs/`, session rosbags in
`~/.robot/bags/` — both purely diagnostic, safe to purge.

## Troubleshooting

- **Robot won't move, `command watchdog` in the CSV** — nothing is
  publishing commands (follower holding / Nav2 idle). Not a fault.
- **Frozen after a crash** — Nav2 nodes respawn and re-activate on their
  own (bonds on); if the driver died, the deadman already neutralized the
  wheels.
- **Ghost "people" on furniture** — check the referee is alive
  (`/person_referee/detections`); geometry alone cannot separate a static
  human from a chair.
- **Diagnosing a bad run** — flight CSV (`~/.robot/drive_logs/`), tracker
  and follower logs (`~/.ros/log/`), then replay the session bag through
  `tools/tracker_replay.py` — in that order.

Project history (old plans, experiment data, retired docs) lives in git
history, not in the working tree.
