# adaptive_ackermann

Self-learning controller for encoder-less Ackermann vehicles. Given only a
**birth certificate** (geometry, sensor mounting, actuator channels/neutrals/
safe bounds — no calibration, no polarity), it learns to drive from ordinary
navigation goals: steering maps, breakaway, a two-point throttle feedforward
line, correction sizing, polarities, steering dynamics, and battery/surface
adaptation, all from odometry alone.

## Architecture (2026-07-13 hardware split)

```
any commander (Nav2, teleop, ...)          any odometry source
        |  /cmd_vel  geometry_msgs/Twist           |  /odom  nav_msgs/Odometry
        v                                          v
   +---------------------------------------------------+
   |  adaptive_ackermann_controller   (this package)   |
   |  physics -> normalized effort; ALL learning here  |
   +---------------------------------------------------+
        |  /actuator_effort  adaptive_ackermann_msgs/EffortCommand
        v
   hardware driver (e.g. pca9685_effort_driver)
   effort -> wire; deadman; neutral-on-error; zero intelligence
```

Mirrors the ros2_control controller / hardware-interface split without
adopting the framework (no encoders, Python learners).

## The effort contract

`EffortCommand.steering_effort` / `.drive_effort` are dimensionless in
`[-1, 1]`:

- `0` = the actuator's **declared neutral** (birth certificate)
- `+1` / `-1` = the declared safe extremes
- mapping is piecewise-affine per side of neutral (spans may be asymmetric)
- **the sign carries no semantic** — which sign drives forward or steers left
  is a property of the vehicle's wiring and is *learned* (polarity
  discovery), never declared

A driver must: clamp to `[-1, 1]`, apply its fixed affine map to the
substrate (PWM microseconds, H-bridge duty+direction, ...), hold **neutral
when commands go stale** (`reference_timeout_s`, stamped messages), and go
neutral-then-release on any error and on shutdown. That is the entire
contract; a new driver is ~100 lines.

## Nodes

- `adaptive_ackermann_controller` — the learning controller.
  In: `/cmd_vel` (post-safety Twist), `/odom` (any planar source >= 10 Hz,
  honest header stamps), `/scan` (final guard). Out: `/actuator_effort`,
  `/speed_limit` (nav2_msgs/SpeedLimit), `/controller/debug` (JSON),
  `/diagnostics`, CSV flight recorder in `~/.robot/drive_logs/`.
  Learned memory persists in `~/.robot/` (version >= 8, effort units).
- `path_segment_dispatcher` — Nav2 FollowPath proxy handling Reeds-Shepp
  cusps one direction at a time + trackability envelope learning. Only
  needed with Nav2.

## Offline tools

- `tools/steering_map_challenger.py` — refit the monotone steering map from
  flight-recorder CSVs (`--bootstrap` writes the geometry-prior map for a
  fresh vehicle; `--since STAMP` restricts training data).
- `tools/drive_scorecard.py` — per-session KPI scorecard.

Pre-2026-07-13 CSVs are pulse-denominated and are automatically ignored
(column names changed with the units). Old `~/.robot` memory is converted
once by `ackermann_robot/tools/convert_memory_to_effort.py`.
