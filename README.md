# ackermann_robot

Clean ROS 2 package retained in the original Git repository so the previous
implementation remains available in history.

The package now contains a measured robot description, lidar/MOLA bringup,
isolated drivetrain experiments, and an experimental adaptive Nav2 controller.
The integrated actuator results, safety limits, architecture, and operating
procedure are in
[`docs/throttle-steering-adaptive-control.md`](docs/throttle-steering-adaptive-control.md).

## Known robot measurements

All dimensions below came from the previous robot description. Values marked
as estimates should be measured again before they are used in a new model.

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

Reference frame used previously: the rear axle center projected onto the
ground, with positive X forward, positive Y left, and positive Z upward.

| Sensor center | X | Y | Z | Yaw |
|---|---:|---:|---:|---:|
| RPLIDAR C1 | 0.237 m | 0 m | 0.165 m | 180 degrees |
| RealSense D435i prism | 0.2975 m | 0 m | 0.1275 m | 0 degrees |

The old Nav2 configuration used a 0.43 x 0.28 m planning footprint and a
1.3 m minimum turning radius. These were planning/tuning values, not direct
physical measurements.
