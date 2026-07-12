# Throttle, steering, and adaptive Nav2 control

## Purpose and status

This is the self-contained record of the actuator experiments and the first integrated driving controller. The raw CSV files are useful for auditing but are not required to understand the controller. It is an experimental low-speed controller using MOLA lidar odometry, not wheel encoders, and is intended for supervised Nav2 goals in the local `odom` frame.

## Hardware and conventions

- PCA9685: I2C bus 1, address `0x40`, prescale 30, about 196.9 Hz.
- ESC channel 14; steering channel 12.
- ESC neutral is 1500 us; below it is forward and above it is reverse.
- Steering electrical guard is 1000--2000 us; increasing pulse produces positive curvature.
- Wheelbase is 0.2775 m and track is 0.247 m.

The bidirectional RC ESC has a neutral region and static-friction breakaway. A large pulse may be required to start, while a smaller one sustains motion. Therefore one static pulse-to-speed curve is insufficient.

## Measurement method

MOLA publishes `odom -> base_link` TF. Experiments integrated displacement along the body heading and unwrapped yaw. Steering curvature was measured over traveled distance:

```text
kappa_measured = delta_yaw / distance
steering_angle = atan(wheelbase * kappa_measured)
```

Using distance separates steering geometry from speed. Each important candidate was driven forward and reverse. Their difference was large enough that one shared steering curve was rejected. Static odometry noise was small enough to detect motion with a threshold and repeated samples. Production steering learning uses at least 0.25 m and never learns during breakaway or traction recovery.

## Throttle results

Tests progressed through static-noise recording, forward and bidirectional breakaway ramps, repeated kick/sustain/coast trials, closed-loop speed, carpet, and mixed carpet/floor transitions. They established:

1. Breakaway and sustain are different regimes.
2. A memorized kick is unreliable as battery voltage and surface change.
3. An informed ramp until odometry confirms motion adapts better.
4. If motion disappears, effort should increase in place rather than restart the whole segment.
5. After breakaway, bounded PI speed correction reduces pulse-stop-pulse behavior.
6. Fresh odometry near 10 Hz is adequate for slow control when output is held between samples, but not for wheel-slip estimation.

Retained steady-state anchors are:

| Direction | Speed (m/s) | ESC pulse (us) |
|---|---:|---:|
| Forward | 0.20 | 1419.38375 |
| Forward | 0.25 | 1417.74321 |
| Reverse | 0.20 | 1592.50878 |
| Reverse | 0.25 | 1593.52498 |

The controller interpolates and cautiously extrapolates these anchors, applies
separate bounded forward/reverse trims, and enters a ramp after three low-motion
samples. Rolling authority is bounded at 1380 us forward and 1640 us reverse.
Recovery may extend to tested immutable limits 1180 and 1820 us, then gives up
after dwelling at the limit. The Nav2 target is now 0.30 m/s, with controller
envelopes of 0.35 m/s forward and 0.30 m/s reverse. A measured speed above
0.55 m/s, direction mismatch, stale inputs, or obstacles commands neutral.

The 50% speed increase from the earlier 0.222 m/s rolling average is supported
by recorded samples between 0.35 and 0.45 m/s: lidar curvature residual did not
degrade in that bin. At 0.33 m/s the 10 Hz C1 advances about 0.033 m per scan,
and MOLA linear deskew remains enabled. Nine recorded neutral-stop events gave a
median deceleration of 0.593 m/s2 and a conservative lower-quartile value of
0.453 m/s2. Directional stopping clearance is therefore computed from current
speed using the configured 0.45 m/s2 minimum deceleration and 0.10 s scan
reaction time, never less than 0.20 m.

## Steering results

Lifted-wheel tests established electrical direction but could not identify Ackermann curvature. Moving forward/reverse blocks and longer side-to-side staircases produced repeatable curvature. Cross-validation showed the value of direction-specific fits: a retrace error fell from about 0.23 m and 9.6 degrees to 0.047 m and 0.27 degrees.

The adaptive limit search increased pulse only while measured curvature continued increasing. The final low-side results were:

| Pulse (us) | Forward κ (1/m) | Reverse κ (1/m) | Mean κ (1/m) |
|---:|---:|---:|---:|
| 1056 | -1.2449 | -1.1913 | -1.2181 |
| 1006 | -1.3421 | -1.2926 | -1.3174 |
| 1000 | -1.3504 | -1.2904 | -1.3204 |

The upper side reached a credible plateau:

| Pulse (us) | Forward κ (1/m) | Reverse κ (1/m) | Mean κ (1/m) |
|---:|---:|---:|---:|
| 1930.54 | 1.8285 | 1.6089 | 1.7187 |
| 1980.56 | 1.9934 | 1.7519 | 1.8727 |
| 2000 | 1.9758 | 1.7594 | 1.8676 |

The complete production lookup tables are in `config/adaptive_controller.yaml`, combining the dense low-side run and final guard runs. Zero curvature was interpolated at 1451.54 us forward and 1475.46 us reverse. This direction-dependent center is measured rolling hysteresis.

Hardware demonstrated more, but production is limited to `|κ| <= 1.15 1/m`, a minimum centerline radius of 0.87 m. This leaves margin from servo endpoints, the low electrical guard, and growing forward/reverse disagreement at high curvature.

## Integrated adaptive controller

Nav2 publishes body velocity. For nonzero linear velocity:

```text
kappa_command = angular.z / linear.x
```

The bridge clamps speed and curvature, selects the forward/reverse steering table, and interpolates curvature to pulse. This preserves the Ackermann interface: Nav2 plans feasible geometry; `angular.z` is not treated as servo angle.

Steering identification runs continuously during ordinary Nav2 goals. A rolling
0.55 s pose window measures curvature as yaw change divided by signed distance.
The estimator compares that response with the average commanded steering pulse
at candidate delays from 0.0 through 0.5 s and learns the most credible effective
actuator delay. Recursive least squares then identifies
`measured_curvature = gain * baseline_curvature + bias` independently for
forward/reverse and negative/positive steering. Breakaway, recovery, stops,
direction changes, low speed, insufficient distance, and implausible curvature
are rejected.

Each identified branch begins in shadow mode. It must accumulate at least 12
accepted observations, and confidence also depends on prediction residual. The
applied inverse-model compensation is blended by that confidence, so a noisy or
degrading model automatically falls back toward the experiment-derived map.
Gain, bias, curvature, pulse, and electrical limits are projected into fixed
bounds. Version-2 trims learned during the earlier fragmented Collision Monitor
run are intentionally not imported.

While the servo is traversing to a new steering pulse, the bridge temporarily
reduces requested speed instead of driving hard along curvature the wheels have
not reached. `/speed_limit` remains deterministic during identification; feeding
model residual directly back into speed made cause and effect ambiguous. Runtime
models are stored atomically at `~/.robot/adaptive_ackermann_runtime.yaml`;
baseline experiment maps, electrical guards, and maximum curvature never change.

Runtime model version 5 retains the probability distribution over candidate
steering delays rather than choosing a fixed bin. Exponentially weighted
prediction errors become evidence weights; their weighted mean is the delay and
their normalized entropy is confidence. Ambiguous evidence therefore produces
little or no preview compensation.

For a fresh `/plan` in `odom`, the controller projects the robot along the path
by `abs(speed) * (learned_delay + remaining_command_slew_time)`. It evaluates
velocity-scaled pure-pursuit carrot geometry at the present and projected path
poses. The difference between actual RPP curvature and simulated present RPP
curvature remains as feedback; only path feed-forward is led. Compensation is
blended by learned delay confidence, cannot cross a forward/reverse cusp, and
cannot expand curvature or electrical limits. A missing, stale, wrong-frame, or
direction-inconsistent path disables preview automatically.

Throttle PI uses conditional anti-windup. Integration freezes during steering
traversal, reversal, startup, recovery, stops, and repeated odometry. At a
rolling actuator bound it uses the existing `Ki/Kp` ratio for back-calculation,
preventing sustained carpet load from winding the trim farther into saturation.
Active rolling effort changes by at most 6 us per fresh odometry observation,
as validated in the earlier experiment controller; safety neutral remains
immediate. Proportional gain is derived from the local slope of the measured
forward/reverse throttle map instead of an unrelated constant. Version-4
throttle trims are not migrated because they were learned during the observed
1380-us/overspeed/neutral limit cycle; steering and delay evidence is preserved.

Runtime version 6 also learns causal breakaway effort. Successful startup logs
showed that motion was detected about one learned response delay after the pulse
that caused it: 31 forward starts gave a median causal 1403 us and 35 reverse
starts gave 1611 us. These are soft priors, not fixed kicks. Startup approaches
the learned threshold over one response delay, holds as soon as raw motion
appears, and gives every stronger effort level a full response interval before
escalating. If surface or battery conditions require more, effort continues in
hardware-tick increments that grow with consecutive failed attempts. Each
successful launch updates the direction-specific causal threshold; traction
recovery remains faster than ordinary startup.

This structure follows the practical findings in steering-system identification
literature: actuator delay must be represented, recursive estimation can update
vehicle-specific parameters online, and adaptation should remain bounded under
input/state constraints. Nav2 still owns path generation and geometric feedback;
the learned low-level inverse makes actual curvature better match the curvature
that Regulated Pure Pursuit already requests.

## Nav2 architecture

```text
RViz goal -> BT Navigator
 -> Smac Hybrid-A* (Reeds-Shepp, Rmin 0.87 m)
 -> Regulated Pure Pursuit (reverse allowed, no rotate-in-place)
 -> /cmd_vel_nav -> Collision Monitor -> /cmd_vel
 -> adaptive_ackermann_controller -> PCA9685

MOLA odom->base_link TF -> controller -> /odom -> Nav2
LaserScan -> costmaps + Collision Monitor + controller final guard
```

Both costmaps are rolling windows in `odom`. This supports local drive-and-learn tests without a map; it is not global localization. The custom behavior tree removes rotate-in-place and retains clearing, reverse backup, and wait recovery.

## Footprint versus turning and inflation radius

The robot footprint used for collision checking is the measured rectangle
`[[-0.10,-0.16], [0.40,-0.16], [0.40,0.16], [-0.10,0.16]]` in `base_link`.
Two other parameters happen to use the word radius but do not make the robot
circular:

- `minimum_turning_radius: 0.87` is the radius of an Ackermann path measured at
  the rear-axle center. Smac uses it to avoid paths the steering cannot follow.
- `inflation_radius: 0.50` is an obstacle-cost field drawn outward from every
  edge of the rectangular footprint. It covers the rectangle's approximately
  0.444 m circumscribed radius and enables Smac's optimized collision checking.

## Running the supervised experiment

```bash
cd ~/ros2_ws
colcon build --packages-select ackermann_robot
source install/setup.bash
```

One launch now starts the robot description, RPLIDAR, MOLA odometry, Nav2,
Collision Monitor, and adaptive controller. First run with actuator output
disabled:

```bash
ros2 launch ackermann_robot bringup.launch.py
```

After confirming `/scan`, `/odom`, diagnostics, and planning, stop it and explicitly arm:

```bash
ros2 launch ackermann_robot bringup.launch.py arm_hardware:=true
```

Bringup records a timestamped rosbag under `~/.robot/bags` by default. Disable
it only when storage is constrained:

```bash
ros2 launch ackermann_robot bringup.launch.py arm_hardware:=true record_bag:=false
```

Keep the car lifted during the three-second ESC initialization, then put it down before sending a short nearby goal. Start with goals under 2 m in a clear area and remain beside the power switch. Ctrl-C, command timeout, stale scan/odometry, Collision Monitor, the speed-dependent directional clearance guard, overspeed, direction mismatch, and exhausted traction authority all command neutral.

The controller's final scan guard transforms every point from the measured
lidar pose (`x=0.237 m`, yaw `pi`) into `base_link`, rejects returns inside the
rectangular footprint, and measures directional clearance from the front or
rear footprint edge. Raw lidar range is not clearance because the sensor is
offset and reversed; treating it as such previously caused repeated false
reverse-obstacle stops on a roughly 0.18 m self-return.

```bash
ros2 topic echo /controller/debug
ros2 topic echo /diagnostics
ros2 topic echo /controller/limits --once
ros2 topic echo /collision_monitor_state
```

Analyze path-preview command prediction in a recorded bag with:

```bash
ros2 run ackermann_robot analyze_preview_replay ~/.robot/bags/<bag-directory>
```

This is a causal software diagnostic. Actual benefit is still judged from
early-versus-late cross-track, heading, and delayed-curvature errors because an
old bag cannot contain motion from a counterfactual command it never executed.

Do not run old experiment nodes concurrently: PCA9685 must have one owner.

## Questions for the first Nav2 drives

- Does curvature error shrink independently by direction and side?
- Does throttle settle without repeated recovery pulses across floor and rug?
- Does the planner request feasible reverse maneuvers rather than spins?
- Are stops attributable in diagnostics to collision, watchdog, progress, overspeed, or traction authority?
- Does lidar odometry remain reliable at maximum permitted curvature?

If these pass, add mapped localization and longer goals next; do not expand the electrical or curvature limits.
