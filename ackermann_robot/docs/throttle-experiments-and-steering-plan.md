> **HISTORICAL** (frozen 2026-07-15) — superseded by `stack-review-2026-07-15.md` and `how-the-learner-works.md`; kept for provenance. Verdicts here may be stale.

# ESC throttle experiments and steering-learning plan

## Purpose and status

This document is the self-contained record of the July 2026 drivetrain
experiments on the orange Ackermann robot. It records the hardware assumptions,
equations, safety rules, raw findings, failed approaches, final experimental
controller, and the next steering-identification program. No value required to
understand or repeat the work is intentionally delegated to another document.

The throttle experiments established a workable low-speed controller using only
lidar odometry. They did **not** create the production `/cmd_vel` controller.
The Python programs remain explicitly armed experiments and still write the
PCA9685 directly.

## Robot and actuator facts

| Item | Value used in the experiments |
|---|---:|
| ROS version | ROS 2 Jazzy |
| Computer | Raspberry Pi 5 |
| Drivetrain | Four-wheel RC drivetrain, one brushed motor |
| Steering | Front Ackermann linkage with hobby servo |
| Wheelbase, rear to front axle | 0.2775 m |
| Track width, wheel-center to wheel-center | 0.247 m |
| Wheel diameter | 0.120 m |
| Motor controller | Redcat Volcano-style brushed forward/reverse ESC |
| Likely ESC family | Hobbywing QuicRun WP-1060 or equivalent |
| PWM generator | PCA9685, I2C bus 1, address `0x40` |
| ESC channel | 14 |
| Steering channel from prior hardware session | 12; re-verify before use |
| Neutral ESC signal | 1500 us |
| PCA9685 prescale | 30 |
| Effective frame rate | about 196.9 Hz |
| Pulse resolution | about 1.24 us per PCA9685 tick |
| Current wiring sense | below 1500 us moves forward; above moves reverse |
| Odometry | MOLA 2D lidar odometry, `odom -> base_link` TF |
| Fresh odometry rate | about 10 Hz |

The ESC identity is an inference from the vehicle, electrical ratings, jumper
modes, and earlier records. The WP-1060 manual documents Forward/Brake/Reverse,
Forward/Brake, and Forward/Reverse modes. F/R uses immediate reverse; F/B/R uses
a brake-neutral-reverse sequence. Its 60 A forward and 30 A reverse ratings are
current limits, **not** proof that reverse throttle should be scaled by 0.5.
The experiments measured both directions rather than assuming symmetry.

References:

- [Hobbywing QuicRun brushed ESC manual](https://www.hobbywing.com/en/uploads/file/20221015/f60b7ebe160a7b283927ae8916d36763.pdf)
- [Official WP-1060 product information](https://www.hobbywing.com/en/products/quicrun-wp-1060-brushed55)
- [Redcat Volcano EPX product information](https://www.redcatracing.com/products/volcano-epx-1-10-scale-electric-monster-truck)

## Safety boundary used during testing

Every hardware experiment required an explicit `--arm` argument. The software
sent neutral on startup, normal exit, Ctrl-C, and handled exceptions. Later
tests also enforced:

- fresh-odometry timeout of 0.5 s;
- front or rear lidar clearance, usually 0.20 m for the final rug tests;
- per-segment distance limit, finally 3.0 m;
- emergency signed-speed limit, finally 0.40 m/s;
- bounded forward and reverse pulse authority;
- at most two high-load recoveries in one segment;
- neutralization when recovery timed out or another stall followed two
  recoveries.

These are software protections, not a hardware dead-man. A killed process,
wedged I2C bus, kernel failure, or power fault can leave the PCA9685 repeating
its last pulse. The PCA9685 output-enable pin must still be wired to a pulled-up
GPIO before autonomous operation around people.

## Measurement equations

### PCA9685 pulse generation

With oscillator frequency `f_clock = 25 MHz` and prescale `P = 30`:

```text
frame_rate = f_clock / (4096 * (P + 1)) ~= 196.9 Hz
us_per_tick = 1e6 / frame_rate / 4096 ~= 1.24 us
tick = round(pulse_us / us_per_tick)
```

Fractional pulse commands use first-order sigma-delta dithering between
adjacent integer ticks. Neutral is never dithered.

### Signed longitudinal velocity from TF

For consecutive planar poses, with displacement `(dx, dy)`, robot heading
`psi`, and elapsed time `dt`:

```text
v_longitudinal = (dx*cos(psi) + dy*sin(psi)) / dt
```

The experiment applies a light recursive estimate:

```text
v_est = 0.75*v_est_previous + 0.25*v_longitudinal
```

Control output is recomputed only when a **fresh** 10 Hz TF measurement arrives.
The PCA9685 output is held between measurements. Earlier 56 Hz feedback reused
each stale measurement five or six times and created a delayed limit cycle.

### Robust stationary-noise calibration

At neutral, the final controller records about 30 fresh velocity samples:

```text
center = median(v)
MAD = median(abs(v - center))
sigma ~= 1.4826 * MAD
movement_threshold = clamp(4*sigma, 0.020, 0.060) m/s
recovery_threshold = movement_threshold + max(0.015, 2*sigma)
```

One final rug run measured:

```text
sigma = 0.00519 m/s
movement threshold = 0.02074 m/s
recovery threshold = 0.03574 m/s
```

A separate 15 s stationary test earlier measured about 0.0087 m/s standard
deviation in finite-difference velocity, 0.95 mm final position drift, and
1.74 mm maximum position excursion.

### Throttle feedback

The rolling controller is feed-forward dominant:

```text
error = target_speed - measured_signed_speed
desired_pulse = feedforward_pulse - Kp*error
```

The same sign works in both directions: a smaller pulse adds forward authority,
while a larger pulse adds reverse authority. `Kp` is derived conservatively
from half the local slope magnitude of the pulse/speed map and bounded to
15--35 us per (m/s). Gain scheduling increases correction smoothly during
overspeed instead of switching between full traction and neutral.

## Experiment progression and findings

### 1. Stationary odometry

The robot was held still while `odom -> base_link` was recorded. Position drift
was small enough for movement detection, but velocity obtained by differencing
TF was noisy around zero. A 0.04--0.05 m/s movement test was initially used;
the final implementation derives it from the neutral noise distribution.

### 2. First forward pulse sweep

Starting at 1500 us, pulse width was decreased slowly. The robot remained still
through approximately 1404 us, moved at about 0.17 m/s near 1403 us, and reached
about 0.38 m/s near 1402 us. This revealed a narrow engagement region and the
difference between static breakaway and rolling sustain.

An early implementation checked distance only between 1.5 s steps and allowed
about 0.62 m travel despite a 0.20 m intended limit. All later limits were
checked continuously.

### 3. Fractional dithering and sustain tests

Sub-microsecond average pulses were created by alternating adjacent PCA9685
ticks. Dithered 1404 us reliably broke forward static friction. Kick-and-hold
tests showed that short 0.5--1.0 s holds confused kick acceleration, odometry
delay, and true steady speed. Two-second holds provided much better rolling
maps.

### 4. Independent bidirectional breakaway

Forward and reverse were ramped independently from neutral. The first reliable
thresholds were about 1405 us forward and 1600 us reverse. Six alternating
repeats measured:

```text
forward: 1407.5, 1407.0, 1401.0 us
reverse: 1597.5, 1601.0, 1609.5 us
```

Mean deviations from neutral were about 94.8 us forward and 102.7 us reverse.
This was broadly symmetric and rejected the earlier hypothesis of a fixed
2--2.5x reverse scaling.

### 5. Open-loop kick/sustain/coast identification

Each trial used a measured breakaway kick, one candidate rolling pulse, and a
neutral coast. Longer runs identified useful low-speed regions:

```text
forward rolling region near 0.20--0.30 m/s: about 1413--1415 us
reverse rolling region near 0.20--0.30 m/s: about 1593.5--1596.5 us
```

The exact pulse moved with surface, battery, temperature, and drivetrain load.
It therefore became a feed-forward anchor, never an open-loop promise.

### 6. Closed-loop speed control

Several approaches were tested:

1. PI at about 56 Hz on repeated 10 Hz measurements caused oscillation.
2. Drive/neutral hysteresis bounded speed but caused visible stop-and-go motion.
3. Shorter neutral pulses removed complete stops but retained ripple.
4. P-only control updated on fresh 10 Hz odometry was smoothest.

The decisive comparison was that fresh-sample control produced roughly
0.011--0.026 m/s settled standard deviation with continuous output, while the
governor experiments reached about 0.05 m/s variation and visible stops.

An offline replay compared raw MOLA velocity, exponential smoothing, and a
position-derived alpha-beta observer. Validation results were:

| Method | Position prediction RMSE | Tracking RMSE | Velocity-step RMS |
|---|---:|---:|---:|
| Raw MOLA | 0.00691 m | 0.0433 m/s | 0.0174 m/s |
| EMA | 0.00691 m | 0.0433 m/s | 0.0174 m/s |
| Alpha-beta | 0.00607 m | 0.0558 m/s | 0.0365 m/s |

The optimal EMA gain was 1.0: use the fresh raw measurement. No odometry filter
was adopted.

### 7. Persistent learning and surface transfer

The model at `~/.robot/esc_model.yaml` stores breakaway history and conservative
baseline feed-forward pulses. A short-lived session trim is separated from the
baseline. Recovery, overspeed, high-acceleration, and high-error samples cannot
update rolling feed-forward.

This separation was necessary because an earlier learner changed the +0.25 m/s
baseline from about 1417.3 to 1410.5 us after carpet transitions. Reusing that
high-load pulse on easier floor produced repeated acceleration. Persistent
updates are now deliberately conservative; surface load belongs in session
feedback, not the long-term baseline.

### 8. Rug-edge adaptation

A segment was lengthened to 8 s and 3 m maximum displacement so both axles could
encounter the rug. The controller evolved into explicit physical states:

```text
STARTUP_RAMP
ROLLING
TRACTION_RECOVERY
POST_RECOVERY_GUARD
OVERSPEED_GAIN_SCHEDULE
FAULT_NEUTRAL
```

- A stall is three consecutive fresh samples below the measured movement
  threshold.
- Recovery continues from the current rolling pulse; it does not neutralize
  and restart.
- Recovery adds about 2.5 PCA9685 ticks per fresh sample, bounded by an
  absolute safety pulse and 22 samples.
- At most two recoveries are allowed per segment, representing separate axle
  load events.
- Slight motion is not proof that an axle crested. Traction success requires
  `max(recovery_threshold, 0.5*abs(target_speed))`.
- After cresting, a short guard sheds traction until fresh-sample velocity is
  no longer increasing or six samples elapse.
- A third stall, timeout, stale odometry, obstacle, distance, or speed fault
  sends neutral.

The final complete rug run handled two forward recovery events in one segment
at about 1380 and 1391 us, another forward event near 1382 us, completed four
long segments, and stayed below the 0.40 m/s emergency cutoff. Rug crossings
still have unavoidable velocity variation while an axle is blocked and then
crests; ordinary rolling remains much smoother.

## Final experimental throttle model

Representative baseline pulses at the end of testing were approximately:

```text
neutral: 1500 us
forward +0.20 m/s: 1419.4 us
forward +0.25 m/s: 1417.7 us
reverse -0.20 m/s: 1592.5 us
reverse -0.25 m/s: 1593.5 us
```

These are starting points, not universal constants. The model applies
direction-specific interpolation, session trim, bounded feedback, startup
breakaway search, and high-load recovery. Learning cannot expand the absolute
safety envelope.

## Known limitations and unresolved engineering risks

1. The experiments are monolithic and directly own I2C. Production must split
   controller logic from a sole-owner actuator driver with a command watchdog.
2. The ESC is fed at about 197 Hz to gain pulse resolution. Standard RC input
   is commonly around 50 Hz, and the ESC manual does not explicitly guarantee
   this input rate. A future hardware-PWM 50 Hz A/B test is warranted.
3. Sigma-delta pulse dithering assumes the ESC/mechanics average adjacent pulse
   widths. It worked empirically but is not guaranteed by the ESC manual.
4. MOLA is a 10 Hz delayed motion sensor, not a wheel encoder. Feed-forward must
   remain dominant and feedback bounded.
5. A physically trapped chassis must fault rather than learn unlimited torque.
6. The hardware PCA9685 OE dead-man is still absent.

## Steering-learning objective

The steering system must learn a map from steering-servo pulse to achieved
curvature, including center offset, left/right asymmetry, settling time,
hysteresis/backlash, speed and traction dependence, and maximum useful
curvature. Throttle and steering are coupled in both directions: curvature is
observable only while the robot is moving, steering load changes longitudinal
speed, and tire force or servo-saver deflection can make the same steering
pulse produce different curvature at different speeds and loads.

For the low-speed bicycle model:

```text
kappa = omega / v
delta_effective = atan(L * kappa)
omega_expected = v * tan(delta_command) / L
turn_radius = 1 / abs(kappa)
```

where `L = 0.2775 m`, `v` is signed longitudinal velocity, `omega` is signed
yaw rate, `kappa` is path curvature, and `delta` is the effective front-wheel
angle. Curvature, not yaw rate alone, is the steering observable; otherwise a
speed transient looks like steering error.

A complete circle is not required. Over one short settled leg, unwrap heading
and accumulate signed longitudinal distance:

```text
delta_s = sum(v_i * dt_i)
delta_psi = unwrap(psi_end - psi_start)
kappa_leg = delta_psi / delta_s
```

When reversing with the steering held fixed, `delta_s` and `delta_psi` both
change sign, so their ratio estimates the same geometric curvature. Robustly
pooling several short forward and reverse legs provides the information of a
long arc while keeping the robot inside a narrow test area.

MuSHR uses a gain-and-offset steering map and calibrates gain from measured
turn geometry at low speed. It explicitly warns that one linear map may not be
accurate over a wide range. Our linkage has servo-saver compliance and known
left/right asymmetry, so separate monotone branches are more appropriate.

References:

- [MuSHR tuning guide](https://mushr.io/tutorials/tuning/)
- [MuSHR system overview](https://mushr.io/tutorials/overview/)
- [F1TENTH odometry and steering calibration guide](https://f1tenth.readthedocs.io/en/ros1/getting_started/driving/drive_calib_odom.html)
- [System identification and control of front-steered Ackermann vehicles](https://arxiv.org/abs/2308.03898)

## Steering experiment plan

### Phase S0: physical and electrical safety

1. Confirm steering is on PCA9685 channel 12.
2. With wheels lifted, confirm pulse polarity and center without reaching
   mechanical stops.
3. Begin from the prior safe envelope only as a hint: about 1137--1860 us with
   1500 us center at the present frame rate. Re-measure before saving it.
4. Establish immutable servo pulse bounds inside linkage stops.
5. Rate-limit steering pulses and release the servo after stationary idle.
6. Never learn wider endpoints automatically.

### Phase S1: learn rolling straight center

Mechanical visual center is not necessarily dynamic straight-ahead center.

1. Use the validated throttle controller near 0.20 m/s on a flat surface.
2. Start near 1500 us and run short forward/reverse shuttle legs with steering
   held at the same pulse through the direction change.
3. Ignore startup, braking, and reversal samples. Pool only settled portions
   from several legs using `delta_psi/delta_s`.
4. End a leg on clearance/distance limits or once it has accumulated enough
   heading change for a confident estimate; do not require an arbitrary full
   circle or 2--3 m single pass.
5. Adjust steering pulse with bounded bisection until pooled curvature overlaps
   the straight-motion curvature noise band.
6. Repeat each candidate after approaching it from both lower and higher servo
   pulses. Save both dynamic centers and their median rather than hiding a
   repeatable hysteresis band.

### Phase S2: structured left/right excitation

Use short forward/reverse shuttles at each steering condition. Test randomized
steering levels so battery, temperature, and floor position are not correlated
with command:

```text
center, small-left, small-right, medium-right, medium-left,
large-left, large-right, then repeats
```

For every target pulse:

1. With throttle neutral, move to a known pulse below the target, then approach
   the target by increasing pulse. Run one forward and one reverse leg without
   changing steering.
2. Return to neutral throttle, move to a known pulse above the same target,
   then approach by decreasing pulse. Repeat the two legs. These are distinct
   hysteresis observations; do not recenter between them.
3. Start each leg with the learned adaptive throttle ramp, then hold steering
   unchanged while longitudinal speed settles.
4. Stop adaptively when accumulated heading change is informative, or at the
   clearance, displacement, time, or speed limit. Small steering angles require
   more pooled legs; large angles require shorter legs.
5. Reject startup, braking, reversal, traction-recovery, overspeed, stale-TF,
   excessive-acceleration, obstacle, and low-speed samples.
6. Log target pulse, prior pulse, pulse approach direction, vehicle direction,
   requested speed, ESC pulse, measured speed, acceleration, yaw, yaw rate,
   signed distance, curvature, effective angle, pose, clearance,
   surface/session, controller state, and confidence.
7. Alternate left/right targets and forward/reverse legs to keep net pose drift
   small and avoid one-sided tire or servo heating. Recenter the test pattern
   manually if pose drift accumulates; do not let the learner compensate for a
   wall or rug interaction.

### Phase S3: separate throttle/load effects from steering hysteresis

Repeat a compact subset of center, small, medium, and large left/right targets
at controlled speeds near 0.15, 0.20, and 0.25 m/s. For every combination,
measure both increasing-pulse and decreasing-pulse approaches. Randomize the
order and repeat it so speed, battery discharge, floor location, and steering
history are not confounded.

The independent experimental variables are steering pulse, pulse approach
direction, requested speed, vehicle direction, and surface. ESC pulse is
logged as the throttle controller's observed effort; it is not treated as a
direct measure of motor torque. Compare settled curvature at the same steering
pulse and approach branch:

```text
hysteresis_effect = kappa(increasing pulse) - kappa(decreasing pulse)
speed_load_effect = kappa(high speed) - kappa(low speed)
direction_effect = kappa(forward legs) - kappa(reverse legs)
```

Use repeated-leg spread and stationary curvature noise to decide whether each
effect is real. If a difference is smaller than the repeatability band, keep a
simpler model. If curvature correlates with ESC effort after controlling for
speed and surface, the likely mechanism is load, tire slip, supply sag, or
servo-saver compliance; collect battery/BEC voltage later if hardware permits.

### Phase S4: fit the steering map

Fit achieved curvature as a function of pulse relative to learned center:

```text
kappa = f_left(center_us - pulse_us)
kappa = -f_right(pulse_us - center_us)
```

The exact signs must be verified physically. Use robust medians per pulse level,
then a monotone piecewise-linear or isotonic fit. Do not force left and right to
share gain. Do not force a single global line if residuals show saturation or
servo-saver wind-in. Add approach-direction branches or a speed term only when
the structured repeats show an effect larger than measurement variability.

Store:

```yaml
steering:
  center_us: ...
  center_from_left_us: ...
  center_from_right_us: ...
  settle_samples: ...
  hysteresis_significant: ...
  speed_load_significant: ...
  left_increasing:  [{pulse_us: ..., curvature_1pm: ..., confidence: ...}, ...]
  left_decreasing:  [{pulse_us: ..., curvature_1pm: ..., confidence: ...}, ...]
  right_increasing: [{pulse_us: ..., curvature_1pm: ..., confidence: ...}, ...]
  right_decreasing: [{pulse_us: ..., curvature_1pm: ..., confidence: ...}, ...]
  max_valid_curvature_1pm: ...
```

Backlash compensation is justified only if approach-direction curves differ
repeatably. Steering dither is not enabled merely because throttle dithering
worked; it can buzz, heat the servo, and excite the linkage.

### Phase S5: held-curvature shuttle validation

Choose unseen target curvatures inside the fitted range. Convert target
curvature to effective steering angle and pulse, drive short forward/reverse
shuttles, pool the settled legs, and measure:

```text
curvature bias
curvature RMS error
left/right symmetry
settling time
minimum turn radius
speed dependence
repeatability after direction reversals
```

Acceptance targets should initially be modest: no wrong-sign curvature,
monotone response, no mechanical saturation, and median curvature error below
15%. Tighten only after repeated sessions.

Validation must include the same target approached from both servo directions
and operated at more than one speed. It should also confirm that a
forward/reverse pair approximately returns the robot's heading and position;
large failure to retrace indicates slip, an odometry problem, or load-dependent
steering that the basic map cannot hide.

### Phase S6: bounded online steering trim

Production steering uses the learned branch as feed-forward and a small
curvature-domain correction:

```text
kappa_target = omega_command / v_command
kappa_measured = omega_measured / v_measured
error = kappa_target - kappa_measured
```

Update only on fresh odometry and only above the measured minimum reliable
speed. Freeze trim during steering motion, throttle recovery, overspeed,
reverse transitions, saturation, or poor TF. Maintain short-lived session trim
separately from the trusted baseline, exactly as learned in the throttle work.

Persistent baseline updates require repeated, settled observations and active
exploration that is not shaped by the current learned correction. Otherwise
the estimator can ratchet on its own output. Safety endpoints, steering rate,
maximum curvature, and collision constraints never adapt outward.

## Steering experiment acceptance and responses

| Observation | Response |
|---|---|
| Curvature monotone and repeatable | Fit branch and continue to unseen levels |
| Left/right gains differ | Keep independent branches; do not average them |
| Same pulse differs by approach direction | Fit bounded backlash branches |
| Forward and reverse `kappa` agree | Pool both vehicle directions |
| Forward and reverse `kappa` disagree repeatably | Keep direction term and inspect compliance/slip |
| Curvature grows for 1--2 s | Learn settling samples; reject transient data |
| `omega` changes but `omega/v` is stable | Speed coupling confirmed; use curvature |
| Curvature changes strongly with speed at the same history | Add speed-indexed map or compliance term |
| Curvature follows ESC effort after controlling speed | Investigate BEC voltage, servo saver, tire load, and slip |
| Response sign flips or is non-monotone | Stop and inspect linkage mechanically |
| Full-lock command stops increasing curvature | Record effective saturation; do not push harder |
| Servo buzzes or heats | Reduce rate/duty and disable dither |
| ICP becomes unreliable in tight circles | Increase radius/speed cautiously or use longer arc pose geometry |

## Steering results obtained

Short forward/reverse shuttles confirmed that the PCA9685 steering command is
held throughout motion and that measured curvature is approximately linear in
servo pulse through the useful range. The rolling center is about 1450 us. A
representative combined fit was:

```text
kappa ~= 0.00325 * (steering_pulse_us - 1450) 1/m
```

Forward/reverse fits moved the apparent center by roughly 20 us, while servo
approach-history hysteresis moved it by about 5--6 us. Direction is therefore
the larger correction. Representative settled results were:

| Steering pulse | Mean curvature | Effective angle | Radius |
|---:|---:|---:|---:|
| 1300 us | -0.492 1/m | -7.8 deg | 2.03 m |
| 1450 us | approximately zero | approximately zero | straight |
| 1600 us | +0.482 1/m | +7.6 deg | 2.07 m |
| 1230 us | -0.709 1/m | -11.1 deg | 1.41 m |
| 1670 us | +0.728 1/m | +11.4 deg | 1.37 m |
| 1180 us | -0.868 1/m | -13.5 deg | 1.15 m |
| 1720 us | +0.921 1/m | +14.3 deg | 1.09 m |

The old flat-floor throttle feed-forward caused every leg to decay below
0.05 m/s and re-enter breakaway recovery as steering tire scrub increased.
A bounded integral session trim, updated only on fresh odometry, removed these
mid-leg stops after its first forward/reverse learning pair. In the following
legs the learned trim reached about -7.3 us forward and +4.2 us reverse.

A final 1740 us forward test measured about +1.02 1/m over its settled portion
but then stopped at 0.41 m and could not restart even at the experiment's
maximum forward recovery effort of 1380 us. Because the reverse repeat never
ran, 1740 us is not validated and no larger angle is permitted. The current
experimental envelope is 1180--1720 us; a production controller should retain
additional margin inside it.

## Experiment programs and commands

Experimental source is isolated under `ackermann_robot/experiments`:

```text
esc_forward_calibrate.py   neutral-centered breakaway experiments
esc_system_id.py           kick/sustain/coast identification
esc_speed_control_test.py  adaptive closed-loop and rug-transition experiment
static_odom_noise.py       stationary odometry measurement
analyze_odom_observer.py   offline raw/EMA/alpha-beta comparison
steering_sanity.py         lifted-wheel channel/polarity/center check
steering_system_id.py      rolling Ackermann curvature/hysteresis identifier
steering_staircase_id.py   continuous balanced steering staircase identifier
steering_limit_search.py   adaptive useful-curvature limit search
```

The retained CSV output is archived under `experiments/data`. New runs still
write into the current working directory; give reviewed runs descriptive names
before adding them to the archive so later tests do not erase their provenance.

Installed commands remain:

```bash
ros2 run ackermann_robot static_odom_noise --duration 30
ros2 run ackermann_robot esc_forward_calibrate --arm
ros2 run ackermann_robot esc_system_id --arm
ros2 run ackermann_robot esc_speed_control_test --arm
ros2 run ackermann_robot analyze_odom_observer \
  src/ackermann_robot/experiments/data/esc_closed_loop.csv
ros2 run ackermann_robot steering_sanity --arm --wheels-lifted
ros2 run ackermann_robot steering_system_id --arm
ros2 run ackermann_robot steering_staircase_id --arm
ros2 run ackermann_robot steering_staircase_id --arm --validation-profile
ros2 run ackermann_robot steering_limit_search --arm
```

Do not run more than one PCA9685-writing experiment or driver at a time.
