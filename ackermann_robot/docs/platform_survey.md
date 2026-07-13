# Platform survey — what similar projects do, adapted to our sensors

*2026-07-02. Distilled from MIT RACECAR, MuSHR, F1TENTH, AutoRally,
DonkeyCar/DIY Robocars, Duckietown (Ackermann DBv2), and the Nav2 MPPI
literature, filtered through our constraints: Pi 5 (~1 spare core), no
encoders, no VESC, PCA9685 → hobby brushed ESC + servo (no feedback),
rf2o lidar ICP 10 Hz + D435i IMU in a 30 Hz EKF.*

## The big picture

Every serious platform linearizes actuation as `command = gain·x + offset`
and calibrates it against a **measured trajectory** — none rely on actuator
feedback for the mapping. The academic platforms (MIT/MuSHR/F1TENTH) then
dodge our whole throttle problem with the VESC's ERPM feedback; **nobody in
this space has a documented solution for brushed-ESC deadband/breakaway
drift** — DonkeyCar doesn't even model deadband, and RLDonkeycar (the one
attempt) gave up and recommended encoders. Our event-based breakaway
detection + planned dither ID has no prior art here; the closest analog is
RLDonkeycar using optical flow as a binary moving/not-moving detector to find
`minThrottle` — same idea, worse sensor than our ICP.

## Directly actionable transfers (ranked)

1. **Check PCA9685 oscillator drift** (DonkeyCar issue #940). The board's
   25 MHz oscillator varies per-board AND with temperature; a documented
   Waveshare clone output 1300 µs when commanded "1500 µs neutral" and ran
   5 Hz off nominal. Some of our "ESC deadband drift" may be the *board*
   moving our pulse widths, not the ESC. One-time fix: measure actual pulse
   width (scope, or Pi GPIO input capture wired to a PCA9685 output),
   set the true `reference_clock_speed`. Cheap, do before trusting any
   temperature-drift conclusions in auto_calib.

2. **Automate MuSHR's steering calibration with the EKF as the ruler**
   (mushr.io/tutorials/tuning/, and Duckietown's camera-based auto-cal paper
   proves the exteroceptive version works):
   - *offset*: constant slow forward, trim servo center until mean EKF yaw
     rate ≈ 0 (our yaw PI already does this transiently — persist it as a
     learned parameter instead of re-finding it every drive).
   - *gain*: command max steer at constant speed, measure R = v/ω from the
     EKF, compare to L/tan(δ_max), L = 0.2775.
   - **Per-direction**: Duckietown DBv2 measured a 24% left/right gain
     asymmetry on a real RC linkage (they fit per-direction 4th-order
     polynomials). Expect ours to be asymmetric too — fit left and right
     separately. Natural auto_calib phase-2 companion to the throttle dither.

3. **Command slew limiters in cmd_vel_to_effort** (first-class params on
   every VESC platform: `max_servo_speed` 3.2 rad/s, `max_acceleration`
   2.5 m/s², 75 Hz smoothers). We currently pass RPP's raw steps straight
   to the actuators. A throttle accel limit also softens the kick-release
   surge; a steering slew limit matches what the servo-saver physically does
   anyway (1–2 s wind-in) so the commanded state tracks reality better.

4. **Command-driven kinematic prediction as odometry fallback** (mushr_pf
   motion model: propagate with commanded (v, δ) + honest noise,
   σ_v = 0.4 m/s, σ_δ = 0.2 rad). This is the documented cure for our
   ICP-freeze/blind-reverse class of failure: when rf2o dies, predict from
   commands with wide covariance instead of freezing. **Critical caveat for
   us:** during a stall the commanded model lies (cmd 0.3, real 0) — gate it
   on the phase-1 vibration bit (chassis-is-moving) before letting it into
   the EKF.

5. **Structured excitation beats iterative learning** (AutoRally: a
   deliberate ~30 min protocol — steady speeds, step throttle + brake,
   zig-zags, both directions — trained their NN dynamics; DAgger-style
   retraining added nothing measurable). For phase 2/3: one good manual
   teleop session with a designed excitation script is worth more than weeks
   of passive nav-goal data. Also validates the dither plan: excitation
   design is the whole game under closed loop.

6. **Persist calibration as a versioned artifact** (Duckietown: per-robot
   YAML with `calibration_time`, save-service). Phase 3 should write learned
   params to a dated yaml the launch reads, not patch config in place.

7. **State-estimates-only system ID is proven** (arXiv 2411.17508: full
   dynamic single-track + tire model identified from [vx, vy, ω, δ] at
   50 Hz, 30 s of driving, no encoders — adapts to new tires in ~1 s). Our
   EKF outputs exactly those states (at lower rate/quality). This is the
   phase-3 endgame template.

## Not transferable (VESC/encoder-dependent — don't chase)

ERPM-based speed control and `vesc_to_odom` (the exact component that lets
those platforms ignore our throttle problem), analytic
pole-count/wheel-circumference speed gains, tachometer odometry.

## MPPI verdict — revised after evidence (2026-07-02)

Maintainer survey (arXiv 2307.15236): MPPI "rarely requires active recovery
behaviors... predictive back-out maneuvers... whereas [RPP] will simply stop
and wait" — the user's instinct that MPPI gets trapped less is **confirmed
by the literature**. RPP's documented Ackermann failure (issue #4757: the
lookahead carrot skips right past Reeds-Shepp cusps, so planned reversal
maneuvers never execute) is plausibly behind some of our stuck episodes.

Against it, for us, today:
- ~30–40× RPP's CPU (Ryzen benchmark: MPPI ~125 Hz max vs RPP >4000 Hz);
  Jazzy ships the slow xtensor build (the 40–45% Eigen speedup is
  Kilted-only, as is missed-loop delay compensation). Best ARM datapoint:
  Jetson Orin ≈ half a core at defaults; **nobody has published a working
  Pi 4/5 config**. Our 20 Hz loop already misses rate, and MPPI (stateful
  across cycles) degrades worse than RPP when the loop is late.
- MPPI's known weak spots are precisely our weak spots: noisy small
  steering/throttle commands (our deadband's worst regime), reverse tracking
  (issues #4425/#5714/#5806), and it seeds rollouts from odom twist — ours
  is 10 Hz lidar ICP with phantom spikes.

**Decision: not "never" — "after the actuator is honest, as a bench
experiment."** Package is already installed (1.3.11). Experiment recipe
(from Orin/community low-compute practice, e.g. batch 300 forks):

```yaml
# Add as SECOND controller plugin (ControllerSelector already in the BT;
# select per-goal via controller_selector topic; FollowPath/RPP stays default)
controller_plugins: ["FollowPath", "MPPI"]
MPPI:
  plugin: "nav2_mppi_controller::MPPIController"
  motion_model: "Ackermann"
  AckermannConstraints: { min_turning_r: 1.3 }   # MUST match Smac
  batch_size: 300          # community low-compute value; raise if CPU allows
  time_steps: 24
  model_dt: 0.1            # matches 10 Hz controller AND 10 Hz rf2o odom
  vx_max: 0.25
  vx_min: -0.20
  wz_max: 0.20             # vx_max * kappa_max(0.8)
  vx_std: 0.2
  wz_std: 0.1              # keep LOW: steering noise is poison for our deadband
  enforce_path_inversion: true
  PreferForwardCritic: { enabled: true, cost_weight: 3.0 }  # forward bias, but don't fight planned reverse
critic list: ConstraintCritic, CostCritic, GoalCritic, GoalAngleCritic,
             PathAlignCritic, PathFollowCritic, PathAngleCritic, PreferForwardCritic
# controller_frequency: 10.0  (global — affects RPP too; part of why this is
# an experiment, not a default)
```
Gate to run it: deadband validated on a healthy pack + CPU diet applied
(drive_logger off ≈ 19%, floor_scan off where safe ≈ 30%). Measure: loop-rate
misses, one-core %, and stuck-episodes per session vs RPP on the same course.
Abort signal: docking oscillation near goals (issue #5375) or steering
chatter — retune critics before adding compute.

## Sources

- MuSHR tuning: https://mushr.io/tutorials/tuning/ · mushr_pf: https://github.com/prl-mushr/mushr_pf
- MIT RACECAR configs: https://github.com/mit-racecar/vesc · particle filter: https://github.com/mit-racecar/particle_filter
- F1TENTH calibration: https://f1tenth.readthedocs.io/en/foxy_test/getting_started/driving/drive_calib_odom.html
- On-track sys-ID in under a minute: https://arxiv.org/abs/2411.17508 · ForzaETH stack: https://arxiv.org/abs/2403.11784
- AutoRally platform: https://arxiv.org/pdf/1806.00678 · IT-MPC/MPPI: https://homes.cs.washington.edu/~bboots/files/InformationTheoreticMPC.pdf · online adaptation: https://arxiv.org/pdf/1905.05162
- DonkeyCar calibrate: https://docs.donkeycar.com/guide/calibrate/ · oscillator drift: https://github.com/autorope/donkeycar/issues/940 · RLDonkeycar: https://github.com/downingbots/RLDonkeycar
- Duckietown wheel cal: https://docs.duckietown.com/daffy/opmanual-duckiebot/operations/calibration_wheels/index.html · Ackermann DBv2: https://github.com/duckietown/dt-car-interface-dbv2 · camera auto-cal paper: Appl. Sci. 11(13):5806
- Nav2 MPPI docs: https://docs.nav2.org/configuration/packages/configuring-mppic.html · controller survey/benchmarks: https://arxiv.org/pdf/2307.15236 · RPP-skips-cusps: https://github.com/ros-navigation/navigation2/issues/4757 · MPPI reverse issues: #4425, #5714, #5806 · Orin CPU report: https://discourse.openrobotics.org/t/nav2-speedups-in-mppi-smac-planner/41667
