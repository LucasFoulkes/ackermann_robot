> **HISTORICAL** (frozen 2026-07-15) — superseded by `stack-review-2026-07-15.md` and `how-the-learner-works.md`; kept for provenance. Verdicts here may be stale.

# Auto-Learn Project — Status & Session Log

*Last updated: 2026-07-02 (evening session). Living document for the
"robot gets more accurate as it drives" effort.*

## Goal

The robot should continuously calibrate itself from normal driving: the
effort→motion map (deadbands, gains), eventually steering, using its own
sensors as ground truth — instead of the hand-tuned constants in
`config/*.yaml` that go stale every time hardware, battery, or terrain
changes. No wheel encoders exist; lidar ICP + SLAM + D435i IMU are the only
feedback.

---

## GOAL & PLAN — restated 2026-07-04 after the F/R migration marathon

**The goal, in one sentence:** every number that maps commands to motion —
forward throttle, reverse throttle, **and steering** — is measured by the
robot from its own sensors during normal Nav2 driving, persisted across
sessions, and applied automatically; hand-tuned constants and their rot are
retired; progress is one number (drive_score) that goes up.

**What is actually learned today (the honest inventory):**
| Parameter | Status |
|---|---|
| Forward breakaway | LEARNED (measured, persisted, seeded, guarded) |
| Reverse breakaway | LEARNED (2026-07-04; seeded from manual sweep) |
| Forward speed map (deadband/slope) | hand-set (from sweeps/log fits) |
| Reverse speed map | hand-set |
| **Steering map (center, gain, per-direction)** | **hand-set — nothing learns it** |
| Ceilings, kick dynamics, PI gains | hand-set scaffolding |

**The plan, in order:**
- **M1 (done):** breakaway learning both directions, wired to starts.
- **M2 (next session):** steering map learned passively — auto_calib already
  collects steady windows; extend to (steer_effort, kappa=w/v) pairs, fit
  offset + per-direction gain (research: MuSHR EKF-as-ruler procedure;
  Duckietown measured 24% L/R asymmetry on RC linkages), feed the steering
  feedforward under the same guards as breakaway (plausibility band,
  explorer, recency). Same pattern as M1 — no new architecture.
- **M3:** the dither (active excitation) → trustworthy speed gain G per
  direction → deadbands/max_speed live-derived. This is the milestone that
  retires most constants.
- **M4:** delete scaffolding. The seven control mechanisms (kick, seed,
  cooldown, interlock hold, watchdog, IMU release, PI stall-hold) exist to
  compensate for unknown maps. As M2/M3 land, each is either justified by a
  written reason or deleted. Target: <= 4 mechanisms.
- **Success metric:** drive_score (progress-weighted) > 80 sustained across
  3 sessions without config edits between them. Then CPU diet, then the
  farm items (GPS, rtabmap multi-sensor).

**Coherence rule learned this week, now binding:** a parameter is either
LEARNED (with guards + exploration) or MEASURED-AND-DATED (with the sweep
that set it); never "tuned until the complaint stopped." Every regression
tonight traced to a constant set under one hardware mode surviving into
another.

## Where we are: PHASE 1 DEPLOYED (observe-only), 2026-07-02

### What runs now

- **`auto_calib` node** (`ackermann_robot/auto_calib.py`, in `drive.launch.py`
  by default, `auto_calib:=false` to disable, ~2% core). Observe-only:
  - Learns **forward breakaway deadband** from motion-onset events during
    stiction-kick ramps. Reports median + IQR every 15 s.
  - Collects **steady (effort, speed) pairs** per direction, fits a line, but
    publishes a **trust flag** — see "why naive learning fails" below.
  - Publishes `/auto_calib/params`; logs via
    `journalctl -u ackermann-drive | grep auto_calib`.
- **EKF at 30 Hz** (was 20; measures ~27). The EKF predicts between the 10 Hz
  sensors — fresher velocity for the speed PI / watchdog / interlock.
- **Vibration metric** in `cmd_vel_to_effort` (accel-envelope from the 30 Hz
  IMU feed, last field of `/cmd_vel_to_effort/debug`). Observe-only: gathering
  data to pick thresholds for a chassis-is-moving bit.

### First real finding (validated offline on 13 of tonight's logs)

**Forward breakaway ≈ 0.65 raw** (51 events, median 0.65, IQR 0.64–0.68;
odometry lag inflates ~0.05) vs `throttle_deadband_fwd: 0.45` in the yaml.
The feedforward for 0.3 m/s lands at ~0.61 — *at* breakaway — which is why
forward starts stall until the stiction kick rescues them.

**CONFIRMED LIVE and APPLIED 2026-07-02 late session:** auto_calib live
reports gave median 0.69, IQR 0.67–0.70 (n=17). The 22:13 drive log showed
the cost of the stale value: 51% of forward-commanded ticks stalled
(cmd ≥ 0.25, |odom_v| < 0.05, median eff_drive 0.659), 109 odom-watchdog
trips across the evening ("stops for no reason"), motion arriving as
stall → stiction-kick → surge bursts (odom_v 0 → ~1.0 m/s).
`throttle_deadband_fwd` 0.45 → **0.60** (breakaway minus odom-lag
inflation). Untested on hardware yet — first drive after this change should
watch for persistent cruise overspeed; if so, lower toward 0.55 before
touching the PI.

### Why naive "learn while driving" fails (measured, not theoretical)

Both failure modes were hit on tonight's real logs before the node design:

1. **Dynamic regression** (fit dv/dt = k1·(u−u0) − k2·v): biased by the
   closed loop — the PI deliberately anti-correlates effort with speed.
   Fits gave u0≈0.84, negative drag. Textbook identification-under-feedback.
2. **Steady-state regression** (fit v = G·(u−u0) on cruise points): biased by
   **terrain selection** — the PI settles at high effort exactly where ground
   is hard, so steady pairs trace the terrain distribution, not the motor
   curve. Fitted G came out *negative* on 91 real pairs.

What survives closed loop: **event-based estimators** (breakaway = motion
onset, an event not an equilibrium) and **known-input correlation** (phase 2).

---

## Roadmap

- **Phase 1 (DONE, observing):** breakaway deadband + steady pairs + trust
  flags + vibration metric. Validate over a few normal drives.
- **Phase 2 (next):** active excitation — a small known feedforward dither
  (±0.02–0.03 effort, sub-Hz) injected by `cmd_vel_to_effort`, and auto_calib
  correlates response against it (lock-in style). This identifies the true
  speed gain G per direction despite the closed loop. Then: vibration-bit
  thresholds → wire into watchdog (fast blind-runaway trip) and kick veto.
- **Phase 2 additions from the platform survey** (see `platform_survey.md`
  for sources/details, 2026-07-02):
  - **PCA9685 oscillator check first**: board clock drifts per-board and
    with temperature (documented 200 µs neutral error on a clone) — measure
    actual pulse widths once before attributing drift to the ESC.
  - **Steering auto-calibration, per-direction** (MuSHR procedure with EKF
    as ruler; Duckietown measured 24% L/R asymmetry on RC linkage):
    offset ← mean yaw rate ≈ 0 driving straight; gain ← R = v/ω at max
    steer vs L/tan(δ); fit left and right separately.
  - **Slew limiters in cmd_vel_to_effort** (every VESC platform ships them:
    ~3.2 rad/s steer, ~2.5 m/s² accel) — also softens kick-release surge.
  - **Command-model odometry fallback** (mushr_pf pattern): propagate from
    commanded (v, δ) with honest noise when rf2o freezes — but gated on the
    vibration bit, because during a stall the command model lies.
  - **One structured excitation teleop session** (AutoRally: 30 min of
    designed excitation beat all iterative retraining): steady speeds, step
    throttle, zig-zags, both directions.
  - **Persist learned params as a dated calibration yaml** (Duckietown
    pattern), loaded at launch — the phase-3 delivery mechanism.
- **Phase 3:** close the loop — feedforward params (deadband, max_speed,
  max_speed_rev) derived live from the learned model, bounded, with the
  watchdog underneath. Retire hand tuning.
- **Phase 4 (optional):** persist SLAM maps across sessions (localization
  mode — accuracy accumulates across days); Nav2 MPPI controller with the
  learned model *only if* RPP + good feedforward still isn't enough
  (MPPI is the most expensive Nav2 controller; Pi budget is ~1 spare core).
  **MPPI stance revised 2026-07-02** after a literature pass (details +
  ready-to-run experiment config in `platform_survey.md`): the
  gets-stuck-less claim is real (maintainer survey: MPPI rarely needs
  recovery behaviors; RPP documented to skip Reeds-Shepp cusps, #4757), so
  it's promoted from "only if" to a **planned bench experiment** — as a
  second selectable controller (BT ControllerSelector), 10 Hz / batch 300 /
  model_dt 0.1, AFTER deadband validation on a healthy pack + CPU diet.
  Known risks: no published Pi 5 config exists, Jazzy has the slow xtensor
  build, and MPPI's weak spots (steering noise near zero, reverse tracking,
  odom-twist-seeded rollouts) are exactly our weak spots.

### Explicitly rejected

- **End-to-end RL:** needs thousands of trials incl. crashes; Pi 5 has no
  compute headroom (DR-SPAAM benchmarked 0.9 Hz here). Model-ID + adaptive
  feedforward gets the benefit without the risk.
- **Visual odometry for more feedback rate:** benchmarked on this Pi
  (2026-06-09): rgbd_odometry caps at 10–12 Hz with quality collapse at 6×
  the CPU of lidar ICP. Camera fps is not the bottleneck; compute is.
  The C1 lidar is hardware-fixed at 10 Hz.

### CPU diet (once auto_calib is trusted)

`drive_logger` (~19% core) → default off; `use_floor_scan:=false` (~30% core)
where low obstacles don't matter. Camera cannot go entirely: the EKF's gyro
fusion and the vibration metric live in it (IMU-only camera ≈ 10%).
icp_odometry also dies without the camera — unresolved bug, see
drive.launch.py comment (2026-06-14).

---

## Session log 2026-07-02 — the ESC reverse saga (context for why auto-learn)

Drivetrain was converted from H-bridge to a Hobbywing QuicRun-1060-style
brushed ESC (ch 14, committed `814fd1d` this session, code from 2026-06-15).
A day of field tests exposed, layer by layer:

1. **Blind reverse runaway** (`9c5c908`): lidar ICP couldn't track fast
   reverse (odom read ~0 / phantom-positive while backing fast) → speed PI +
   stiction kick stacked effort to raw −0.8. Fixes: odometry-distrust
   watchdog (2 s commanded-but-unmeasured motion → neutral) + per-direction
   raw ceilings.
2. **ESC reverse cliff** (`84d8246`): SLAM-ground-truth showed raw −0.61…−0.64
   ≈ 0.25 m/s but −0.69 ≈ 0.7–0.9 m/s. Ceiling calibrated to the good side.
3. **Non-deterministic engagement** (`95009a0`): same −0.6 effort → 0.3 m/s or
   NOTHING, coin-flip. Cause: ESC brake/reverse state machine. Fix: driver
   double-tap (brake tap → neutral gap → reverse). Also fixed watchdog
   latching across stale-cmd gaps (bricked all motion).
4. **True reverse threshold** (`7ab33ce`): raw sweep below the old floor —
   nothing at −0.40, moves at −0.50, and **constant effort = constant
   acceleration** (reverse is torque command: −0.50 held 3 s went 0.2→0.9
   m/s). deadband_rev 0.55 (H-bridge fossil) → 0.45, max_speed_rev → 2.0.
5. **PI wound the wrong way** (`0461d96`): during the double-tap's 300 ms the
   integral wound deeper-into-reverse. Reverse trim is now brake-only, kick
   forward-only, ceiling −0.55.
6. **Watchdog blocked stall escape** (`89e062c`): a forward-stall trip carried
   across Nav2's sign flip to reverse. Sign flip now re-arms. Plus reverse
   ff −0.51 (max_speed_rev 2.6), PI brake authority 0.2.
7. **Self-calibration phase 1** (`07d8095`): this document's subject.

**Hardware TODO (DONE 2026-07-03, with one incident):** ESC mode jumper is
now on **F/R** (single-click fwd/rev) and `esc_double_tap: false`. INCIDENT:
it spent part of 2026-07-03 on **F/B by mistake — B means BRAKE, not
backward** — F/B has NO reverse, and through our inverted motor wiring that
presented as "robot only reverses, forward completely dead even at raw 1.0."
Bench test (driver alone, raw efforts) diagnosed it in minutes; no hardware
damage. Rules learned: (1) the three modes are F/B/R, F/B, F/R — we want
F/R; (2) power the ESC ON only while the driver is streaming neutral (it
samples the throttle line at boot); (3) the jumper can walk with vibration —
seat it firmly/tape it; (4) **F/R maps throttle polarity OPPOSITE to F/B/R**
— after the jumper fix the robot drove backward on forward commands (53%
wrong-way ticks, and "backward" got the hot forward calibration = "way too
fast"); `esc_inverted` flipped true→false, bench-verified both directions.
ESC startup beeps: 2 short = 2S LiPo detected, 1 long = self-test OK
(normal, not a fault).

**Also known:** ESC reverse hardware current is half of forward (30 A vs 60 A
cont.); throttle range self-calibrates at power-on (power ESC with driver
already at neutral).

## Addendum 2026-07-02 ~22:40 — deadband retest inconclusive: plant is fading

The 22:30 drive (first with deadband_fwd 0.60) got WORSE, and the cause is
physical, not the config: forward speed-per-effort collapsed across the
evening (v/eff while moving: 0.58 @ 20:17 → 0.62 @ 22:13 → **0.12 @ 22:30**;
eff 0.75 produced 0.09 m/s) and auto_calib breakaway kept CLIMBING
(0.65 offline → 0.69 @ 22:13 → **0.76 @ 22:30**).

**Sound corrected by user: high-frequency BUZZ/whine, not beep patterns.**
That is the signature of a motor held ENERGIZED BELOW BREAKAWAY — stall
ticks put 0.6–0.75 effort into a stationary rotor, and the windings sing at
the ESC's internal chopping frequency (kHz, distinct from our 50 Hz PCA9685
servo pulse, which is fine and matches DonkeyCar practice on the same board).
The robot stalled 51–92% of forward-commanded time tonight, i.e. it spent
most of the evening buzzing.

**Working theory — stall-heat spiral, not (only) battery:** a stalled brushed
motor has no back-EMF, so stall effort = near-max current = I²R heat. Hot
copper gains ~+40% resistance and hot magnets weaken → less torque per
effort → breakaway climbs → more stalling → more heat. The buzz and the
power fade are the same phenomenon. (Pi supply clean, throttled=0x0; pack
voltage unmeasurable since the power-HAT death — a sagging pack would stack
on top of this, same direction.) The deadband_fwd 0.60 fix attacks the
spiral at its root by cutting stall dwell; the odom watchdog's hold-neutral
is also what limits buzz episodes to 2 s.

**Lesson for the project thesis:** breakaway is not a constant — it drifted
+0.11 within one evening with temperature/battery state. Exactly why phase 3
wants the feedforward fed live from auto_calib instead of any yaml constant.

**Next session, in order:** (1) charge/swap the pack, check ESC/motor temp by
hand; (2) redo the deadband_fwd 0.60 validation drive on a full pack — if it
now overspeeds, 0.60 is too hot for a healthy battery (auto_calib's live
breakaway report is the tiebreaker); (3) first field test of the new
`retrace_recovery` node (BT BackUp now retraces the recorded forward path —
validated in fake-odom simulation only).

## Addendum 2026-07-02 ~23:30 — third round: the two-threshold model

The 23:09 drive (deadband 0.60) diagnosed itself: overshoot to 0.75–0.95 m/s
at cmd 0.25 once rolling, yet still 49–75% stalled with breakaway drifted to
0.74–0.80 (hot motor). Conclusion: **breakaway and keep-rolling are two
different thresholds and one deadband constant cannot serve both.** Fix
applied: deadband_fwd 0.55 + max_speed 1.9 map the ROLLING regime (cmd 0.25
→ raw 0.61 ≈ the measured 0.30 m/s rolling point); the stiction kick
(rate 0.8, cap 0.45) is now the designated breakaway bridge. This is the
role separation the platform survey implied — VESC platforms get the rolling
map for free from ERPM feedback and never meet breakaway at all.

Also this round, all from the same session's logs:
- **Odom spike defense added** — EKF `odom0_twist_rejection_threshold` 1.0 →
  2.5σ. (CORRECTION 2026-07-03: the rf2o covariance patch made the same
  night was DEAD CODE — the active lidar odom is rtabmap icp_odometry on
  /odom_icp, not rf2o, which is no longer wired into the launch. The
  measured noise improvement — at-rest std 0.105 → 0.032 — therefore came
  from the EKF gate alone. rf2o patch left in place in case it returns.)
- **depth_floor_scan ring suppression ate real obstacles**: wall arc at
  0.61 m / 0.16 m spread matched the "noise ring" signature (≤0.18) every
  cycle → camera contributed nothing all session. Tightened to spread ≤0.10
  AND ≥60% coverage.
- **BIDIRECTIONAL goal headings reverted** — no-valid-path went 3/evening →
  10/session, presenting as "stuck and won't recover."
- **Retrace recovery worked on hardware**: 8 invocations, completed 0.50 m
  retraces. Fixed a cosmetic shutdown traceback (publish after context
  teardown on SIGINT).
- CPU is currently NOT the bottleneck: median 46–48%, p90 56–63%, only
  brief 100% spikes and 6 missed control loops/session.

## Addendum 2026-07-02 ~23:45 — fourth round: the model holds

23:24 drive, first with the two-threshold split + honest rf2o covariance:
- **Zero watchdog trips** (109/evening at peak). Cruise median 0.29 m/s at
  cmd 0.25, eff 0.60 — the rolling map is calibrated. Stall ticks (45%) are
  now brief kick-ramp transients at each of the many starts, not 2 s hangs.
- **Odom fix confirmed**: at-rest odom_v std 0.105 → 0.032, worst phantom
  0.78 → 0.16 m/s. Collision-ahead halts 466 → 166/session.
- **auto_calib is consistent**: breakaway median 0.66 (IQR 0.65–0.68, cool
  motor) vs 0.65 offline — the event estimator repeats across days/sessions.
  Steady-gain fits still trust=LOW (correctly; awaits phase-2 dither).
- **Camera now marks for real — and grabbed floor texture**: the gravity
  floor fit reads 2–3 cm low, so min_obstacle_height 0.04 was effectively
  ~0.015 above true floor. Raised to 0.065 (true ~0.04; the 6 cm wheels
  climb 2 cm anyway).
- **Weave/oversteer measured**: steer_sat 34% of moving time. Structural
  insight: the yaw PI compares yaw RATE, so kick-transient overspeed makes a
  correct curvature read as "too much w" → PI unwinds good steering → RPP
  re-corrects → weave. Halved yaw gains (0.12/0.1) as damping; the real fix
  — yaw loop on curvature (w/v) instead of w — queued for phase 2.

## Addendum 2026-07-03 ~00:00 — fifth round: carpet, and the camera joins the local costmap

23:35 drive: 9 stuck episodes (up to 10 s) ALL with eff_drive pinned at
0.88 = the kick+ceiling maximum. **Carpet breakaway > 0.90 raw** — the
ceiling was hard-floor-calibrated, and the watchdog cycled hold/retry every
2 s against it ("stops when it can clearly move"). Fix: raw_throttle_max_fwd
0.90 → 1.0 and kick_max 0.45 → 0.85, so the kick can escalate to full
throttle in ~1.4 s. Full throttle is only reachable while stalled (kick
bleeds on motion; PI brakes; watchdog bounds blind runaway).
**Surface-dependent breakaway is the clearest phase-3 use case yet** — no
constant serves both tile and carpet; the live breakaway estimate should
set the kick envelope.

Also: camera_layer restored to the LOCAL costmap (removed in the
dotted-ghost era; the artifact is now filtered at the source). Until now
low obstacles never reached RPP's collision check — the camera only
influenced 0.33 Hz global replans. Collision-ahead halts kept falling:
466 → 166 → 40/session as odom cleaned up. Breakaway median 0.65 again
(third consistent session).

## Addendum 2026-07-03 ~00:30 — F/R mode first drive: the reverse band is 4 ticks wide

00:15 drive (jumper in F/R, double-tap off): direction flips engage in 0.7 s
median (was 1.3 s+ with the tap dance) — but p90 was 8 s. Cause found in the
efforts: reverse stalls at raw −0.467 and does 0.47 m/s at −0.49. **The
usable reverse band is ~0.02 logical ≈ 4 PWM ticks at 50 Hz** (esc range
capped to ±51 ticks; 1 tick ≈ 4.9 µs). Reverse is bang-bang by quantization:
RPP approach creep mapped below engagement (the 8–10 s cusp hangs = "bad at
3-point turns"), one tick more = rocket. Interim: deadband_rev 0.46 +
ceiling 0.51 centers commands in the band; Nav2 min_approach 0.15 stops
commanding speeds the hardware can't make; RPP min_lookahead 0.35 so the
carrot can't skip Reeds-Shepp cusps (nav2 #4757).

**Queued experiments:** (1) proper F/R reverse raw sweep (the F/B/R "cliff"
data is obsolete); (2) raise PCA9685 pwm_hz 50 → 100+ — doubles tick
resolution inside the band (ESC frame-rate tolerance unverified, bench-test
with wheels off the ground); (3) servo idle-relax + forward-escape retrace
shipped this round, verify on next drive. No command-transport timing issues
found (0 dropouts, 0 missed control loops — the "timing" feel was the
flip-latency tail).

## Addendum 2026-07-03 ~00:45 — PHASE 3 LINK #1 SHIPPED: learned breakaway drives the kick

`cmd_vel_to_effort` now subscribes to `/auto_calib/params`: once auto_calib
has ≥10 breakaway events, forward kicks SEED at the live median (minus
0.05 odom-lag inflation) instead of ramping from zero. First closed
learn→act loop: the start behavior now adapts to surface and motor
temperature within one session. Also enabled the kick in REVERSE
(`kick_reverse: true`) — the forward-only gate guarded against the
double-tap misfire, which died with the F/R jumper. Reverse kick authority
is bounded by raw_throttle_max_rev (0.58: breakaway headroom over the
0.46/0.49 rolling band; the 00:22 run showed 82% reverse stall because the
kick couldn't fire in reverse AND the 0.51 cap gave no headroom).

Watch on next drive: journal line "kick seeding live: breakaway=..." =
the loop is closed; buzz per start should drop to near nothing once seeded.

## Addendum 2026-07-03 ~01:00 — the seeding ratchet (and its cure)

The 00:50 run validated the lunge chain (lunges 10 → 3, collision-ahead
479 → 96 — the lunge-pitch-phantom hypothesis was right) and exposed the
first genuine ADAPTIVE-CONTROL failure of the project: published breakaway
ratcheted 0.65 → 0.82 → 0.98 across seeded sessions. **A seeded start jumps
effort over the true threshold, so effort-at-onset measures the seed, not
the plant — the estimator was eating its own output.** Same disease as the
phase-1 regression bias (identification under feedback), new costume.

Cure, both sides:
- auto_calib rejects jump-onset events (effort rose > jump_gate 0.12 in the
  0.4 s before onset) — only swept onsets are measurements.
- cmd_vel_to_effort runs every 5th start UNSEEDED (kick_seed_explore_every)
  — periodic honest sweeps let the estimate FALL when the surface eases.
  One slightly buzzy start in five is the exploration budget.
- Poisoned persisted state deleted; ring re-learns clean from next session.

**Standing lesson for every future learned parameter (gain, steering,
reverse):** the moment a learned value drives actuation, the estimator must
either reject actuation-shaped samples or be fed scheduled exploration.
Phase-2 dither IS that exploration, formalized.

## Addendum 2026-07-05 — the reverse-lunge marathon and the pulse-domain fix

A long night chasing "reverse moves way faster than forward and lunges."
Everything tried FIRST, and why each failed — all of it worth keeping
because the failures share one root:

- Opened the reverse map (deadband_rev, max_speed_rev), added a REVERSE
  effort envelope, per-direction PI cruise memory, interlock windowing
  (reverse ICP reports phantom +0.35 forward while backing), overspeed
  hysteresis, then open-loop pulse-and-coast. Each helped a little; none
  fixed it. **Common flaw: every one operates on LOGICAL EFFORT** — the
  input to `logical_to_raw()` — while the actual asymmetry is DOWNSTREAM in
  the pulse/tick map.
- Detour: theorized reverse was a "torque command" (constant effort →
  constant accel). Half-right — the runaway is real — but the framing sent
  us tuning effort-domain limiters that always reacted too late (reverse
  ICP lags ~0.5 s).
- **Checked the ESC manual (Hobbywing 1060):** reverse is proportional, not
  special — BUT runs a weaker output stage (30 A vs 60 A, 0.002 vs 0.001 Ω).
  That means a *different µs→speed curve each side of neutral*. Measured:
  0.50 vs 0.61 m/s per raw unit; eff −0.48 → ~1 m/s² vs +0.41 → ~0.4 m/s².
- **The fix: `esc_rev_scale` (ackermann_driver), a single gain on the pulse
  DEVIATION where esc_throttle < 0**, after all effort logic, before
  esc_inverted. 0.5 was slightly weak (near the reverse dead zone), 0.75 hit
  it. Robot-frame, reverse-only; forward is not touched (the multiply is
  gated on esc_throttle < 0.0). "Kinda had the effect we wanted" → landed.

**The standing lesson (now the first thing to try for any regime asymmetry):**
if one direction/speed-regime is wrong by a roughly constant ratio, the bug
is almost certainly in the PULSE DOMAIN (tick map / output stage), and the
fix is one gain on the output — NOT re-tuning the effort maps three layers
up. We spent hours in the effort domain because that's where all the knobs
are; the knob that mattered didn't exist until we added it at the output.
Note the user's own instinct ("send 50% of the signal") pointed straight at
the pulse domain — trust that framing next time.

Follow-ups: (a) proper fix is recalibrating `esc_min_tick` from a reverse
sweep so the tick map is honest across the whole range; `esc_rev_scale` is a
correct equivalent pinned near one operating point — fine unless slow/fast
reverse drift appears. (b) The reverse effort-domain scaffolding (overspeed
hysteresis, reverse envelope, rev_pulse) is now redundant backstop → M4
delete candidates after a few clean reverse drives.

## How to check what it's learning

```bash
journalctl -u ackermann-drive -f | grep auto_calib     # live reports
ros2 topic echo /auto_calib/params                     # machine-readable
```

Report format: `FWD breakaway: n=.. median=..` (want n>20 before trusting),
`FWD/REV steady: n=.. G=.. u0=.. trust=..` (trust LOW until phase-2 dither).
