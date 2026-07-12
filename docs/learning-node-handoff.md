# Learning-Node Handoff — state, evidence, and remaining path

Written 2026-07-11 (end of a full measurement-and-build day). Audience: an AI
(or human) continuing this project with fresh context. Everything below is
backed by logs in `~/.robot/drive_logs/`, bags in `~/.robot/bags/`, and
experiment CSVs in `~/ros2_ws/`. The assistant memory directory
(`~/.claude/projects/-home-luky-ros2-ws/memory/`) holds the distilled
version; this document is the complete one.

## 1. The goal

**Any Ackermann robot should learn to drive itself from its own ordinary
driving — the way a person adapts to a rental car.** No recurring scripted
calibration, no special floor space, no extra sensors (explicit owner
decision: NO voltage sensor — the signal-response mismatch IS the
measurement; battery, rug, and temperature are one latent quantity).

The deliverable ("the little dude") is one ROS 2 node plus two files:

- **Birth certificate** (human-written once per robot): actuator channels,
  tape-measured wheelbase/footprint, wheels-lifted servo pulse limits,
  lidar mount pose. Only things that cannot be learned safely by driving.
- **Memory file** (`~/.robot/adaptive_ackermann_runtime.yaml`, schema v7):
  everything learned — maps, breakaway, trims, floor observer, effort scale.
  Values persist across boots; *confidence* deliberately does not for
  quantities that drift while powered off (battery).

**Certification principle (the project's backbone):** every folded/online
estimator ships in shadow mode and is trusted only after it reproduces the
scripted ground-truth measurements on THIS robot (section 3). Only then may
it be promoted to control, and only then is it trusted on a new robot.

Definition of done: (exam 1) wipe the memory file on this robot and it
relearns its own certified numbers during a supervised ordinary drive;
(exam 2) a second, unmeasured chassis bootstraps from a birth certificate
and reaches the same drive quality with no scripted run.

## 2. System

- Pi 5 (NVMe HAT). PCA9685 at I2C 0x40, **prescale 30 → ~197 Hz, 1.24 µs/tick**
  (do not assume 50 Hz / 4.9 µs — an old retracted claim). ESC ch14
  (<1500 µs = forward), steering ch12. Power-monitor HAT (0x36) DEAD since
  2026-06-15 — no voltage telemetry, by design we don't want it anyway.
- RPLIDAR C1 (~10 Hz) → MOLA lidar odometry → `odom→base_link` TF. No
  encoders. Wheelbase 0.2775 m, footprint [−0.10..0.40]×[±0.16].
- Nav2 Jazzy: Smac Hybrid-A* (Reeds-Shepp, radius loaded from persistent
  four-branch trackability at launch; 1.30 m bootstrap), chronological cusp
  dispatcher, then RPP (`nav2_odom_experiment.yaml`). The BT computes once per
  new/updated goal (`navigate_to_pose_ackermann.xml`).
- Controller: `ackermann_robot/adaptive_ackermann_controller.py` (single
  node, ~1100 lines), params in `config/adaptive_controller.yaml`
  (**the deployed 24-point steering maps live there, NOT the sparse
  3-point defaults in the .py — experiment scripts must read the yaml**).
- Safety: independent watchdog `tools/esc_watchdog.py` + user systemd unit
  `esc-watchdog.service` — enforces *no live owner process ⇒ no drive pulse*
  (PCA9685 free-runs its last pulse; a SIGKILLed controller would otherwise
  drive away). Enabled and active.
- Build: `colcon build --packages-select ackermann_robot` from `~/ros2_ws`
  (copied install, NOT symlink — rebuild after every edit).
- Drive: `ros2 launch ackermann_robot bringup.launch.py arm_hardware:=true`.
  Experiments: same launch with `arm_hardware:=false record_bag:=false`,
  then the experiment binary with `--arm` (experiments own the PCA9685).

## 3. Ground-truth experiments (all 2026-07-11, hardwood)

### 3.1 Two-speed steering step (`steering_step_id`, csv `steering_step_id.csv`)
S-shaped legs, instant steering steps 0→±0.8→0 κ at 0.30 and 0.15 m/s,
48 steps. **Per-direction fit t50 = τ + d₀/v:**

- forward: **τ = 235 ms** [184, 312], **d₀ ≈ 0** [−1.6, +1.0 cm] (pure time lag)
- reverse: **τ = 168 ms** [139, 298], **d₀ = 3.3 cm** [0.6, 3.5], R² = 0.89
  (steered axle trails in reverse — needs rolling distance)
- Measured *through* the odometry pipeline → directly valid for
  preview/pose prediction. t63 ≈ 0.267 s fwd / 0.347 s rev.
- CAUTION: an early pooled fit gave "d₀ = 3–4 cm overall" — pooling artifact,
  retracted. Also retracted: "reverse +κ 12% map error" (the experiment
  script had hardcoded stale 3-point maps; against the deployed 24-point map
  the error is ~4%, in-family).
- Real finding: **forward center bias +0.033 1/m** (repeatable) → deployed
  map center point relabeled to (0.033, 1451.54); commanding κ=0 now sends
  ≈1441.6 µs. Reverse center fine (−0.006).

### 3.2 Sustain-map mining + curvature drag (from step-run data, no extra floor time)
- Straight blocks give map points (n=9 each, σ<1.2 µs):
  fwd **[0.148→1419.05, 0.302→1413.19]** (−38 µs per m/s),
  rev **[0.138→1590.34, 0.309→1594.57]** (+25 µs per m/s). Deployed.
- **Curvature drag = ZERO** (<1 PWM tick at κ=0.8, paired within-leg
  comparison, both speeds/directions). Throttle map stays 1-D.
- **Negative result that matters:** the throttle map CANNOT be mined from
  ordinary closed-loop driving — the slope signal (~1.6 µs over the driven
  speed range) is below the battery/surface confounder floor (~4.3 µs);
  fitted slopes came out sign-flipped. Steering-lag/gain CAN be probed.

### 3.3 Kinetic stall floor (`esc_sustain_floor_id`, csv `esc_sustain_floor.csv`)
Quasi-static pulse decay from 0.20 m/s (1 tick / 0.8 s) until stall:
- **Rolling dies at ~0.08 m/s both directions** (fwd last sustained
  0.104 @ 1415.5 µs, stall at 1416.8; rev mirror).
- Curve is LINEAR down to ~0.10 with map-matching slopes → no extra map
  points needed; the missing fact was the floor itself.
- Closed-loop 0.10 crawl is marginal (recoveries); **reliable floor 0.12**
  → `minimum_sustain_speed_mps: 0.12` and Nav2 RPP floors raised
  (`min_approach_linear_velocity`, `regulated_linear_scaling_min_speed`
  0.07 → **0.13**; 0.07 was BELOW the physical floor: mid-turn/near-wall
  regulation was commanding unsustainable speeds → crawl-stalls, worst in
  reverse arcs).

### 3.4 Direction flip (`esc_direction_flip_id`, csv `esc_direction_flip.csv`)
Launch, cruise, flip variants (immediate opposite pulse / 0.2 s / 0.5 s
neutral dwell), 12 legs:
- **ZERO vehicle rock-back in all flips** — the ESC reverses gracefully.
- **Immediate opposite-side pulse = active brake: 0.39–0.45 m/s² vs
  0.30–0.33 coasting (~1.4×)**, clean. Full reversal onset ~0.75–0.9 s via
  immediate flip (faster than any dwell variant). → Future primitives:
  brake-assisted cusp cycles, launch-overshoot braking. Not yet used.
- Consequence: the 58 'direction mismatch' faults in the 21:34 reverse-stress
  drive were NOT vehicle physics. Log forensics showed **Nav2/RPP command
  sign churning (+-+-+ within 2 s) near path inversions** (known Jazzy RPP
  weakness; proper fix = Nav2 Lyrical `enforce_path_inversion`, see
  masterplan). Deployed mitigation: `direction_flip_debounce_s: 0.5` —
  launch/flip only after the commanded sign is stable 0.5 s ('direction
  settle' soft stop otherwise). **UNVERIFIED — check next drive.**

### 3.5 The costmap discovery (not an experiment — a bag audit)
Both Nav2 costmaps had been **completely empty since the beginning**: the
per-source `max_obstacle_height` defaults to 0.0 and silently discarded
every lidar point. The planner was blind; all "avoidance" was the
controller's own scan guard + collision monitor vetoing blind plans (the
root of historical obstacle-stop churn — 268 s/run before, 0 after).
Fixed in `nav2_odom_experiment.yaml` (min 0.0 / max 2.0 on both scan
sources), verified live (~1500 lethal cells). **Rule: before ever blaming
the planner, verify the costmap contains the obstacle.**

## 4. Controller mechanisms and their status

| mechanism | params | status |
|---|---|---|
| launch: ramp-to-learned-breakaway, raw-evidence drop-through | `startup_drop_through_mps 0.15` (slope-converted; a fixed 6 µs was 2× too strong in reverse) | VERIFIED (hot launches 21%→~0 on calm runs) |
| breakaway learner (per-launch EW, causal attribution via `startup_kick_pulse_us`) | priors 1403/1611, forgetting 0.90 | live, works; values ≡ 0.70 m/s steady if held — never hold them |
| session warm-up (re-feel pedal) | 5 launches/dir, 6 µs weaker, forgetting 0.70 | VERIFIED (fresh-charge first-third hot launches 22→2) |
| planned_stop (stall near cusp OR path end < 0.40 m = intended stop; coast, steering pre-positions; 2 s recovery fallback) | `planned_stop_cusp_m`, `planned_stop_wait_s` | VERIFIED (0 fallback deadlocks; goal-end stall class eliminated) |
| obstacle gate on planning speed (max of measured, commanded) | in `_desired` | VERIFIED (chatter loop dead) |
| direction-flip debounce 0.5 s | `direction_flip_debounce_s` | built, UNVERIFIED |
| measured steering lag, applied once in preview | `steering_lag_time_{fwd 0.235, rev 0.168}`, `steering_lag_distance_{fwd 0, rev 0.033}`; `_steering_lag()` = τ + d₀/max(v, floor) | live; online delay estimator runs telemetry-only (locked at 0.20 grid cell — right cell, can't resolve direction split) |
| steering RLS (4 quadrant models) | `apply_steering_models: false` | SHADOW — proven load-free (XTE unchanged when disabled); closed-loop-biased by construction (gain≈1 artifact) |
| floor observer (died = recovery entry, held = slow-roll streaks) | shadow; logged `floor_estimate_*`, persisted | **CERTIFIED: died_ema 0.0836 fwd / 0.0834 rev vs scripted 0.08, in 3 ordinary drives.** First folded estimator certified. Reverse died_ema can be inflated by sagged-battery events — watch. |
| effort scale (ONE latent gain, both directions pooled) | KF: meas sd 0.05, process 5e-4/s, boot sd 0.10; obs = (pulse−1500)/(map(measured v)−1500); gates: |v−target|<0.06 AND rolling ≥1.5 s (`rolling_since`) | built, shadow, **needs one calm-drive verification**. Two prior bugs documented below — read them before touching it. |
| probe scheduler (±8 µs steering taps, 0.5 s, ≥4 s apart; gated: rolling, straight-ish, clear, no stop <1 m, XTE<5 cm) | `probe_*` params | VERIFIED: 5.9/min, XTE during probes 8 mm vs 10 baseline (invisible). ~49 taps banked. Per-tap κ SNR ≈ 0.12 (naive) → needs stacked estimator; consider 12 µs amplitude (n ∝ 1/amp²). |
| direction-mismatch fault | threshold = breakaway (0.045), debounced from motion (0.025) | live |
| watchdog failsafe | `tools/esc_watchdog.py`, systemd user unit | ACTIVE |

**Effort-scale bug history (do not repeat):** (a) observing the live pulse
without a transient gate → g dove to 0.80 while the P-term braked hot-launch
overshoot ("world got easier"); (b) the "fix" — observing feedforward+trim —
was TAUTOLOGICAL (with v converged, ratio ≡ 1) and froze the estimator at
exactly 1.000 for a full drive. Final form: live pulse + rolling-age gate.
Lesson: **an estimator reading a suspiciously constant value is as broken as
one reading a wild value.** Replay validation against contrasting logs
(16:46 sagged ≈1.05–1.09 vs 19:59 fresh ≈0.97) is the cheap test.

## 5. Learning-architecture lessons (why things are the way they are)

1. **Closed-loop identification is endogenous.** The steering RLS converged
   to gain≈1/small-bias with no XTE benefit — the textbook artifact
   (Gustavsson/Ljung). Cure: exogenous excitation (the probe taps) or IV.
2. **Signal-to-confounder decides what is passively learnable.** Floor: yes
   (certified). Steering lag: yes via probes. Throttle map slope: NO
   (needs paired scripted contrasts — done once, never again).
3. **Battery = rug = temperature = one latent gain**, no sensor; value
   persists, confidence resets. A stopped robot has zero evidence — a
   cautious first touch after a gap is information-theoretically required.
4. **Semantics beat gains**: most "smoothness" wins came from teaching the
   state machine what stalls MEAN (intended stop vs traction failure), not
   from tuning.
5. **Compensate a delay exactly once.** The lag is applied in the preview
   only. Historic bug class: lookahead + preview + delay-aligned ID all
   compensating the same 0.2 s.
6. **Every claim gets verified in the next drive log**; every experiment
   script must read deployed calibration (yaml), not defaults. Smoke-test
   pattern: run the controller disarmed 7 s, grep Traceback, then
   verify-and-delete the stub CSV it creates (VERIFY before delete — a
   blind `rm` once ate a real log).

## 6. Standard drive-log analysis (the verification loop)

Logs: `~/.robot/drive_logs/adaptive_drive_*.csv`, 50 Hz rows, `fresh_odom`
marks real odometry ticks (~10 Hz). Per-run standard metrics used all day:

- **Launch peaks**: for each `startup|recovery → rolling` transition, max
  |measured_v_mps| over the next 2 s. Track median / p90 / count>0.5.
  (History: median 0.42 → 0.30–0.35; hot 21% → ~0 on calm runs.)
- **Rolling exits**: transitions out of `rolling` with `fault` — recovery
  count, direction-mismatch count, obstacle-stop seconds, planned_stop
  entries + fallbacks.
- **XTE**: |cross_track_error_m| while rolling (median ~0.007–0.010,
  p90 ~0.035 — stable all day; use as regression canary).
- **Learners**: `floor_estimate_*`, `floor_observer_json`,
  `breakaway_models_json` (start vs end), `effort_scale(±sd)`, `probe_id`
  count, `throttle_*_trim_us`.
- **Context caveat**: runs are only comparable on similar courses — the
  21:34 "regression" was a deliberately reverse-heavy stress course.

## 7. Remaining roadmap

- **Verification drive** (any normal drive): direction-settle debounce
  (mismatch faults 58 → ~0 expected), effort-scale first honest trace,
  reverse-arc behavior at the new 0.13 floor.
- **Stage 3b — probe-fed lag estimator (shadow):** stack probe responses
  (probe_id-tagged; commands also flow through `pulse_history`, so
  `_learn_steering`'s window machinery can be reused). Estimate lag in
  τ + d₀/v structure by regressing stacked response timing on 1/v across
  natural speed spread. Certify against 3.1 numbers. Also probe-based
  steering gain/center (IV: planner path curvature is exogenous).
- **Stage 4 — promote/rollback + certification report:** auto-compare every
  shadow estimate to ground truth + to applied constants on held-out
  prediction; promote at stop-events only; auto-rollback on KPI regression
  (use section 6 metrics). First promotions queued: floor observer (ready),
  effort scale (scale feedforwards + breakaway kick; then DELETE the
  session-warmup counter — caution proportional to uncertainty replaces it).
- **Stage 5 — new-body bootstrap:** confidence-gated speed cap, probe-priority
  first drives, blank memory start. Optional primitives from 3.4: opposite-
  pulse braking at cusps and post-launch.
- **Exam 1 (amnesia rehearsal):** back up + wipe runtime yaml, supervised
  ordinary drive, robot must relearn its certified numbers.
- **Exam 2 (robot #2):** birth certificate + goals. This is the deliverable
  proof. Also relevant then: masterplan targets Nav2 Lyrical (MPPI with
  actuator-delay model, `enforce_path_inversion` for the RPP churn).

## 8. Open watch items

- Direction-settle debounce unverified (58-fault storm baseline).
- Effort-scale trace needs one calm drive (expect 0.95–1.10, no flatlines,
  no dives).
- Reverse mid-path stalls at sagged battery (trim clamp ±40 µs may be tight
  on a dying pack — only if it recurs on fresh charge).
- Reverse tight-arc tracking inherently worse (trailing-steer physics);
  same-course A/B needed to judge the 0.13-floor trade.
- Floor observer reverse died_ema inflates during battery-marginal periods.
- Probe SNR economics: 8 µs needs ~283 naive taps; stacked estimator and/or
  12 µs amplitude before judging stage 3b feasibility.
- Restored planning docs (`docs/masterplan.md` + siblings) are HISTORY —
  deliberately unedited; several verdicts superseded (quantization, lunges,
  §6 conflicts largely explained by the empty-costmap confound).

## 9. One-day metrics arc (for morale and for baselines)

| metric | morning | evening |
|---|---|---|
| launch peak median | 0.42 (0.7 max) | 0.30–0.35 (0.50 max calm) |
| hot launches | 21% | ~0 calm / battery-transient only |
| obstacle-stop time | 268 s / run | 0–15 s |
| planner sees walls | never had | yes (verified cells) |
| stall semantics | kick everything | planned_stop @ cusp+goal |
| XTE median | 7–10 mm | 6–10 mm (unchanged, good) |
| self-learners running | 0 | 3 (floor certified, effort + probes shadow) |

## 10. Post-handoff cusp work (2026-07-12)

The 02:08 drive completed the direction-debounce verification but showed the
mitigation was not a solution: 478 raw sign flips, 503 local path-direction
flips, 330 `direction settle` episodes / 195.9 s in 539.9 active seconds.
Steady steering remained accurate; the failure clustered at cusps.

The behavior tree was changed from unconditional 1 Hz replanning to retain a
valid path and recompute only for a changed goal or `IsPathValid == false`.
The 02:28 drive then published only six plans and local path-direction flips
fell to five. It exposed two independent assumptions:

1. `path_timeout_s: 2.0` expired each committed plan, disabling lag preview,
   XTE, cusp distance, and planned-stop semantics for 92% of rolling.
2. Jazzy RPP still skipped inversions inside a fixed path: 30 raw sign flips
   from those six plans (upstream RPP Ackermann pruning defect).

Prepared next build:

- committed plan lifetime (`path_timeout_s: 0`, valid until replacement);
- installed Jazzy MPPI controller with Ackermann model, min radius 0.87 m,
  `enforce_path_inversion: true`, PathAngle mode 2, +/-0.30 m/s and
  +/-0.345 rad/s envelope;
- 500 trajectories x 25 steps x 0.10 s (2.5 s horizon; reduced after Pi smoke
  test), visualization off;
- hard adaptive-node path-segment latch: MPPI commands opposite the current
  segment are held at zero until the robot is stopped within 0.12 m of the
  chronological cusp;
- preview nearest-point search is monotonic and restricted to the latched
  segment, so overlapping later geometry cannot steal the steering state;
- any nonzero command below the measured 0.12 m/s reliable floor is quantized
  to 0.12 while preserving curvature; exact zero remains a cusp/goal stop;
- new CSV telemetry: `command_gate_reason`, `path_progress_index`, and
  `path_segment_end_index`.

### 10.1 Failed 10:56 MPPI drive and correction

The first armed MPPI run disproved the downstream hard-gate design. Steering
was active (turning rows were commanded about 126 us off center at median, and
measured curvature broadly followed requested curvature), but tracking failed:
rolling XTE reached 1.60 m and heading error reached 1.15 rad. The gate held
MPPI recovery reversals roughly 0.56 m before a planned cusp, while Collision
Monitor stopped forward motion. Eleven recorded plans included genuine
Reeds-Shepp reversals; the last two had direction sequences `- + - +`, so the
global planner did know how to request a three-point maneuver.

Prepared correction:

- delete command modification by the adaptive-node cusp gate; MPPI owns
  recovery direction and its native `enforce_path_inversion` owns path cusps;
- retain chronological segment state for telemetry only;
- disable the RPP-specific pure-pursuit preview curvature override with MPPI
  (`apply_path_preview_compensation: false`). It failed held-out replay and
  changed 185 accepted command rows by >0.02 1/m, including 13 sign changes;
- retain direction-specific static steering maps, mechanical lag telemetry,
  actuator slew, physical curvature clamp, and direction debounce;
- extend MPPI from 2.5 to 3.0 s at the Pi-safe 500 samples, and strengthen
  PathFollow / PathAngle weights from 5.0 / 2.2 to 8.0 / 6.0;
- tighten goal completion from 0.25 m / 0.55 rad to 0.15 m / 0.30 rad;
- restore the planner-honesty link at every launch. The worst credible
  forward/reverse and left/right RLS branch may contract the nominal 1.15 1/m
  envelope, never expand it. The resulting single curvature/radius pair is
  injected into Smac, MPPI, and the actuator clamp before lifecycle configure.

The nominal 0.87 m radius is `1 / 1.15`; it is a conservative production
limit, not the hardware endpoint. Runtime learning can now make the promised
radius wider if steering degrades. It cannot make the radius tighter without a
new supervised limit certification.

### 10.2 Failed 11:17 tight-cusp run and correction

The first forward goal succeeded. The second plan was a 0.532 m reverse arc to
a cusp followed by forward motion. Its heading change required approximately
-1.14 1/m, but MPPI commanded only -0.218 1/m median while following that
reverse segment. The actuator passed the request faithfully. The robot came
within 0.106 m of the cusp but retained 0.459 rad yaw error, narrowly failing
MPPI's 0.40 rad inversion tolerance. It then continued reversing until 1.37 m
beyond the cusp, switched once, and drove forward in a large loop. Total travel
was 4.13 m and the final pose was near the start. This was neither sign chatter
nor the removed downstream gate.

Prepared correction:

- Smac plans at 80% of the learned physical curvature. With the current 1.15
  1/m envelope, planned minimum radius is 1.087 m while MPPI and the actuator
  retain the full 0.870 m physical radius for correction;
- a bounded path-curvature floor activates only when MPPI requests the same
  turn direction but less curvature than the lag-led pure-pursuit geometry.
  Stronger or opposite MPPI steering remains authoritative, and the assist
  disengages outside 0.20 m XTE or 0.50 rad heading error;
- a cusp-miss guard records closest approach. After entering 0.18 m, moving
  0.08 m away while still commanding the old segment blocks only that old
  direction. Opposite recovery remains allowed;
- progress timeout is reduced from 20 to 6 s so a guarded miss triggers Nav2
  recovery/replanning promptly;
- CSV/debug telemetry adds `steering_assist_active` and
  `steering_command_source`.

Counterfactual replay of the failed bag applies -1.15 1/m from the first
reverse approach (instead of -0.694 initially / -0.218 median) and trips the
cusp guard at 10.53 s, 0.205 m from the cusp after a 0.106 m closest approach,
rather than permitting another 7.3 s of travel away.

### 10.3 Architectural rollback after the 11:33 regression

The curvature-floor/cusp-guard build exposed the fundamental ownership error.
The run blocked 4,697 controller ticks (93.9 s) in the external cusp latch;
Collision Monitor separately zeroed planner motion for 95.7 s. On nonzero
post-monitor commands, speed was raised downstream for 36.9 s and curvature
was changed downstream for 53.2 s (median curvature mutation 0.451 1/m). MPPI,
the safety monitor, and hardware therefore operated on three different models.

One failed goal began with direction segments of only 0.095 m reverse, 0.142 m
forward, then 1.183 m reverse at about -0.987 1/m. MPPI drove forward 3.3 s and
then nearly straight reverse for 30.4 s (median raw curvature +0.011 1/m),
traveling 6.93 m. `IsPathValid` generated 24 replacement plans while the goal
was unchanged, resetting the external observer. For 1,212 of 1,525 reverse
command ticks, that observer believed the active full-plan direction was
forward. A passive `/plan` subscriber cannot reproduce MPPI/RPP's internal
transformed and pruned controller path.

Historical comparison made rollback mandatory:

| metric | committed RPP 02:28 | MPPI + floor 11:33 |
|---|---:|---:|
| rolling XTE p90 | 0.018 m | 0.083 m |
| heading error p90 | 0.093 rad | 0.303 rad |
| nearest-path distance p90 | 0.032 m | 0.221 m |

The plant remained healthy: current RLS gains were 0.983--1.006 with small
biases, consistent with the held steering experiments. The regression was
controller architecture, not steering calibration.

Current rollback:

- restore Jazzy Regulated Pure Pursuit as the default segment tracker;
- add a `FollowPath` action proxy. It splits the committed Reeds-Shepp path at
  each direction cusp and sends one inclusive direction run at a time to a
  remapped `/follow_path_backend` controller action, with 0.60 s neutral dwell;
- compute a path only for a new goal (`GoalUpdatedController`); do not replan
  every second merely because the already-diverged path becomes invalid;
- restore planner minimum trackable radius >=1.30 m and analytic expansion 6 m;
- disable external `/plan` interpretation, curvature floor, and cusp guard in
  the actuator bridge;
- move minimum sustainable speed, physical curvature projection, and
  steering-transition slowdown before Collision Monitor;
- execute the monitor-approved nonzero Twist unchanged downstream. Watchdog,
  stale-sensor, and local-obstacle logic may still replace it with neutral.
- disable steering probe taps during rollback verification so even intentional
  identification excitation cannot alter approved curvature.

MPPI returns to bench-only status. Do not promote it again on Jazzy without an
A/B gate, actuator-delay/open-loop support, and proof that its command is not
mutated downstream.

### 10.4 Trackability learner after successful rollback run

The next RPP/dispatcher drive was much better: 14 goals succeeded with no
controller or progress failures, and all 42 dispatched direction runs completed
(21 forward, 21 reverse). Offline matching measured 0.016 m median / 0.120 m
p90 XTE overall. Failures were branch- and entry-state-dependent, not evidence
for one universal mechanical radius. The prior `max(1.30 m, physical/utilization)`
launch rule was therefore still a fixed planner assumption and has been removed.

`path_segment_dispatcher` now owns a persistent four-branch trackability model:

- forward-negative, forward-positive, reverse-negative, reverse-positive;
- eligible only for >=0.50 m segments with clean <=0.06 m / <=0.15 rad entry,
  at least ten moving samples, <=5% Collision Monitor blocking, and no obstacle,
  recovery, or overspeed contamination;
- pass threshold is XTE p90 <=0.05 m and heading p90 <=0.15 rad;
- three clean near-boundary passes expand curvature by at most 0.02 1/m per
  update; two comparable failures contract it faster;
- the weakest branch is bounded by the learned physical curvature times 0.80
  utilization and becomes Smac/RPP's radius on the next launch.

The 1.30 m radius remains only a bootstrap prior when no evidence exists. State
is atomically written to `~/.robot/planner_trackability.yaml`. Each segment's
entry, tracking, blocking, endpoint, eligibility, verdict, and resulting radius
is published on `/planner_trackability` and included in the flight-recorder bag.
Intermediate cusps use a dedicated 0.07 m / 0.15 rad goal checker. Non-final
direction fragments below 0.18 m are explicitly skipped rather than silently
succeeding under the final goal's broad tolerance.

The first build with two goal checkers exposed a Jazzy integration rule: the BT
normally sends an empty final `goal_checker_id`, but that is rejected when the
controller server has multiple checkers. Forty FollowPath attempts were rejected
before control and BT recovery BackUp produced the only movement. Fixed by
mapping an empty final ID explicitly to `goal_checker`; intermediate segments
still select `cusp_goal_checker`. Regression coverage verifies empty, explicit,
and intermediate IDs. Failed-build checkpoint: `62bc8ba`.

Operational rule requested by the owner: checkpoint the package in Git before
every physical experiment and record the short SHA with the run. Never begin an
armed run from an uncommitted tree; this makes each experimental configuration
directly reversible and attributable to its CSV/bag.

### 10.5 Event-triggered replanning and rocking-loop guard

Run `adaptive_drive_20260712_124024` is tagged
`experiment-20260712-124024` at commit `92aa9ac`. It proved ordinary segment
tracking can be good, but exposed two recovery loops:

- after FollowPath failed, its inner RecoveryNode cleared the local costmap and
  replayed the original path; one retry began 1.461 m and 1.397 rad from the
  first segment before the outer recovery eventually generated a fresh path;
- a later declared forward segment ran for about 111 s while RPP alternated
  forward and reverse. The robot rocked enough to keep SimpleProgressChecker
  satisfied without chronological path progress. Goal preemption ended it.

The next build removes the inner stale FollowPath retry. A failure reaches the
outer recovery, clears both costmaps, and recomputes once from the current pose.
The dispatcher independently projects odometry onto the active segment and
aborts for fresh planning after 6 s without 0.05 m new along-track progress, or
after 0.75 s of an opposite-direction request. A moving obstacle is allowed a
2 s wait before BackUp. Offline replay would have stopped the observed rocking
loop 9.4 s after segment start instead of allowing 111 s. A dense disarmed
FollowPath integration test confirmed backend cancellation and frontend error
105 (`FAILED_TO_MAKE_PROGRESS`) complete cleanly.

### 10.6 July 12 13:08 run and next correction

Run `adaptive_drive_20260712_130802` is tagged
`experiment-20260712-130802` at commit `975c4e4`. It was materially better:
14 frontend goals began, 11 succeeded, three were preempted, and none ended in
a controller failure. Thirty direction segments were scored. Two
wrong-direction and two no-progress aborts caused fresh planning in about
0.24--0.56 s instead of replaying stale geometry. Curvature saturation fell to
20.7% from 35.4% in the preceding run. Steering RLS stayed healthy
(gain 0.972--1.007, delay about 0.30 s), so the remaining defects were command
semantics and replanning latency, not a broken steering plant.

The planner radius reported as 1.30 m was not yet learned. It was the bootstrap
prior mixed with incomplete evidence: forward-negative had no eligible
segments; the other three branches had three each, with reverse-positive only
passing one. The state now explicitly publishes `bootstrap_prior`,
`mixed_prior_and_evidence`, or `learned` plus confidence. It may claim learned
only after every direction/turn branch has at least three eligible observations.
The evidence history is retained in `~/.robot/planner_trackability.yaml`.

Two additional causes were visible:

- a committed path was reconsidered mainly after RPP/Collision Monitor was
  already close to an obstacle; the dispatcher now checks the next 1.50 m via
  Nav2's footprint-aware `IsPathValid` service every 0.50 s and replans after
  0.75 s of persistent invalidity;
- six of 30 launches exceeded 0.35 m/s despite a 0.30 m/s request, and seven
  command episodes traveled under 0.10 m. Slow and short moves now use
  pre-steer plus bounded learned-breakaway pulse-and-coast, never an escalating
  held launch kick.

Goal replacement is serialized: the outgoing backend action must cancel before
the next frontend goal dispatches. Disarmed integration verified both this
ordering and a stationary 0.35 m short-path command. The latter produced six
0.48--0.50 s launch pulses, returned to neutral between every pulse, stayed
within the learned breakaway plus 6 us cap, and exited through the six-second
chronological no-progress guard. The smoke CSV was verified before deletion.

Do not raise speed for the next physical run. One-third faster is 0.40 m/s,
which exceeds the current 0.35 m/s forward and 0.30 m/s reverse envelope and
would worsen the observed late-obstacle margin. If this build passes, stage a
separate 0.35 m/s-forward experiment while retaining 0.30 m/s reverse.
