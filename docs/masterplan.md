# Masterplan — the robot drives to its goal smartly

*2026-07-04. Consolidates auto_learn.md (actuation learning), platform_survey.md
(prior art), stack_map.md (architecture), today's safety-layer findings, and a
three-track research pass (RC platforms / Nav2-2026 / adaptive control — digests
in §4). This is the top-level document; the others stay as deep dives.*

## 1. The goal, and what "done" means

**One sentence:** given a goal pose, the robot plans a feasible path once,
drives it smoothly (no lunges, no weave), avoids obstacles including low and
moving ones, recovers on its own when blocked, and adapts its actuation maps
to battery/surface/temperature without hand-tuning.

**Definition of done — numbers, measured by drive_score/analyze_drive_log per
session (BARN-style, see §5-T4):**

| KPI | Today (2026-07-04 16:01 log) | Done |
|---|---|---|
| Goal success rate (goals reached / sent) | ~2 of many | > 90% |
| BARN score: success × (optimal time / actual time) | not computed | > 0.5 avg |
| Motion duty while a goal is active | 31% | > 80% |
| Go-segment median length | 0.1 s (!) | > 5 s |
| Stall ratio (cmd>0, ZUPT says stationary) | ~50% eras | < 10% |
| Jerk RMS (200 Hz IMU, lunge measure) | not computed | trend ↓, then floor |
| Steering wrong-sign rate, mid band | 59% | < 15% |
| Recoveries per goal | frequent | < 0.3 |
| Manual interventions / restarts per session | several | 0 |
| Config edits between sessions | most sessions | 0 across 3 sessions |

## 2. System truth — constraints we plan around, not against

- **No encoders, no motor feedback, no steering feedback. No new sensors.**
  Feedback = lidar ICP + IMU EKF at ~10 Hz (100–200 ms effective latency) +
  raw IMU at 200 Hz. Battery voltage unmeasurable (power HAT dead 2026-06-15).
- **Actuators:** PCA9685 (one PWM frequency shared by all 16 channels) →
  hobby ESC (F/R mode; breakaway 0.6–0.8 raw drifting with temp/battery/surface;
  rolling band ~2 ticks at 100 Hz) + steering servo behind a servo-saver
  (1–2 s wind-in to lock, load-dependent, hysteretic).
- **Fwd/rev are NOT mirror images in the pulse domain (2026-07-05, the big
  lesson).** The ESC's reverse output stage (30 A vs 60 A forward, per the
  Hobbywing manual) gives a *different microseconds-to-speed curve* on each
  side of neutral: near our normal command, reverse rose ~2.5x hotter per
  µs than forward (measured 0.50 vs 0.61 m/s per raw-effort unit; eff −0.48
  → ~1 m/s² rocket vs +0.41 → ~0.4 m/s²). Two days of reverse "fixes" all
  failed because they tuned LOGICAL EFFORT (deadbands, max_speed, kick,
  envelope, PI) — inputs to logical_to_raw() — while the asymmetry lives
  DOWNSTREAM, in the tick map, which assumed the two sides were symmetric.
  **The cure was one gain on the OUTPUT: `esc_rev_scale` (0.75) in
  ackermann_driver, applied to the pulse deviation where esc_throttle < 0,
  after all effort logic, before esc_inverted.** Robot-frame, reverse-only,
  forward untouched. General rule this taught: *if one direction/regime is
  wrong by a ~constant ratio, fix it in the pulse domain, not by fighting
  the effort maps three layers up.* (Proper fix = recalibrate esc_min_tick
  from a reverse sweep so the tick map is honest; the scale is a correct,
  simpler equivalent pinned near one operating point.)
- **Compute:** Pi 5, ~1 spare core. ROS 2 Jazzy + Nav2 1.3.12 (+ Kilted smac
  overlay). MPPI on Jazzy = xtensor build, measured too heavy here.
- **Physics results we must respect (research-confirmed, §4c):**
  - A speed loop through 10 Hz laggy feedback cannot exceed ~1 Hz closed-loop
    bandwidth. It will never catch a breakaway surge. Feedforward must carry.
  - With ~2-tick actuator resolution, an integrating controller **must**
    limit-cycle (quantization limit-cycle theorem). The lunge is not a tuning
    problem; it is a resolution problem.
  - Below the ESC's minimum continuous speed there is no continuous-throttle
    solution — deliberate pulse-and-coast or accept the floor.

## 3. Obstacle inventory (ranked by what blocks the goal most)

1. **Throttle resolution + two-threshold stiction → lunges/tiny steps.**
   Evidence: 88 go-segments median 0.1 s (16:01 log); rolling band 0.47→0.49
   raw = 0.25→0.60 m/s. Root cause per §2. Fix: T1 (resolution + kick shape),
   then M3 (adaptive map).
2. **Steering lag + no feedback → over/understeer, weave, clipped corners.**
   Evidence: mid-band steering 59% wrong-sign; 1–2 s wind-in; yaw PI fights
   kick transients (compares yaw rate, not curvature). Fix: M2.
3. **Post-contact / near-wall freeze.** Two mechanisms, both partially fixed
   2026-07-04: (a) collision monitor was direction-blind then silently ceased
   publishing (stop_pub_timeout) — fixed with velocity_polygon fwd/rear split,
   probe-verified; (b) planner "Start occupied" near walls — long footprint +
   inflation + no start tolerance in Smac. Remaining work in T0/M5.
4. **Controller skips reverse maneuvers (Jazzy RPP cusp bug #4757, fixed only
   in Lyrical).** Mitigated by short lookahead today; real fix is M5.
5. **Nothing measures progress.** drive_score exists but KPIs above are not
   all computed; regressions get discovered by vibes. Fix: T4.
6. **Plant drift (battery/temp/surface) invalidates constants.** auto_calib
   M1 handles breakaway; speed maps and steering are still hand-set. Fix:
   M2/M3 (and breakaway drift doubles as the battery-state estimate).
7. **~~Reverse lunges faster than forward~~ FIXED 2026-07-05.** Root cause
   was the fwd/rev pulse-domain asymmetry (§2); cure was `esc_rev_scale`
   0.75. Reverse now tracks forward. Leftover reverse scaffolding (overspeed
   hysteresis, reverse envelope, rev_pulse fallback) was built to fight this
   at the effort layer and is now redundant backstop — candidates for the
   M4 delete pass once a few good reverse drives are logged.

## 4. Research digests (2026-07-04) — what the world does

### a) RC platforms (MIT RACECAR, MuSHR, F1TENTH, AutoRally, DonkeyCar, Duckietown, feldfreund)
- VESC platforms buy motor feedback and reduce everything else to a 4-param
  linear calibration; even they compute odometry from **commanded** steering.
  Nobody compensates servo lag; nobody without feedback regulates speed
  (DonkeyCar: calibrated open loop + launch kick, tolerate speed variance).
- AutoRally: identify dynamics from ~30 min structured driving; controller
  outputs raw effort through the model, **no inner speed loop**. Lesson:
  actuator nonlinearity belongs in feedforward, feedback only trims.
- **New, directly usable:** Pi 5 RP1 hardware PWM (GPIO 12/13/18/19) has
  ~ns resolution — move the ESC wire there, keep servo on PCA9685; avoids the
  shared-frequency constraint entirely. Also: hobby ESCs often expose
  programmable neutral-range/punch (check the QuicRun with a program card);
  PCA9685 oscillator drift audit still pending (survey item #1).
- feldfreund (farm): dual-RTK moving-base + row-following as an action +
  real-time motor loop on a microcontroller — reference for the farm phase.

### b) Nav2 in 2026 (verified releases and PRs)
- Jazzy 1.3.12 (ours) < Kilted 1.4.2 < Lyrical/main 1.5 (Ubuntu 26.04 LTS,
  no binary tag yet as of 2026-07-04).
- **Kilted+:** MPPI rewritten in Eigen (PR #4621, 40–50% faster, ARM-viable —
  the Pi 5 blocker was Jazzy's xtensor build); Smac goal_heading_mode (we
  already have via overlay); **Route Server** (lane graphs — the farm answer
  to crop-row ambiguity).
- **Lyrical/main:** MPPI `model_delay_vx/wz` (PR #6154, merged 2026-06-08) —
  first stock controller that models actuator lag; docs example is 600 ms
  steering delay, ours is 1–2 s. MPPI `open_loop: true` (PR #5617) for
  laggy/odometry-poor platforms — made for encoder-less robots.
  `FeasiblePathHandler` (PR #5446, closes #4757) gives RPP cusp-aware
  pruning. DWPP option in RPP. Per-plugin costmap clearing (#6140) — clear
  obstacle layers only, keep the live SLAM static layer.
- **#4425 thread (MPPI reverse on Ackermann) maintainer guidance:** rear-axle
  base_link (we comply); **dwell at Reeds-Shepp cusps until steering settles**
  (the model assumes wheels turn while stopped — with our servo this is
  mandatory and we don't do it); oscillating reverse legs are usually the
  REPLANNING RATE, not the controller → BT "replan only if path invalid".
- **Live misconfiguration found:** Smac `analytic_expansion_max_length`
  defaults to 3.0 m; docs require ≥ 4–5× minimum_turning_radius (ours 1.3 m
  → need ~6 m) or "planning times will begin to spike". We never set it.
- RPP lag levers: lookahead ≥ v·τ_steer (0.25 × 2 s = 0.5 m; our
  min_lookahead 0.35 is below it — but was chosen to stop cusp-skipping;
  genuine trade-off until M5 removes the bug). `use_fixed_curvature_lookahead`
  is the docs' lag answer but **failed here 2026-07-04** (collapsed/zero
  curvature commands two runs) — do not blindly retry on Jazzy; revisit on
  Lyrical.

### c) Adaptive control / estimation theory
- Limit-cycle cures in order: actuator resolution (PWM freq + **sigma-delta
  dithering between adjacent ticks** — inertia is the low-pass; buys 1–2
  effective bits), error dead-zone on the integrator (sized wider than the
  breakaway jump), feedforward dominance, pulse-and-coast below the floor.
- Velocity estimation: predict with IMU accel / correct with ICP is standard;
  but ForzaETH **dropped accelerometers** from their EKF for vibration noise —
  bench-check our accel noise before investing (survey said the same).
  ZUPT (variance gate + throttle-below-breakaway gate) is cheap and shipped
  here already (v3).
- Adaptation: ArduPilot Rover's 2-param, event-triggered cruise learning is
  the shipped precedent — vindicates our event-based approach. RLS/MRAC
  explicitly warned against (covariance windup on sparse bursty data). Every
  learned param needs exploration or actuation-sample rejection (we learned
  this independently — the seeding ratchet).
- **Steering without a sensor: gyro curvature trim.** Actual curvature
  = yaw_rate/v at 200 Hz; slow integral on (κ_cmd − κ_meas) compensates
  servo-saver wind-in and drift, and learns the steering map as a by-product
  (fixed-wing trim-hold pattern). This is our yaw PI moved to curvature
  space — already queued as phase 2; research confirms it's the right target.
- Metrics: BARN score + Arena-Rosnav family (time-to-goal, path efficiency,
  jerk RMS, smoothness) — adopt, don't invent.

## 5. The plan

Ordering principle (AutoRally lesson): **make the actuator honest first**
(resolution + calibrated feedforward), then steering, then let learning
retire constants, then upgrade the brain. Each phase has an acceptance
metric; a phase isn't done until the metric holds for 3 sessions.

### T0 — this week: config + hygiene quick wins (no new code)
- Smac: set `analytic_expansion_max_length: 6.0` (docs-flagged misconfig).
- BT: adopt replan-only-if-path-invalid semantics (maintainer-endorsed fix
  for reverse flip-flopping; we already replan at 0.33 Hz — this goes
  further). Keep the START_OCCUPIED → backup → clear → replan branch.
- Cusp dwell: at direction changes, hold position and pre-set steering for
  ~1 s before rolling (extend the direction interlock — it already holds
  ~1.3 s; make it *steer while holding* instead of centering).
- Finish workspace hygiene: remove src/build, src/install, src/log spills +
  src/ackermann_robot/install|log; wipe+rebuild install/ackermann_robot so
  every data file is a symlink (half are stale plain copies today).
- Field-verify the 2026-07-04 collision-monitor velocity_polygon fix +
  Start-occupied escape on the real robot.
- **Acceptance:** post-collision escape works on hardware; zero
  "File exists" build failures; no planner-time spikes near walls.

### T1 — actuation resolution + start shape (the anti-lunge phase)
1. Bench-audit PCA9685 oscillator (survey item; scope or GPIO capture).
2. ESC signal wire → Pi 5 RP1 hardware PWM (ns resolution; servo stays on
   PCA9685). Fallback if RP1 misbehaves: PCA9685 @ 200–400 Hz (verify servo
   tolerates shared frequency) + sigma-delta dither on the throttle tick.
3. Check ESC programmable neutral-range/punch (program card) — free
   deadband narrowing if supported.
4. Re-run the raw sweep on the new resolution; recalibrate rolling maps.
   Formalize the kick as DonkeyCar-style launch: seeded amplitude (from
   auto_calib), fixed short duration, drop straight to rolling feedforward;
   IMU accel-step release (~20 ms) instead of waiting for 10 Hz odom.
5. Pulse-and-coast (existing `slow_pulse`, currently off) re-enabled ONLY for
   commands below the measured floor, duty regulated by the slow loop.
- **Acceptance:** go-segment median > 5 s; stall ratio < 10%; jerk RMS down
  ≥ 3× on the same course; zero watchdog trips on a healthy pack.

### M2 — steering (from auto_learn.md, upgraded by research)
- **SHIPPED 2026-07-12: planner honesty link.** Physical steering RLS defines
  only the outer actuator envelope. A persistent four-branch learner scores
  clean path-tracking segments and sets Smac minimum_turning_radius + RPP
  min-radius from the weakest forward/reverse, left/right branch at restart.
  Radius 1.30 m is a bootstrap prior, not a hard clamp. Repeated clean
  near-boundary passes explore slowly; comparable failures contract faster.
- Yaw loop → **curvature space** (κ = w/v from the 200 Hz gyro; slow integral
  trim). Passive steering-map learning (steer_effort ↔ κ pairs, per-direction
  gain + offset, Duckietown 24% asymmetry precedent) under M1-style guards.
- Steering slew limit matched to the servo-saver (~3 rad/s equivalent), and
  command curvature early via lookahead ≥ v·τ once M5 removes the cusp bug.
- Keep backlash-inverse OFF until the PI is calm (it over-compensated on the
  fixed linkage); re-evaluate with the learned hysteresis width.
- **Acceptance:** mid-band wrong-sign < 15%; steer saturation < 10% of moving
  time; nav2_track_err median down 2×.

### M3 — active excitation → learned speed maps (unchanged from auto_learn.md)
- Lock-in dither identification of G per direction; deadbands/max_speed
  live-derived; breakaway drift published as battery-state proxy.
- **Acceptance:** drive_score > 80 across 3 sessions, zero config edits.

### T4 — measurement (parallel, cheap, do alongside T1)
- Extend drive_score/analyze_drive_log to compute every KPI in §1
  (BARN score needs planned-path time: log Smac path length + cruise speed).
- One-line session verdict printed at drive end + appended to a CSV ledger
  so trends survive across days.
- **Acceptance:** every table-§1 number computable from one command.

### M5 — the brain upgrade (Jazzy → Lyrical, when 1.5.x binaries tag)
- Ubuntu 26.04 reinstall on NVMe; config migration (~a day: RPP renames,
  PathHandler split, MPPI plugin syntax, CheckPoseOccupancy).
- MPPI Eigen with `motion_model.plugin: AckermannMotionModel`,
  `min_turning_r: 1.3`, `open_loop: true`, `model_delay_wz` ≈ measured servo
  lag (M2 gives us the number), `model_delay_vx` ≈ ESC ramp, 10–15 Hz,
  batch ~1000 → measure on Pi 5; RPP + FeasiblePathHandler as fallback.
- Per-plugin costmap clearing (obstacle layers only — protects live SLAM).
- Farm: Route Server lane graphs down crop rows + GPS/RTK fusion (existing
  farm memory), feldfreund as architecture reference.
- **Acceptance:** recoveries per goal < 0.3; reverse maneuvers complete
  without oscillation; controller CPU < 60% of one core.

### M4 — delete scaffolding (after M2+M3, per auto_learn.md)
- Each of the seven compensating mechanisms (kick, seed, cooldown, interlock
  hold, watchdog, IMU release, PI stall-hold) is re-justified in writing or
  deleted. Target ≤ 4.

## 6. Conflicts: research says / we measured (do not blindly "fix")

| Research recommendation | Local evidence | Resolution |
|---|---|---|
| `use_fixed_curvature_lookahead: true` for lag | Collapsed curvature cmds, 2 runs, 2026-07-04 | Retry only on Lyrical MPPI/RPP, not Jazzy |
| `min_lookahead_dist` ≥ 0.55 (v·τ) | 0.35 chosen to stop cusp-skipping (#4757) | Keep 0.35 until M5's FeasiblePathHandler, then raise |
| `change_penalty: 0.0` (admissibility) | 0.25 stopped plan flip-flopping | Keep 0.25; revisit with BIDIRECTIONAL goals on M5 |
| IMU-accel velocity prediction at 50 Hz | ForzaETH dropped accels (vibration); our vib envelope is the noisy proof | Bench-measure accel noise first; low priority |
| Velocity smoother in the chain | Lifecycle/latency risk; 10 Hz odom → OPEN_LOOP only | Only reconsider at M5; CM ordering: smoother → monitor |

## 7. Explicitly not doing (standing decisions, with reasons)

- End-to-end RL (compute, crashes); visual odometry (benchmarked: 6× CPU for
  10–12 Hz); fast inner speed loop (impossible per §2); MPPI on Jazzy
  (xtensor/ARM, measured); new sensors/hardware (owner constraint) — except
  the ESC-wire-to-RP1 move, which is a wire, not a sensor.
- Adapt-everything-always: adaptation stays event-gated + clamped + explored
  (seeding-ratchet lesson). Constants are LEARNED or MEASURED-AND-DATED.
