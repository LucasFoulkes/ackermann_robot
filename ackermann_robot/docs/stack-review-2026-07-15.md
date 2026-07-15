# Stack Review — 2026-07-15

Scope: the complete drive stack (hardware → learning → navigation).
Person tracker/follower explicitly excluded. 148 commits in the monorepo,
plus the driver repo (37bc356). No git remotes configured — both repos
exist only on this Pi.

## 1. Architecture (what we built)

Two repos, five moving parts, strict layering — a controller/hardware
split modeled on ros2_control without the framework (no encoders):

```
Nav2 (Smac Hybrid-A* planner + RPP controller + BT navigator)
        │ FollowPath
path_segment_dispatcher      cusp splitting, per-segment supervision,
        │ FollowPath backend   trackability learning → planner radius
RPP → /cmd_vel_nav → collision_monitor → /cmd_vel
        │
adaptive_ackermann_controller   THE BRAIN: effort-native [-1,1],
        │ EffortCommand          learns everything, hardware-free
pca9685_effort_driver           dumb + safe: affine effort→µs, 0.5 s
        │ I2C                    deadman, idle servo release
PCA9685 → ESC (double-tap reverse) + steering servo
```

Odometry: MOLA 2D lidar ICP → `/odom` (~10 Hz) via tf_odom_bridge.
No wheel encoders, no IMU in the loop, no battery gauge (by principle:
energy state is inferred behaviorally, never read).

- `adaptive_ackermann` — controller + dispatcher + adaptive_model +
  offline tools (drive_scorecard, steering_map_challenger)
- `adaptive_ackermann_msgs` — EffortCommand (stamped efforts)
- `ackermann_robot` — bringup (single launch w/ embedded cleanup),
  configs, birth certificate, urdf, tools, docs, history archive
- `pca9685_effort_driver` (own repo) — driver node + esc_watchdog backstop

## 2. What is learned vs declared

Birth certificate (operator-owned, never learned): geometry (wheelbase
0.2775 m), actuator endpoints/neutrals, safety ceilings. Fingerprint
`orange-crawler-1/wb0.2775`.

Learned, persistent (`~/.robot/adaptive_ackermann_runtime.yaml` v8):

| Quantity | Current state |
|---|---|
| Breakaway efforts | fwd −0.207 / rev +0.272, drop-on-raw-evidence |
| Cruise anchors + throttle trims | per-direction, converging post-pack-swap |
| effort_scale (fast battery compensator) | 0.973, session-adaptive |
| Steering RLS (4 branches) | gains 0.89–1.00, 1.4–2k obs each |
| Steering delay | 0.20 s estimate, conf 0.99 (see §5 caveat) |
| Polarities (steer + throttle) | discovered, both 1.0 |
| Floor observer (stall floors) | fwd/rev, ~0.12 m/s |
| Learned steering inverse map | ±0.6 region, challenger-refit, gated |
| Dispatcher trackability → planner radius | re-earning post-quarantine (0.51–0.77 1/m, 18 obs) |

Measured ground truths (drilled, not assumed): full-lock κ = **1.99
left / 1.40 right** (champion map extrapolation was right within 1%);
forward steering lag pure time ~0.24 s; stall floor ~0.08–0.12 m/s;
ESC double-tap reverse = factory firmware; closed-loop steering delay
**~0.40 s** (2× the estimator's number — it omits servo travel + MOLA
latency + odom filtering).

## 3. Certification status

- Amnesia exam (from-scratch learning): **24/26 goals**, 2026-07-13,
  pre-split. Not yet re-run on the effort-native 4-package split
  (converter round-trip verified instead).
- Steering map: validated at full lock (±1%) by the capability drill.
- Rollback attribution: fixed (rolling-age gate), verified 0 benched
  ticks in the confirmation session.
- Deadman drill: driver holds neutral on controller kill — verified.
- Watchdog: esc_watchdog self-match fixed, backstops the driver.

## 4. The 2026-07-14/15 arc (capability + stability campaign)

1. **Capability measured and promoted**: wheels never reached lock
   (clamp 1.15 + map asked 0.55–0.81 effort). Drill measured true full
   lock; `maximum_curvature_1pm` promoted 1.15→1.35 (≈1.30 executable),
   planner radius and cornering speed deliberately NOT coupled.
2. **Pack exhaustion episode**: behavioral latch fired correctly at full
   authority/no motion; stall-poisoned trackability quarantined
   (`planner_trackability.yaml.stall_poisoned_20260714`).
3. **The S-snake saga** (three lookahead tunings failed; root cause =
   0.40 s measured loop delay): resolved by restructuring steering to
   **feedforward-dominant** — dispatcher segment path curvature at a
   lag-compensated preview point is primary; RPP error feedback capped
   at 0.40 1/m, low-passed at τ=0.4 s, all-or-nothing blend (ce43cc1,
   469f45f). Straights confirmed better by user.
4. **Cusp handover** (d93de6a): reversal within 0.25 m of a committed
   segment's end commanding the next leg's direction = handover, not
   abort. Confirmed firing ("0.00 m remaining", 2× in one run).
5. **Fragment merge** (edd6b44): consecutive same-direction segments
   (after sub-0.18 m fragment skip) merged — killed the 9×3–4 s
   stop-crawl-stop chains.
6. **Launch lunge**: ramp now starts at the breakaway pop's actual
   speed instead of braking it (469f45f). Approach speed 0.14→0.18
   (was AT the stall floor → stall-pop cycles).
7. **Collision monitor gap closed** (51b2e3d): velocity polygons only
   covered the retired ≤0.30 m/s envelope — above that NO stop box was
   applied (monitor only warns). forward_fast/reverse_fast added.

## 5. Control pipeline (current, ~25 active mechanisms)

Steering command path: RPP raw → [path preview feedforward + capped
(0.40) low-passed (τ0.4) feedback, blend gated at 0.5 confidence] →
κ clamp 1.2978 → champion piecewise map + RLS correction (+ learned
inverse inside ±0.6 when not benched) → fixed-lag compensation
(0.235/0.168 s per direction) → polarity → effort.

Throttle: target ← RPP × regulated scaling × speed_limit (weak-plant) →
launch ramp from pop speed → throttle map + learned line + trims +
effort_scale → breakaway ladder w/ pre-steer + curvature boost →
gentle pulse-coast (≤0.10 requests) → PI w/ anti-windup states.

Supervision: dispatcher watchdogs (no-progress 6 s, wrong-direction
0.75 s → handover|abort, path-invalid 0.75 s), behavioral pack latch,
odom outlier gate, collision monitor, driver deadman, esc_watchdog.

**Caveat to carry**: the online delay estimator's 0.20 s is actuation-
onset only. The full loop is ~0.40 s. Any future feedback tuning must
use the measured number; if snake returns, measure period + cmd→meas
lag first (memory: steering-feedback-restructure).

## 6. Known defects / gaps (priority order)

1. **Effort-era amnesia re-certification not run** — the from-scratch
   path hasn't been proven since the 4-package split.
2. **Trackability confidence low post-quarantine** (18 obs) — planner
   radius conservative until it re-earns; expect wide plans short-term.
3. **ODAAC steals not yet implemented**: session-consolidation gate
   (would have prevented the stall-poisoning), isotonic map projection,
   variance-damped α, rejection-reason stats, per-side κ caps (left is
   worth 1.99 but symmetric clamp sits at the weak side's 1.35).
4. **3-point drill mode never run** (needs reverse thrust + space) —
   the original "rotation almost in place" goal remains unverified.
5. Direction changes cost ~2 s serialized (settle + double-tap +
   breakaway) — compressible if it matters.
6. Nav2 goal-cycle gaps (6–14 s of "nothing commanded" between goals)
   — BT/Smac latency, not robot health.
7. Hygiene: no git remotes (single-SD-card risk with 13 GB of
   irreplaceable training data); nested build/install/log dirs inside
   the repo (ignored but present); two drive_scorecard copies (live +
   history); several docs stale relative to this review.

## 7. Data assets

169 drive-log CSVs (7.0 GB), bags 6.1 GB, era snapshots
(`~/.robot_snap_main_20260714_1930`, pre-effort backup 2026-07-13),
quarantined trackability. These logs are the training corpus and the
forensic record — they are not backed up anywhere.

## 8. Bottom line

The architecture goal is real: a portable, effort-native, self-
identifying Ackermann brain with hardware behind a one-message
contract, learning everything from a birth certificate and its own
odometry. The learner is healthy (breakaway, maps, trims, battery
compensation all live and provably adapting). This week's instability
was command-side (Nav2 feedback vs. unmodeled delay) and is now solved
structurally, not by tuning. The robot turns at its true capability for
the first time. The biggest open risks are un-recertified from-scratch
learning, un-backed-up repos/data, and the one-session-poisoning class
that the consolidation gate (ODAAC steal #1) would close.
