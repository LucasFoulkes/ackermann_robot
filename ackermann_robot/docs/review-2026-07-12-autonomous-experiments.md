> **HISTORICAL** (frozen 2026-07-15) — superseded by `stack-review-2026-07-15.md` and `how-the-learner-works.md`; kept for provenance. Verdicts here may be stale.

# Review — first autonomous experiment day (2026-07-12)

Five autonomous Nav2 coverage sessions, two offline learning studies, and the
first live promotion of learned steering. This document synthesizes what was
discovered, checks it against the research record in `masterplan.md` §4, and
assesses distance to the north-star goal and the amnesia exam of
`universal-adaptive-ackermann.md`.

## 1. Experiment ledger

| Run | What | Outcome |
|---|---|---|
| Session 1 (151827) | 10-goal coverage, clamp 1.148 | 10/10; steering median 0.203, saturation 43.6%, slow cusp goals 63–66 s |
| Session 2 (153340) | same, learned clamp 0.92 active | 3 clean then obstacle-boxed aborts (front clearance 0.11 m); contamination rejection held |
| Session 3 (154316) | same + runner auto-relocation | 10/10; steering median 0.125 (−38%), saturation 0%, cusp goals 15–24 s |
| Offline Stage B | monotone map from 30 ordinary drives (14.6k samples) | Matches dense map on baseline, beats it on coverage bags; wrong-sign 0.2% |
| Offline Stage C | first-order steering state fit | Beats fixed delay on all 6 held-out splits (fwd τ≈0.4 s; rev τ≈0.4 s + 0.01 m) |
| Session 4 (160502) | blend plumbing in SHADOW | 9/9; learned within 6 µs (fwd) / 16 µs (rev) of champion, p90 13/37 µs |
| Session 5 (161047) | first BLEND (learned map driving, kappa <= 0.6) | Applied 47% of rolling, zero rollbacks, error 0.079 applied vs 0.089 champion regions; battery sag late (effort_scale → 1.10) makes it non-comparable for promotion |

Tooling built: `auto_coverage_drive.py` (survey gate, corridor checks,
maneuver rotation, auto-relocation, session JSON), `drive_scorecard.py`
(Stage A one-command scorecard), `steering_map_challenger.py` (Stage B fit +
learned-memory export), `steering_dynamics_challenger.py` (Stage C fit),
plus controller `learned_steering_mode` off/shadow/blend with trust region,
learning freeze while applied, and automatic rollback.

## 2. What we discovered (ranked)

1. **Ordinary driving is sufficient to learn the steering map.** The Stage B
   challenger, seeded only from wheelbase geometry, reproduced the
   scripted-calibration dense map to within servo hysteresis and beat it on
   held-out coverage bags. The universal-learner premise is now evidence, not
   hypothesis. Critically, this worked *because of* the §5.3 gates
   (settled-window alignment, contamination rejection, held-out promotion) —
   a naive fit was already known to produce wrong-signed slopes.
2. **The learned-envelope loop works end-to-end on the real robot.** Session
   1 evidence contracted the trackable clamp 1.148 → 0.92 1/m; sessions 3–5
   ran at 0% saturation with steering error down 38% and cusp maneuvers 3–4×
   faster. First learned quantity to improve the physical robot A/B.
3. **Steering is a gradual state, not a delay** — confirmed twice: the live
   delay estimator's confidence collapsed (1.00 → 0.33 across the day, the
   fixed-delay model failing to explain evidence), and the offline
   first-order model beat fixed delay on every held-out split.
4. **Safety invariants held under stress.** Session 2: obstacle-contaminated
   failures did NOT contract the learner (§13 invariant verified live);
   collision monitor + scan guard stopped the robot at 0.11 m; the 3-strike
   abort ended the session. Session 5: zero false rollbacks under battery
   sag.
5. **`effort_scale` is a working battery gauge** (power HAT is dead, so this
   matters): it climbed 1.00 → 1.10 as the pack died, with stalls 15%→23%.
   Operating rule: treat >1.08 as end-of-session.
6. **Gap found: short-term state persists across power cycles.** The sagged
   effort_scale (1.099) and inflated floor observations are saved in runtime
   memory; §12 requires post-power-off reset of battery-sensitive state and
   it is not implemented. Next fresh-pack session would start ~10% hot.
7. **Goal-selection strategy, not the controller, caused most failures.**
   2 of 5 sessions degraded from room congestion; the fix (relocate to the
   most open sector when the forward wedge < 2.2 m) worked in sessions 4–5.

## 3. Against the literature (masterplan §4)

- *AutoRally: nonlinearity belongs in feedforward; feedback only trims.*
  Confirmed — the learned monotone map IS the feedforward, and it tracked
  slightly better than champion regions with the RLS trim frozen.
- *RLS/MRAC warned against on sparse bursty data.* Vindicated: the RLS
  branches never earned applied confidence all day, while binned-median +
  isotonic fit (closer to ArduPilot's event-based philosophy) passed
  held-out on the first attempt.
- *ArduPilot event-triggered learning precedent.* Our breakaway, floor,
  trackability, and now steering-map learners are all event/evidence-based —
  the shipped-precedent pattern continues to work.
- *Quantization limit-cycle theorem; feedforward must carry.* Still visible:
  launch surges >0.40 m/s and ~17% stall at cusp approaches are throttle-side
  and unchanged — the throttle map spans only 0.20–0.25 m/s and normal
  driving cannot mine it (2026-07-11 result). Stage F needs probe excitation,
  as §8 already concluded.
- *Nav2 maintainer guidance (cusp dwell, replan discipline).* The dispatcher
  honors it; clean cusp endpoints reached 0.022–0.14 m today.

## 4. Distance to the goal

Authority audit — who steers/throttles the robot today:

| Quantity | Authority today | Amnesia-ready? |
|---|---|---|
| Steering map, |κ|≤0.6 | LEARNED (blend, rollback armed) | largely — relearnable from ordinary drives |
| Steering map, high κ | dense measured map | no — learned map has data there but unpromoted |
| Steering dynamics/preview | measured constants | no — Stage C model exists offline only |
| Trackable envelope | LEARNED | yes |
| Planner radius bootstrap | fixed 1.30 m, launch-time only | no — needs geometry scaling + between-goal updates (Stage E) |
| Breakaway | LEARNED (causal, applied) | yes |
| Sustain floor | observed; configured floor authoritative | partly |
| Throttle steady map | measured anchors | no — provably not minable from normal driving; needs probes (Stage F) |
| Polarity | doc says learned; discovery NOT coded | no |
| Blank-state bootstrap path | schema designed, no code | no |

Stage scorecard: A ✅ · B ✅ offline + blend live (promotion 0/3 sessions,
battery invalidated the first) · C ✅ offline, not integrated · D ~40% ·
E ~50% (envelope yes; bootstrap scaling + between-goal updates missing) ·
F ~20% (breakaway/floor yes; steady map + probes missing) · G 0%.

**Overall: roughly a third of the way to the amnesia exam.** Steering is the
most advanced (~60%); throttle is the long pole (~20%) because it needs the
probe machinery that nothing else has exercised yet; the blank-bootstrap
code path (birth-certificate loader, polarity discovery, cautious priors)
exists only on paper.

Estimated remaining at today's pace: Stage D 2–3 fresh-pack sessions +
disarmed rollback test; Stage C integration ~2 sessions; Stage E code + 1–2
sessions; Stage F probe experiments 3–5 sessions (highest technical risk);
bootstrap code + amnesia dress rehearsal 2–3 sessions. ≈ 12–18 sessions
across 2–4 more days like today, assuming Stage F cooperates.

## 5. Before the next run

1. Charge/replace the pack; verify `effort_scale` converges back near 1.0
   early, or reset it manually (and implement §12 post-power-off decay).
2. Clear the congested pocket (105°/−165° region of the room).
3. Deliberate rollback test — disarmed, no battery needed: force
   `learned_steering_rollback_error_1pm` low and verify fallback + logging.
4. Then two clean blend sessions to close Stage D.
