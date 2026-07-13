# External-source research digest — 2026-07-12

Deep-research pass over 20 user-supplied sources (RC platforms, Nav2 issues,
farm robotics, sensors, follow-me). 105 agents, adversarial 3-vote
verification per claim; only verified findings below. Full trace:
`~/.claude/.../tasks/w84331yn5.output`. Companion to `masterplan.md` §4 —
where verdicts differ, this digest is newer and verified.

## 1. Nav2 Ackermann reverse/cusp — verified release status

- **#4757 (RPP cusp-skipping) is algorithmic and UNFIXABLE on Jazzy.** RPP's
  path pruning is a direction-blind closest-point search; adjacent
  Reeds-Shepp cusps get skipped structurally. The official fix (PR #5446,
  merged main 2025-12-29) is a ~100-file rework moving inversion handling
  into a controller-server `FeasiblePathHandler` plugin — zero backports,
  ships post-Kilted (L-turtle). Jazzy RPP has NO inversion parameters at
  all. **Verdict: our chronological cusp dispatcher is the correct
  mitigation class and must stay until an L-turtle migration.**
- **REFUTED (0–3): "the workaround people ship is switching to MPPI."**
  MPPI-as-proven-Ackermann-reverse-alternative did not survive verification:
  #4425 closed as a frame-configuration question with no code fix and still
  recurs (2025-08-28); #6107 (rollouts violating min_turning_r, real-world
  near-misses) closed 2026-07-06 UNFIXED. Any RPP→MPPI move must be gated on
  our own A/B, not documentation or folk wisdom.
- **If we do trial MPPI on Jazzy 1.3.12, the field is clean:** all known
  Ackermann-reverse code fixes are verified ancestors of the 1.3.12 tag
  (#4664 wz-sign, #4954 bidirectional accel, #4510 min_turning_r). The
  min_turning_r fix note matters for reading old reports: MPPI silently used
  0.2 m regardless of config until 2024-08 — discount all pre-Aug-2024
  MPPI-Ackermann anecdotes. Shipped knobs for cusp discipline:
  `enforce_path_inversion`, PathAlignCritic `use_path_orientations: true`,
  PathAngleCritic `mode: 2`, PreferForwardCritic weight (reporter-confirmed
  =0 stopped random reverse steering on a Humble golf car).
- **Maintainer guidance (from #4425), all already satisfied by our stack:**
  rear-axle `base_link` (we comply); stationary steering transition at every
  cusp — Nav2 will NOT insert the dwell (our dispatcher dwells; keep ≥
  wind-in 0.2–0.4 s + ESC spin-down); `cmd_vel` is body-velocity,
  "BYO-vehicle" — explicitly legitimizes our custom adaptive controller.
- **BT: replan-only-if-invalid** (`navigate_w_recovery_and_replanning_only_
  if_path_becomes_invalid.xml` ships in 1.3.12) is reporter-confirmed to stop
  Reeds-Shepp direction flip-flop. **Ours already goes further** — exactly
  one plan per goal + dispatcher-owned per-segment validity — no change.

## 2. Actuator delay (Stage C validation)

MPPI PR #6154 (`model_delay_vx/wz`, main/L-turtle only, ABI-breaking, docs
example: 600 ms steering delay) is a shipped implementation of exactly our
servo-lag problem. Its design — **replay in-flight commands into the model
rather than adding servo lead** — independently validates our fixed-lag /
preview architecture and the Stage C gradual-state direction. Docs typo:
`model_delay_wx` in docs is `model_delay_wz` in code.

## 3. Hobby-ESC behavior (Stages D/F, launch problem)

- The reverse **double-tap (rev-neutral-rev) is factory-standard** hobby-ESC
  firmware (DonkeyCar docs + HobbyWing QuicRun manuals), with reverse
  engaging only after motor spin-down. Implication: the drop-through
  sequencer needs a neutral DWELL sized to spin-down, coupling ESC timing
  with cusp timing.
- **Many ESCs reprogram to single-click "crawler" Fwd/Rev mode** (jumper or
  program card) — immediate reverse, no brake stage. Worth checking on our
  QuicRun before further hardening the software sequencer: could shorten
  cusp dwells AND soften launch dynamics. Caveat: changes breakaway
  behavior → learned breakaway/floor estimators must relearn (they can —
  that's the design).
- No external precedent survived for pulse-level launch shaping without
  feedback — our breakaway-approach launch remains ahead of practice.

## 4. Platform portability survey (Stage G context)

Across DonkeyCar, AutoRally, MuSHR: the "birth certificate" holds declared
identity + hand-entered pulse endpoints, and **nobody learns actuation maps
from ordinary driving**:
- DonkeyCar: five PWM constants total; everything else delegated to
  behavioral cloning. Our birth certificate is already a strict superset.
- AutoRally: ~20 infrastructure steps, hand-entered per-chassis pulse yaml,
  serial-number registry; zero learning.
- MuSHR: only the **tape-measure certification** survived verification —
  drive 7–8 ft along a tape, compare odometry to physical truth, explicit
  numeric threshold (0.05–0.076 m), iterate until pass. **ADOPT for Stage G:
  scripted physical-ground-truth checks with numeric tolerances.** (MuSHR
  parameter-level claims about their speed/steering maps were refuted —
  don't cite them.)

Conclusion: Stages D–G exceed established platform practice; we are not
reimplementing anyone's wheel.

## 5. Follow-me milestone (deferred)

L-turtle packages a **Following Server** (`opennav_following`, PR #5565;
PoseStamped-topic or TF-frame detection input, FollowObject BT action) — not
bloom-released in any distro as of 2026-07-12. On Jazzy: the tutorial-era BT
pattern (periodic replan to detection-driven goal + truncated path).
**Design contract now:** whatever leg/person detector we adopt should emit
PoseStamped/TF in that shape so the eventual server drops in.

## 6. Coverage gaps — needs a second research pass

12 of 20 sources produced zero verified claims (pipeline outcome, not
worthlessness): CygLiDAR D2, OAK/Luxonis, AgriGS-SLAM, feldfreund,
upo_laser_people_detector, person_following_robot, RACECAR, F1TENTH,
DIY Robocars, ros2_control thread, Duckietown, Isaac sample. Open problems
still unanswered by verified evidence:
- **Probe excitation for throttle ID (Stage F)** — next pass should target
  system-identification literature (persistent excitation under closed loop,
  dual control, active learning), not hobby-platform docs.
- **Farm phase** (crop rows, RTK, coverage) — opennav_coverage and the Nav2
  GPS tutorial surfaced as leads.
- **Vertical-gap sensor** (low obstacles + person detection): CygLiDAR D2 vs
  OAK vs reviving the D435i — needs driver-maturity-focused pass.
- **ros2_control Ackermann** (post-encoder future).

## 7. Action shortlist

| Action | When | Feeds |
|---|---|---|
| Keep dispatcher as cusp authority; do NOT wait for Jazzy fix | standing | E |
| Tape-measure certification w/ numeric thresholds | build into Stage G suite | G |
| Check ESC crawler-mode reprogram (program card) | next hardware session | D/F, launches |
| Confirm cusp dwell ≥ wind-in + ESC spin-down | config audit, now | 3/5 |
| MPPI A/B trial (all fixes in 1.3.12; soft-cost knobs) | optional experiment, after Stage D | E |
| L-turtle source overlay (FeasiblePathHandler + model_delay) | at M5 migration | E |
| Leg detector emits PoseStamped/TF per Following Server contract | follow-me milestone | follow-me |
| Second research pass: probes/farm/sensors/ros2_control | before Stage F | F, farm |
