# How the learner actually works

Every number below is from the code as of 2026-07-15 (post-ODAAC-steals).
This is the mechanical answer to "what is happening when it learns."

## The only inputs

The learner sees exactly two streams: the effort commands it sent
(a ring buffer with timestamps), and MOLA lidar odometry (~10 Hz pose,
from which it derives speed and yaw rate through a 3-sample median).
There is no encoder, no IMU term, no battery voltage. Every learned
quantity is some form of "I commanded X at time t, and the world
measured Y at time t+delay."

Causal attribution is the foundation: an odometry sample at time t is
explained by the command at t − lag (measured per direction: 0.235 s
forward, 0.168 s reverse). Learning without this alignment learns noise.

## Eight learners, slowest to fastest

**1. Breakaway effort** (per direction) — "how hard to push to start
moving." When a launch confirms motion, the effort that was *causally
held when raw motion first appeared* becomes one observation. Update:
EMA with forgetting 0.5 for the first 3 launches of a session (fast
re-acquisition after battery/surface change), then ~0.9 (stability).
NEW: the innovation's EW variance damps the rate — scattered evidence
slows learning down to 1/4 speed instead of averaging garbage
(`breakaway_variance_reference: 0.01`). Rejections counted:
`breakaway_no_causal_command`, `breakaway_implausible_effort`.

**2. Cruise anchor** (per direction) — "what effort holds what speed."
Only updates in settled cruising: rolling state, measured/commanded
ratio in (0.75, 1.25), no probe active. Then effort, speed, and spread
each move by 1% per tick toward the current value. This is the second
point of the feedforward line; breakaway is the first.

**3. Throttle trims** (per direction) — slow integral on speed error,
rate-limited (`trim_rate`). The bias absorber for everything the map
doesn't know.

**4. effort_scale** — the fast battery/payload compensator. A 1-D
Kalman filter on the ratio (current effort)/(map effort at this speed):
process noise grows with wall time, so it stays alert; measurement
noise keeps single ticks from yanking it. Clamped [0.7, 1.3]. This is
what tracked the pack dying (→latch) and recovering (0.97 today)
without any gauge. Rejection counted: `effort_scale_weak_map_signal`
(map effort too small to divide by).

**5. Floor observer** (per direction) — "below what speed do I stall."
Watches sustained-crawl streaks: 4+ consecutive slow ticks that survive
→ `held_ema` (10% EMA of the streak minimum); a streak that dies →
`died_ema`. The stall floor estimate is died+0.02 or held−0.01,
clamped [0.08, 0.18]. Feeds minimum sustainable speed everywhere.

**6. Steering RLS** (4 branches: forward/reverse × left/right) — "when
I ask the map for curvature κ, what do I actually get." Every 0.25 s
while rolling: integrate the last ~0.5 s of motion into measured
κ = Δyaw/Δdistance; look up what the champion map *predicted* for the
causally-aligned commands (tried at 13 delay candidates — the winner
updates the delay estimator); then a 2-parameter recursive least
squares (gain, bias) per branch with forgetting, innovation clamped
±0.6, gain clamped [0.4, 1.6], bias ±0.35. `residual_ema` (10% EMA of
|error|) is each branch's self-reported trustworthiness — it feeds the
launch-time conservative curvature limit. Rejections counted:
`identification_direction_mixed`, `identification_kappa_outlier`.

**7. Learned steering inverse map** — built OFFLINE by the challenger
tool from ≥25-sample bins across session CSVs, loaded at boot, applied
only inside its data-backed region (±0.6 today). NEW: isotonic
projection at load — a non-monotonic (self-crossing) map steers the
wrong way inside the crossing, so it is projected to the nearest
monotonic map and warned about, never trusted raw. Guarded live by the
**rollback monitor**: applied-error EMA accumulates only in steady
rolling (>1.5 s, the attribution fix) and benches the learned map above
0.85 — the champion map + RLS keep steering when it's benched.

**8. Dispatcher trackability** (4 branches) — "what path curvature can
I actually TRACK," which becomes the planner's turning radius. Each
completed/aborted segment with clean entry (≤6 cm, ≤0.15 rad) is a
pass/fail observation at its demand curvature. Three consecutive passes
near the boundary → estimate grows 20% of the way toward demand+0.10
(slow exploration). Two failures → contracts by half the gap toward
0.9×demand (fast retreat). NEW: **session consolidation** — at each
boot, any branch whose evidence all comes from ONE session decays
halfway back toward the prior; only estimates reproduced across ≥2
sessions persist. (This is the gate that would have stopped the
2026-07-14 dead-pack afternoon from teaching "radius 2.5 m" at
confidence 1.0.)

## What is deliberately NOT learned

Hard safety ceilings (birth certificate), the champion steering map's
existence (only its correction), comfort limits (cornering speed),
and anything from a battery gauge. Polarity (which effort sign means
forward / left) IS learned — discovered by voting episodes, never
declared.

## The gates (why samples get thrown away)

A sample must survive: fresh odometry, not an outlier (median-filter
deviation or yaw-rate beyond the per-side envelope), steady rolling
(>1.5 s) for error EMAs, single direction across the identification
window, |measured κ| ≤ 2.5, sufficient distance, causal command
available, plausible effort range. Every rejection increments a
per-reason counter published at 1 Hz in `/controller/debug`
(`learning_rejections`) — a spike in one reason is a fault signature.

## Persistence and defense layers

Runtime memory (`~/.robot/adaptive_ackermann_runtime.yaml`, v8) saves
every 10 s: breakaway (with variance), anchors, trims, effort_scale,
RLS models, delay estimator, polarities, floor observer. Fingerprinted
to the vehicle. Defense in depth, outermost first: birth-certificate
hard clamps (last line before publish) → per-side executable κ caps →
launch-time conservative shrink from RLS residuals → rollback benching
→ session consolidation → isotonic projection → variance damping →
sample gates → rejection accounting.
