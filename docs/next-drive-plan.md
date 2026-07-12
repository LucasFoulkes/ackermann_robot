# Next supervised drive — RPP with chronological cusp dispatch

Updated 2026-07-12 after the successful RPP/dispatcher validation and the
trackability-learning implementation. This file supersedes the earlier MPPI,
curvature-floor, fixed-radius, and cusp-gate drive plans.

## Why the stack was rolled back

The committed-path RPP run at 02:28 achieved 0.018 m rolling XTE p90 and 0.093
rad heading-error p90. Its defect was path pruning across Reeds-Shepp cusps,
not ordinary segment tracking.

Jazzy MPPI was promoted despite the project's own bench-only warning. It lacks
this vehicle's actuator-delay model, under-commanded tight reverse paths, and
assumes its Twist is applied unchanged. Downstream curvature floors, speed
quantization, and an independent cusp observer then made MPPI, Collision
Monitor, and the hardware operate on different commands. The 11:33 run reached
0.083 m XTE p90, blocked 93.9 s in the cusp latch, and repeatedly failed
progress. One goal generated 24 incompatible replans while the robot traveled
6.93 m away from the intended maneuver.

## Current architecture

```text
Smac Hybrid (one path per new goal, Reeds-Shepp, learned trackable radius)
  -> path_segment_dispatcher (split at every direction cusp; score each run)
  -> RPP backend (exactly one forward or reverse segment)
  -> executable command boundary (speed floor, curvature clamp, servo-transition slowdown)
  -> Collision Monitor
  -> actuator bridge (no nonzero speed/curvature changes)
  -> measured direction-specific steering and throttle maps
```

The dispatcher uses a dedicated 0.07 m / 0.15 rad cusp goal checker and waits
0.60 s after each actual direction change. RPP never receives overlapping
geometry beyond a cusp, so Jazzy's closest-pose pruning cannot skip into a
later maneuver. Non-final fragments shorter than 0.18 m are explicitly skipped
instead of being accepted accidentally by the broad final-goal tolerance.

The physical curvature envelope remains learned conservatively, currently
about 1.146 1/m (0.872 m steady radius). The planning envelope is now learned
from actual path tracking, not derived from steady full-lock steering and not
permanently clamped at 1.30 m. The 1.30 m value is only the bootstrap prior.
Forward/reverse and positive/negative curvature are scored separately. Only a
clean entry, unblocked segment of at least 0.50 m can update the model. Three
clean near-boundary passes earn a small curvature exploration step; two
comparable failures contract faster. The weakest of the four branches sets the
radius on the next launch, within 80% of the learned physical envelope. State
persists in `~/.robot/planner_trackability.yaml`; per-segment evidence is also
recorded on `/planner_trackability` in the bag.

The path is committed only while it remains executable. The dispatcher now
measures chronological distance along the active one-direction polyline. If it
makes no new 0.05 m progress for 6.0 s, or RPP requests the opposite direction
for 0.75 s inside that segment, the dispatcher cancels it with
`FAILED_TO_MAKE_PROGRESS`. The BT does not retry those old segments: it clears
both costmaps and computes one fresh path from the live pose. This is
event-triggered replanning, not periodic replanning. A moving obstacle gets a
2.0 s wait before BackUp is considered.

The first rollback-validation run was materially successful: 14 goals
succeeded, no controller/progress failures occurred, and 42 chronological
segments executed (21 forward, 21 reverse). Rolling path error was 0.016 m
median / 0.120 m p90 overall. The remaining errors clustered at cusp entry and
on saturated branch-specific turns, which is why those conditions are now
measured instead of treating every failure as proof of a steering-radius limit.

Minimum sustainable speed and steering-transition slowdown are applied before
Collision Monitor. After the monitor, the actuator node may only stop for a
watchdog, stale sensor, or closer local obstacle; it does not raise speed or
change curvature. Steering identification probe taps are disabled for this
rollback verification.

## Drive procedure

1. Before every experiment, checkpoint all code changes in the package Git
   repository. Record the resulting short SHA with the run notes so a bag/CSV
   can always be tied to the exact executable source:

   ```bash
   cd ~/ros2_ws/src/ackermann_robot
   git status --short
   git add -A
   git commit -m "experiment: describe the next run"
   git rev-parse --short HEAD
   ```

   If the tree is already clean, record the existing SHA; do not create an
   empty commit.
2. Put the robot in a clear area and stay beside the power switch.
3. Start:

   ```bash
   cd ~/ros2_ws
   source install/setup.bash
   ros2 launch ackermann_robot bringup.launch.py arm_hardware:=true
   ```

4. Issue one simple forward arc goal.
5. Issue one simple reverse arc goal.
6. Issue at least one clean turn in each of the four direction/turn branches.
7. Issue one goal requiring a single forward/reverse cusp.
8. Only if those behave normally, issue the prior three-point goal.
9. Stop cleanly with Ctrl-C after the goal finishes or is canceled.

Expect a visible neutral dwell at each cusp. Do not expect rapid alternating
forward/reverse commands. Every controller launch opens a new timestamped CSV
in write mode; older sessions remain separate and are never appended.

## Acceptance criteria

| Metric | Required |
|---|---:|
| RPP rolling XTE p90 | <= 0.035 m |
| rolling heading error p90 | <= 0.15 rad |
| backend segment sequence | exactly the path's chronological direction runs |
| plan replacements during an unchanged goal | 0 |
| post-monitor nonzero speed mutation | 0 rows |
| post-monitor curvature mutation | 0 rows |
| direction-settle time | < 5 s total on a one-cusp test |
| obstacle/watchdog faults during clear-space motion | 0 |
| eligible trackability evidence | clean-entry segments only |
| cusp endpoint | <= 0.07 m and <= 0.15 rad before reversal |
| stale FollowPath retries after segment failure | 0 |
| opposite-direction command inside a segment | abort/replan within 0.75 s |
| no chronological segment progress | abort/replan within 6.0 s |

Do not expect the radius to jump during a drive: evidence is persisted as it is
collected, and the weakest learned branch is applied at the next safe launch.
