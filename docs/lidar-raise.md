# Raising the lidar: procedure and rationale

**Goal:** move the 2D lidar from 16.5 cm to **~37–40 cm** above ground.

**Why:** at 16.5 cm the lidar sees ankles. DR-SPAAM's training data
(DROW dataset) was captured at **37 cm** — calf/knee height — which is
why our referee scores run weak (human 0.4–0.6, threshold-hugging)
and why one chair leg ever reached 0.39. At the training height the
detector separates humans from furniture by a wide margin, legs
present larger/cleaner clusters to the geometric detector, and ankle
occlusion by low clutter disappears. This lifts every layer at once:
detection range, referee confidence, identity continuity during
drive legs.

**Pick 37–40 cm, not 60:** every centimeter above the robot's own
pass-under height is a blind band for the safety chain (see tradeoff
below), and 37 cm is what the neural referee was trained at.

## Mounting requirements

- Rigid (no wobble at full steering slew — a flexing mast smears
  scans worse than the height helps).
- Level within ~1° (a tilted plane reads the floor as a wall arc on
  one side at 3–4 m).
- Same yaw orientation as today (connector pointing the same way);
  if it rotates, measure the new yaw.
- 360° clear of the mast/cables: any strut inside the scan plane
  becomes a permanent phantom obstacle at fixed bearing.

## After mounting — update these

1. Measure the new height (floor → scan plane center), and check
   whether x/y shifted (mast offset from the old mount point).
2. `urdf/ackermann_robot.urdf.xacro`: `lidar_z` 0.165 → measured.
3. `config/birth_certificate.yaml` lidar section: `x_m` / `y_m` /
   `yaw_rad` ONLY if the mount moved/rotated them (planar stack uses
   these; height is TF-only).
4. Rebuild `ackermann_robot`, relaunch.

## Verification checklist

1. RViz: walls in the scan align with the map; no fixed-bearing
   phantom returns (mast/cable in view).
2. Drive a slow figure-8: odometry stays sane (MOLA consumes the same
   scans — new plane should be as feature-rich or better).
3. Stand 2–4 m away, walk: leg clusters present at knee height
   (widths will read larger; gates 0.03–0.25 m single / 0.45 merged
   have headroom — retune only if confirmed necessary from a bag).
4. Record a follow session bag, run
   `tools/drspaam_bench.py <bag> <ckpt>`: person scores should jump
   from 0.4–0.6 to >0.7 with ghosts unchanged near zero. If so,
   consider raising the referee floor `CONF_MIN` 0.30 → 0.45 and the
   fast-confirm bar 0.45 → 0.60 (tracker) for an even harder
   furniture wall.

## The tradeoff to keep in mind

A 2D plane at 40 cm cannot see anything **below 40 cm** — boxes, stool
footrests, sofa rails. Anything the robot cannot pass under but
shorter than the plane is invisible to the obstacle gates and the
costmap. Keep the room's low clutter in mind during tests, and the
designed long-term fix is fusing the D435i depth camera into the
costmap as a second obstacle source (it already runs for its IMU).
