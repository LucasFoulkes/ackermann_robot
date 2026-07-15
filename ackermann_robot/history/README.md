# History — archived experiments and measurement data

Nothing in this folder is installed or imported by the running system. It is
the paper trail of how the robot's knowledge was earned (May–July 2026),
kept for provenance and for re-analysis of old data.

## experiments/

The scripted identification era, superseded on 2026-07-13 by the hardware
split + effort-native migration. Everything here is **pulse-microsecond
denominated** and most scripts drive the PCA9685 directly over I2C — they
predate the `pca9685_effort_driver` and must never run while the driver owns
the bus (the esc-watchdog treats them as legitimate owners).

- `esc_*` / `steering_*_id` / `steering_limit_search` / `steering_sanity` —
  bench identification scripts (breakaway, sustain floor, steering steps,
  staircases, limits). Their measurements seeded the config maps that are now
  bootstrap priors; the robot re-learns all of it from ordinary driving.
- `steering_map_challenger.py`, `drive_scorecard.py` — µs-era copies, still
  useful to analyze pre-2026-07-13 CSVs (effort-era copies live in
  `adaptive_ackermann/tools/`).
- `steering_dynamics_challenger.py` — fits learned_steering_dynamics.yaml
  from drive CSVs; µs-era, needs an effort-domain port before next use.
- `analyze_*`, `lockin_bench.py`, `static_odom_noise.py` — offline analyzers.

Cross-imports were rewritten to plain module imports, so a script can still
be run from inside this directory if ever needed.

## data/

Raw measurement CSVs from the identification era (the evidence behind the
map values quoted in docs/ and the config history). See
`docs/throttle-experiments-and-steering-plan.md` for method, findings, and
safety boundaries.

Live operational tools moved to `tools/` (`auto_coverage_drive.py`,
`preflight_session.sh`, `amnesia_exam.sh`, `convert_memory_to_effort.py`).

Note (2026-07-15): files here are frozen archive copies. Live tools moved
to `adaptive_ackermann/adaptive_ackermann/tools/` (drive_scorecard,
steering_map_challenger) — same names there are the maintained versions.
