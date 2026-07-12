# Experiment archive

The executable ROS 2 experiment modules live in
`ackermann_robot/experiments/`. The `data/` directory contains the CSV files
retained from the July 2026 throttle experiments.

The programs intentionally continue to write new CSV files into the directory
from which they are run. This keeps physical test output out of the installed
Python package. After a run has been reviewed, move the CSV here with a new,
descriptive name rather than overwriting the retained evidence.

See `docs/throttle-experiments-and-steering-plan.md` for the complete method,
findings, safety boundaries, and steering-identification plan.
