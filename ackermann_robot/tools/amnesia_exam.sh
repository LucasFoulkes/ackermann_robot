#!/bin/bash
# The amnesia exam (universal-adaptive-ackermann.md §1): wipe ALL learned
# memory AND demote every hand-measured config value to a generic guess,
# then boot from the birth certificate alone. Usage:
#   amnesia_exam.sh begin    -> backup + wipe + generic overrides + bootstrap
#   amnesia_exam.sh restore  -> undo config overrides (learned memory stays
#                               whatever the robot earned; backups retained)
MODE="${1:-begin}"
STAMP=$(date +%Y%m%d_%H%M%S)
CONFIG=/home/luky/ros2_ws/src/ackermann_robot/ackermann_robot/config/adaptive_controller.yaml
ROBOT=/home/luky/.robot

if [ "$MODE" = "begin" ]; then
  echo "[exam] backing up learned memory -> ~/.robot_exam_backup_$STAMP"
  mkdir -p ~/.robot_exam_backup_$STAMP
  cp -a $ROBOT/*.yaml ~/.robot_exam_backup_$STAMP/ 2>/dev/null
  rm -f $ROBOT/adaptive_ackermann_runtime.yaml \
        $ROBOT/planner_trackability.yaml \
        $ROBOT/learned_steering_map.yaml \
        $ROBOT/learned_steering_dynamics.yaml
  echo "[exam] learned memory wiped"

  echo "[exam] demoting hand-measured config values to generic guesses..."
  cp "$CONFIG" "$CONFIG.pre_exam"
  python3 - "$CONFIG" <<'PYEOF'
import re, sys
path = sys.argv[1]
s = open(path).read()
# All values are normalized efforts (2026-07-13 hardware split): forward
# drive is negative, +/-1 = declared safe extremes.
# Throttle maps: generic hobby-ESC guesses (deliberately imprecise), NOT the
# measured anchors. The learned line + ladder + trim must close the gap.
s = re.sub(r"forward_throttle_map: \[[^\]]*\]",
           "forward_throttle_map: [.2, -0.19, .3, -0.25]", s)
s = re.sub(r"reverse_throttle_map: \[[^\]]*\]",
           "reverse_throttle_map: [.2, 0.19, .3, 0.25]", s)
# Breakaway priors: generic gentle values far from the measured -0.30/+0.35.
s = re.sub(r"forward_breakaway_prior_effort: -?[0-9.]+",
           "forward_breakaway_prior_effort: -0.16", s)
s = re.sub(r"reverse_breakaway_prior_effort: -?[0-9.]+",
           "reverse_breakaway_prior_effort: 0.16", s)
# Steering fallback maps: geometry-generic (declared neutral = 0 effort,
# generic span) so a rollback never hands authority to measured knots.
s = re.sub(r"forward_steering_map: \[[^\]]*\]",
           "forward_steering_map: [-1.15, -0.76, 0.0, 0.0, 1.15, 0.65]", s)
s = re.sub(r"reverse_steering_map: \[[^\]]*\]",
           "reverse_steering_map: [-1.15, -0.76, 0.0, 0.0, 1.15, 0.65]", s)
# Steering lag constants: generic single guess instead of measured values.
s = re.sub(r"steering_lag_time_forward_s: [0-9.]+",
           "steering_lag_time_forward_s: 0.25", s)
s = re.sub(r"steering_lag_time_reverse_s: [0-9.]+",
           "steering_lag_time_reverse_s: 0.25", s)
s = re.sub(r"steering_lag_distance_reverse_m: [0-9.]+",
           "steering_lag_distance_reverse_m: 0.0", s)
open(path, 'w').write(s)
print("config overrides applied")
PYEOF

  echo "[exam] generating prior-only bootstrap steering map..."
  python3 /home/luky/ros2_ws/src/ackermann_robot/adaptive_ackermann/adaptive_ackermann/tools/steering_map_challenger.py --bootstrap
  echo "[exam] READY. Boot with tools/preflight_session.sh, run ordinary"
  echo "       sessions, refit steering between sessions with:"
  echo "       adaptive_ackermann/tools/steering_map_challenger.py --since $STAMP"
  echo "$STAMP" > /tmp/amnesia_exam_stamp
elif [ "$MODE" = "restore" ]; then
  if [ -f "$CONFIG.pre_exam" ]; then
    mv "$CONFIG.pre_exam" "$CONFIG"
    echo "[exam] config restored (learned memory left as the robot earned it)"
  else
    echo "no pre-exam config backup found"
  fi
else
  echo "usage: amnesia_exam.sh begin|restore"; exit 1
fi
