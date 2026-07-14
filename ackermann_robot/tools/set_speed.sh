#!/bin/bash
# Live cruise-speed change, no restart. Usage: set_speed.sh 0.45
# Sets the adaptive controller's dynamic speed envelope AND RPP's desired
# velocity (both are runtime-reconfigurable). Hard clamp gets +0.05
# headroom; the overspeed tripwire keeps >=0.25 of launch margin.
V="${1:?usage: set_speed.sh <cruise m/s>}"
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
CLAMP=$(python3 -c "print(round($V + 0.05, 3))")
TRIP=$(python3 -c "print(round(max(0.65, $V + 0.25), 3))")
set -e
ros2 param set /adaptive_ackermann_controller maximum_measured_speed_mps $TRIP
ros2 param set /adaptive_ackermann_controller maximum_forward_speed_mps $CLAMP
ros2 param set /adaptive_ackermann_controller navigation_speed_limit_mps $V
ros2 param set /controller_server FollowPath.desired_linear_vel $V
echo "cruise -> $V m/s (clamp $CLAMP, tripwire $TRIP) — live, no restart"
