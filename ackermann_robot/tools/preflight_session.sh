#!/bin/bash
# One-command session pre-flight: fixes every environment failure class that
# cost sessions on 2026-07-12/13 (zombie nodes, stale FastDDS SHM locks,
# wedged I2C, double stacks), then launches the armed stack and waits for
# health. Prints READY on success; any failure prints a specific reason.
# (no `set -u`: ROS setup.bash references unbound variables)
LOG="${1:-/tmp/bringup_$(date +%H%M%S).log}"

echo "[preflight] stopping any existing stack..."
pgrep -af "component_container|controller_server|planner_server|behavior_server|smoother_server|bt_navigator|lifecycle_manager|collision_monitor|velocity_smoother|path_segment_dispatcher|adaptive_ackermann|pca9685_effort_driver|tf_odom_bridge|robot_state_publisher|mola|sllidar|ros2 bag|rplidar" \
  | grep -v grep | awk '{print $1}' | xargs -r kill 2>/dev/null
sleep 4
pgrep -af "mola|sllidar|bt_navigator|adaptive|dispatcher|planner_server|pca9685|tf_odom" \
  | grep -v grep | awk '{print $1}' | xargs -r kill -9 2>/dev/null
sleep 1
if pgrep -af "mola|bt_navigator|planner_server" | grep -v grep; then
  echo "FAIL: processes refuse to die"; exit 1
fi

echo "[preflight] clearing FastDDS shared memory + daemon..."
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
source /opt/ros/jazzy/setup.bash
source /home/luky/ros2_ws/install/setup.bash
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1

echo "[preflight] checking I2C (expect PCA9685 at 0x40)..."
if ! timeout 8 i2cget -y 1 0x40 0x00 >/dev/null 2>&1; then
  echo "FAIL: PCA9685 not answering at 0x40 (power off, or bus clamped -> cold power-off needed)"
  exit 1
fi

echo "[preflight] launching armed stack (log: $LOG)..."
nohup ros2 launch ackermann_robot bringup.launch.py arm_hardware:=true \
  > "$LOG" 2>&1 &
LAUNCH_PID=$!

echo "[preflight] waiting for health (odom + scan + nav server)..."
DEADLINE=$((SECONDS + 150))
while [ $SECONDS -lt $DEADLINE ]; do
  if timeout 4 ros2 topic echo /odom --once >/dev/null 2>&1 \
     && timeout 4 ros2 topic echo /scan --once >/dev/null 2>&1 \
     && ros2 action list 2>/dev/null | grep -q navigate_to_pose; then
    echo "[preflight] checking for lidar self-returns (cable/debris on robot)..."
    SELFRET=$(timeout 15 python3 - <<'PYEOF'
import math, rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
rclpy.init()
node = rclpy.create_node('selfret_check')
scans = []
qos = QoSProfile(depth=5); qos.reliability = ReliabilityPolicy.BEST_EFFORT
node.create_subscription(LaserScan, '/scan', lambda m: scans.append(m), qos)
while len(scans) < 3:
    rclpy.spin_once(node, timeout_sec=1)
s = scans[-1]; a = s.angle_min; near = 99.0
for r in s.ranges:
    ang = a; a += s.angle_increment
    if not math.isfinite(r) or r <= s.range_min: continue
    x, y = 0.237 - r*math.cos(ang), -r*math.sin(ang)
    near = min(near, math.hypot(x, y))
print(f'{near:.2f}')
PYEOF
)
    if python3 -c "exit(0 if float('$SELFRET' or 99) < 0.45 else 1)" 2>/dev/null; then
      echo "WARNING: return at ${SELFRET} m from robot center — check for a cable,"
      echo "         debris, or rug edge ON or against the robot near the lidar."
      echo "         (This exact failure cost 3 sessions on 2026-07-13.)"
    fi
    echo "READY (launch pid $LAUNCH_PID)"
    exit 0
  fi
  sleep 3
done
echo "FAIL: stack did not become healthy in 150 s — see $LOG"
grep -iE "error|died|Failed" "$LOG" | tail -5
exit 1
