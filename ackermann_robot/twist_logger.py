#!/usr/bin/env python3
"""Log desired twist vs actual twist vs raw effort to a CSV for tuning.

Fixed-rate (zero-order hold) so every row is time-aligned across topics that
publish at different rates:
  /cmd_vel_nav or /cmd_vel  -> desired twist (cmd_v, cmd_w)
  /odom                     -> actual twist  (odom_v, odom_w)
  /ackermann/cmd_effort     -> signal sent   (eff_steer, eff_throttle)

7 columns: t, cmd_v, cmd_w, odom_v, odom_w, eff_steer, eff_throttle
"""
import csv
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

COLUMNS = ["t", "cmd_v", "cmd_w", "odom_v", "odom_w", "eff_steer", "eff_throttle"]


class TwistLogger(Node):
    def __init__(self):
        super().__init__("twist_logger")
        hz = max(1.0, float(self.declare_parameter("rate_hz", 20.0).value))
        out_dir = os.path.expanduser(
            str(self.declare_parameter("out_dir", "~/ros2_ws/logs").value))
        cmd_topic = str(self.declare_parameter("cmd_topic", "/cmd_vel").value)
        cmd_stamped_topic = str(
            self.declare_parameter("cmd_stamped_topic", "/cmd_vel_nav").value)
        odom_topic = str(self.declare_parameter("odom_topic", "/odom").value)
        effort_topic = str(
            self.declare_parameter("effort_topic", "/ackermann/cmd_effort").value)

        self.cmd_v = self.cmd_w = 0.0
        self.odom_v = self.odom_w = 0.0
        self.eff_steer = self.eff_throttle = 0.0

        self.create_subscription(Twist, cmd_topic, self._on_cmd, 10)
        self.create_subscription(
            TwistStamped, cmd_stamped_topic, lambda m: self._on_cmd(m.twist), 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(Float32MultiArray, effort_topic, self._on_effort, 10)

        os.makedirs(out_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(out_dir, f"twist_{stamp}.csv")
        self._file = open(self.path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(COLUMNS)
        self._file.flush()

        self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(
            f"twist_logger: {hz:.0f} Hz -> {self.path} (cols: {', '.join(COLUMNS)})")

    def _on_cmd(self, msg: Twist):
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z

    def _on_odom(self, msg: Odometry):
        self.odom_v = msg.twist.twist.linear.x
        self.odom_w = msg.twist.twist.angular.z

    def _on_effort(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            self.eff_steer = float(msg.data[0])
            self.eff_throttle = float(msg.data[1])

    def _tick(self):
        t = self.get_clock().now().nanoseconds / 1e9
        self._writer.writerow([
            f"{t:.3f}",
            f"{self.cmd_v:.4f}", f"{self.cmd_w:.4f}",
            f"{self.odom_v:.4f}", f"{self.odom_w:.4f}",
            f"{self.eff_steer:.4f}", f"{self.eff_throttle:.4f}",
        ])
        self._file.flush()

    def destroy_node(self):
        try:
            if not self._file.closed:
                self._file.close()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TwistLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
