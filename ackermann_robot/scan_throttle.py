#!/usr/bin/env python3
"""Republish /scan at a lower rate for slam_toolbox only.

RF2O and the local costmap keep the full-rate /scan. SLAM async on the Pi
falls behind and fills the message-filter queue when fed every lidar frame.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanThrottle(Node):
    def __init__(self):
        super().__init__("scan_throttle")
        hz = float(self.declare_parameter("hz", 5.0).value)
        in_topic = str(self.declare_parameter("in_topic", "/scan").value)
        out_topic = str(self.declare_parameter("out_topic", "/scan_slam").value)
        self.period_ns = int(1e9 / max(1.0, hz))
        self.latest = None

        self.create_subscription(
            LaserScan, in_topic, self._on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(LaserScan, out_topic, 10)
        self.create_timer(self.period_ns / 1e9, self._publish)
        self.get_logger().info(
            f"scan_throttle: {in_topic} -> {out_topic} at {hz:.1f} Hz for SLAM"
        )

    def _on_scan(self, msg: LaserScan):
        self.latest = msg

    def _publish(self):
        if self.latest is not None:
            self.pub.publish(self.latest)


def main():
    rclpy.init()
    node = ScanThrottle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
