#!/usr/bin/env python3
"""Republish IMU at a fixed rate so robot_localization is not flooded on the Pi.

The D435i fused stream can be ~200 Hz; the EKF processes every IMU callback and
can starve RF2O / drop /odom to ~5 Hz. This node passes the latest sample at
imu_hz (default 30), which matches ekf_rf2o_imu.yaml.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuThrottle(Node):
    def __init__(self):
        super().__init__("imu_throttle")
        hz = float(self.declare_parameter("imu_hz", 30.0).value)
        self.period_ns = int(1e9 / max(1.0, hz))
        self.latest = None

        in_topic = str(self.declare_parameter("in_topic", "/imu/data").value)
        out_topic = str(self.declare_parameter("out_topic", "/imu/data_ekf").value)

        self.create_subscription(Imu, in_topic, self._on_imu, 1)
        self.pub = self.create_publisher(Imu, out_topic, 10)
        self.create_timer(self.period_ns / 1e9, self._publish)

    def _on_imu(self, msg: Imu):
        self.latest = msg

    def _publish(self):
        if self.latest is not None:
            self.pub.publish(self.latest)


def main():
    rclpy.init()
    node = ImuThrottle()
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
