#!/usr/bin/env python3
"""Twist → Float32MultiArray [steer, throttle] in -1..1 using Ackermann kinematics."""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__("cmd_vel_bridge")
        p = self._p
        self._max_speed = p("max_speed", 2.0)          # m/s at full throttle
        self._max_steer = p("max_steering_angle", 0.37) # rad (~21 deg)
        self._wheelbase = p("wheelbase", 0.27)           # m

        self.create_subscription(Twist, "cmd_vel_nav", self._on_twist, 10)
        self._pub = self.create_publisher(Float32MultiArray, "/ackermann/cmd_effort", 10)
        self.get_logger().info(
            f"max_speed={self._max_speed} m/s  "
            f"max_steer={self._max_steer} rad  "
            f"wheelbase={self._wheelbase} m"
        )

    def _p(self, name, default):
        return self.declare_parameter(name, default).value

    def _on_twist(self, msg: Twist):
        v, w = msg.linear.x, msg.angular.z

        throttle = clamp(v / self._max_speed) if self._max_speed else 0.0

        if abs(v) < 0.01:
            angle = math.copysign(self._max_steer, w) if abs(w) > 0.001 else 0.0
        else:
            angle = math.atan(self._wheelbase * w / v)

        steer = clamp(angle / self._max_steer) if self._max_steer else 0.0

        out = Float32MultiArray()
        out.data = [steer, throttle]
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
