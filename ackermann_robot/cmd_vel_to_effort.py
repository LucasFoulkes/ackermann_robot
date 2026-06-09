#!/usr/bin/env python3
"""cmd_vel -> [steer, throttle] on /ackermann/cmd_effort for ackermann_driver.

Pipeline:
  1. Bicycle model (wheelbase from URDF): v, w -> logical steer + logical throttle
  2. logical_to_raw(): compensate motor/servo deadband (tune in cmd_vel_to_effort.yaml)
  3. Publish Float32MultiArray [data[0]=steer, data[1]=throttle]

Measurements (robot.urdf.xacro):
  wheelbase = 0.2775 m  (rear axle to front axle)
  max_steer_angle = seed only — refine with steer_calibration / field tests

max_speed: cmd_vel linear.x at this value -> logical throttle 1.0 -> raw ~1.0 after
deadband map. Tune from /odom (first log suggested ~0.85 m/s at strong forward effort).
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

from ackermann_robot.effort_deadband import logical_to_raw


def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))


class CmdVelToEffort(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_effort")
        p = lambda n, d: self.declare_parameter(n, d).value

        self.L = float(p("wheelbase", 0.2775))
        self.max_speed = max(0.05, float(p("max_speed", 0.85)))
        self.max_steer = max(0.05, float(p("max_steer_angle", math.radians(15.0))))
        # Bicycle model: |omega| <= |v| * kappa_max, kappa_max = tan(delta_max) / L.
        self.kappa_max = math.tan(self.max_steer) / self.L
        self.v_eps = float(p("v_epsilon", 0.02))
        self.timeout_ns = int(float(p("cmd_timeout_s", 0.5)) * 1e9)
        hz = max(1.0, float(p("control_hz", 50.0)))
        self.dt = 1.0 / hz

        self.apply_deadband = bool(p("apply_deadband", True))
        self.thr_dead_fwd = float(p("throttle_deadband_fwd", 0.45))
        self.thr_dead_rev = float(p("throttle_deadband_rev", 0.55))
        self.steer_dead_pos = float(p("steer_deadband_pos", 0.55))
        self.steer_dead_neg = float(p("steer_deadband_neg", 0.60))

        self.closed_loop = bool(p("closed_loop", False))
        self.ki = float(p("vel_ki", 0.3))
        self.trim_max = float(p("vel_trim_max", 0.2))
        self.odom_timeout_ns = int(float(p("odom_timeout_s", 0.5)) * 1e9)

        self.cmd = (0.0, 0.0)
        self.last_cmd_ns = self.get_clock().now().nanoseconds
        self.v_meas = 0.0
        self.last_odom_ns = 0
        self.integral = 0.0

        self.create_subscription(Twist, p("cmd_topic", "/cmd_vel"), self._on_cmd, 10)
        self.create_subscription(
            TwistStamped, p("cmd_stamped_topic", "/cmd_vel_nav"),
            lambda m: self._on_cmd(m.twist), 10)
        if self.closed_loop:
            self.create_subscription(Odometry, p("odom_topic", "/odom"), self._on_odom, 10)
        self.pub = self.create_publisher(
            Float32MultiArray, p("effort_topic", "/ackermann/cmd_effort"), 10)
        self.create_timer(self.dt, self._tick)

        self.get_logger().info(
            f"wheelbase={self.L:.4f} m, max_speed={self.max_speed:.2f} m/s, "
            f"max_steer={math.degrees(self.max_steer):.1f} deg "
            f"(kappa_max={self.kappa_max:.2f} 1/m), deadband={self.apply_deadband}")

    def _on_cmd(self, msg: Twist):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.last_cmd_ns = self.get_clock().now().nanoseconds

    def _on_odom(self, msg: Odometry):
        self.v_meas = msg.twist.twist.linear.x
        self.last_odom_ns = self.get_clock().now().nanoseconds

    def _raw_throttle(self, logical: float) -> float:
        if not self.apply_deadband:
            return clamp(logical)
        return logical_to_raw(logical, self.thr_dead_rev, self.thr_dead_fwd)

    def _raw_steer(self, logical: float) -> float:
        if not self.apply_deadband:
            return clamp(logical)
        return logical_to_raw(logical, self.steer_dead_neg, self.steer_dead_pos)

    def _tick(self):
        now = self.get_clock().now().nanoseconds
        if now - self.last_cmd_ns > self.timeout_ns:
            v, w = 0.0, 0.0
        else:
            v, w = self.cmd

        if abs(v) > self.v_eps:
            # Nav2 RPP has no Ackermann limit — it can command |w| >> |v|*kappa_max when
            # correcting path error. Clamp omega so steer effort matches physical full lock.
            w_max = abs(v) * self.kappa_max
            w = clamp(w, -w_max, w_max)
            steer_log = clamp(math.atan(self.L * w / v) / self.max_steer, -1.0, 1.0)
        else:
            steer_log = 0.0

        throttle_log = clamp(v / self.max_speed, -1.0, 1.0)
        if self.closed_loop:
            throttle_log = clamp(throttle_log + self._speed_trim(v, now), -1.0, 1.0)

        steer = self._raw_steer(steer_log)
        throttle = self._raw_throttle(throttle_log)

        self.pub.publish(Float32MultiArray(data=[float(steer), float(throttle)]))

    def _speed_trim(self, v_cmd, now):
        if abs(v_cmd) < 0.05 or (now - self.last_odom_ns) > self.odom_timeout_ns:
            self.integral = 0.0
            return 0.0
        err = v_cmd - self.v_meas
        self.integral = clamp(self.integral + err * self.dt, -0.3, 0.3)
        return clamp(self.ki * self.integral, -self.trim_max, self.trim_max)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToEffort()
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
