#!/usr/bin/env python3
"""Discover max_steer_angle, min turning radius, and max_speed BY DRIVING.

We only know servo ticks, not the physical steering geometry — so drive the robot
(teleop) and measure the truth from rf2o /odom:

  * FULL-LOCK circle  -> turning radius R = v/omega  -> max_steer_angle = atan(L/R)
                         and minimum_turning_radius = R   (feeds the Nav2 planner)
  * STRAIGHT at steady throttle -> max_speed = v / throttle  (feeds cmd_vel_to_effort)

This node only LISTENS (no commands). Drive with teleop:
  1. Hold full steering lock + slow forward, drive 1-2 full circles -> read STEER block.
  2. Drive straight at a steady throttle for a few seconds      -> read SPEED block.

Then paste the printed values into:
  - cmd_vel_to_effort:  max_steer_angle, max_speed
  - nav2_params.yaml planner GridBased:  minimum_turning_radius
so Nav2 only plans paths the robot can actually follow.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray


class SteerCalibration(Node):
    def __init__(self):
        super().__init__("steer_calibration")
        p = lambda n, d: self.declare_parameter(n, d).value

        self.L = float(p("wheelbase", 0.2775))
        self.full_lock = float(p("full_lock_threshold", 0.9))   # |steer| to count as lock
        self.straight = float(p("straight_threshold", 0.1))     # |steer| to count as straight
        self.min_v = float(p("min_speed", 0.05))
        self.min_w = float(p("min_omega", 0.05))

        self.effort = (0.0, 0.0)        # latest (steer, throttle)
        self.radii = []                 # full-lock turning radii (m)
        self.steer_angles = []          # implied steering angle (rad)
        self.speeds = []                # straight max_speed estimates (m/s)

        self.create_subscription(Float32MultiArray, p("effort_topic", "/ackermann/cmd_effort"),
                                 self._on_effort, 10)
        self.create_subscription(Odometry, p("odom_topic", "/odom"), self._on_odom, 20)
        self.create_timer(1.0, self._report)
        self.get_logger().info("steer_calibration listening — drive full-lock circles, then straight.")

    def _on_effort(self, msg):
        if len(msg.data) >= 2:
            self.effort = (msg.data[0], msg.data[1])

    def _on_odom(self, msg):
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        steer, thr = self.effort

        if abs(steer) >= self.full_lock and abs(v) >= self.min_v and abs(w) >= self.min_w:
            r = abs(v / w)
            self.radii.append(r)
            self.steer_angles.append(math.atan(self.L / r))
            self.radii = self.radii[-300:]
            self.steer_angles = self.steer_angles[-300:]
        elif abs(steer) <= self.straight and abs(v) >= self.min_v and abs(thr) >= 0.1:
            self.speeds.append(abs(v / thr))
            self.speeds = self.speeds[-300:]

    def _report(self):
        lines = []
        if self.steer_angles:
            r = float(np.median(self.radii))
            a = float(np.median(self.steer_angles))
            lines.append(
                f"STEER (n={len(self.steer_angles)}): min_turning_radius={r:.3f} m | "
                f"max_steer_angle={a:.3f} rad ({math.degrees(a):.1f} deg)")
        else:
            lines.append("STEER: drive a FULL-LOCK circle (|steer|>=0.9, moving)")
        if self.speeds:
            ms = float(np.median(self.speeds))
            lines.append(f"SPEED (n={len(self.speeds)}): max_speed={ms:.3f} m/s")
        else:
            lines.append("SPEED: drive STRAIGHT at steady throttle")
        self.get_logger().info(" | ".join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = SteerCalibration()
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
