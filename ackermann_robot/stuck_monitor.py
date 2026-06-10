#!/usr/bin/env python3
"""Detect commanded-but-not-moving stalls (cmd_vel vs odom).

Subscribes to Nav2 cmd (/cmd_vel_nav) and /odom. When a non-zero drive command
persists but linear speed and pose displacement stay near zero, publishes
/robot_stuck (std_msgs/Bool) and logs a throttled warning.

This complements Nav2 PoseProgressChecker (path following only) and catches
cases where recovery behaviors spin wheels against a wall without odom progress.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from action_msgs.srv import CancelGoal


class StuckMonitor(Node):
    def __init__(self):
        super().__init__("stuck_monitor")
        p = lambda n, d: self.declare_parameter(n, d).value

        cmd_topic = str(p("cmd_topic", "/cmd_vel_nav"))
        cmd_fallback = str(p("cmd_fallback_topic", "/cmd_vel"))
        odom_topic = str(p("odom_topic", "/odom"))
        self.cmd_min_v = float(p("cmd_min_linear", 0.06))
        self.cmd_min_w = float(p("cmd_min_angular", 0.20))
        self.odom_min_v = float(p("odom_min_linear", 0.025))
        self.min_displacement = float(p("min_displacement_m", 0.04))
        self.stuck_time_s = float(p("stuck_time_s", 2.5))
        # Act on stuck, not just report: cancel the nav goal after this many
        # extra seconds of grinding (costmap-blind obstacles: too close for the
        # camera, below the lidar). Canceling stops cmd_vel -> stale timeout
        # zeroes the motors.
        self.cancel_goals = bool(p("cancel_goals", True))
        self.cancel_after_s = float(p("cancel_after_s", 4.0))
        self.cmd_grace_s = float(p("cmd_grace_s", 0.4))
        hz = max(1.0, float(p("check_hz", 10.0)))

        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.odom_v = 0.0
        self.pose = None
        self.cmd_active_since = None
        self.window_start = None
        self.window_start_pose = None
        self.stuck = False

        self.create_subscription(
            TwistStamped, cmd_topic, lambda m: self._on_cmd(m.twist), 10)
        self.create_subscription(Twist, cmd_fallback, self._on_cmd, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.pub = self.create_publisher(Bool, str(p("stuck_topic", "/robot_stuck")), 10)
        self.cancel_cli = self.create_client(
            CancelGoal, "/navigate_to_pose/_action/cancel_goal")
        self.cancel_sent = False
        self.create_timer(1.0 / hz, self._tick)

        self.get_logger().info(
            f"stuck_monitor: cmd>{self.cmd_min_v:.2f} m/s for {self.stuck_time_s:.1f}s "
            f"but odom<{self.odom_min_v:.2f} m/s and disp<{self.min_displacement:.2f} m"
        )

    def _on_cmd(self, msg: Twist):
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z

    def _on_odom(self, msg: Odometry):
        self.odom_v = msg.twist.twist.linear.x
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)

    def _cmd_demands_motion(self) -> bool:
        return abs(self.cmd_v) >= self.cmd_min_v or abs(self.cmd_w) >= self.cmd_min_w

    def _displacement(self, start):
        if self.pose is None or start is None:
            return 0.0
        dx = self.pose[0] - start[0]
        dy = self.pose[1] - start[1]
        return math.hypot(dx, dy)

    def _tick(self):
        now = self.get_clock().now()
        demanding = self._cmd_demands_motion()

        if not demanding:
            self.cmd_active_since = None
            self.window_start = None
            self.window_start_pose = None
            self._set_stuck(False)
            self.cancel_sent = False
            return

        if self.cmd_active_since is None:
            self.cmd_active_since = now
            return

        if (now - self.cmd_active_since).nanoseconds < int(self.cmd_grace_s * 1e9):
            return

        if self.window_start is None:
            self.window_start = now
            self.window_start_pose = self.pose
            return

        elapsed = (now - self.window_start).nanoseconds / 1e9
        disp = self._displacement(self.window_start_pose)
        moving = abs(self.odom_v) >= self.odom_min_v or disp >= self.min_displacement

        if moving:
            self.window_start = now
            self.window_start_pose = self.pose
            self._set_stuck(False)
            self.cancel_sent = False
            return

        if elapsed >= self.stuck_time_s:
            self._set_stuck(
                True,
                f"cmd v={self.cmd_v:+.2f} w={self.cmd_w:+.2f} but "
                f"odom v={self.odom_v:+.2f}, disp={disp:.3f} m over {elapsed:.1f}s",
            )
        if (self.cancel_goals and not self.cancel_sent
                and elapsed >= self.stuck_time_s + self.cancel_after_s):
            if self.cancel_cli.service_is_ready():
                self.cancel_cli.call_async(CancelGoal.Request())  # empty = cancel all
                self.cancel_sent = True
                self.get_logger().warn(
                    f"stuck {elapsed:.1f}s with motion commanded — canceling nav goal")
            else:
                self.get_logger().warn("stuck but nav cancel service not ready",
                                       throttle_duration_sec=5.0)

    def _set_stuck(self, value: bool, detail: str = ""):
        if value == self.stuck:
            return
        self.stuck = value
        self.pub.publish(Bool(data=value))
        if value:
            self.get_logger().warn(f"[stuck_monitor] STUCK — {detail}")
        else:
            self.get_logger().info("[stuck_monitor] unstuck — motion resumed")


def main(args=None):
    rclpy.init(args=args)
    node = StuckMonitor()
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
