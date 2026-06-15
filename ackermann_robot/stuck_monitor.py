#!/usr/bin/env python3
"""Detect commanded-but-not-moving stalls (cmd_vel vs odom) and escape them.

Subscribes to Nav2 cmd (/cmd_vel_nav) and /odom. When a non-zero drive command
persists but linear speed and pose displacement stay near zero, publishes
/robot_stuck (std_msgs/Bool) and logs a throttled warning.

On a confirmed stall it cancels the nav goal, then runs a one-shot ESCAPE
REFLEX: drive a short pulse in the OPPOSITE direction of the jammed command
(bump-escape — the space we just came from was free moments ago). This is the
only recovery that works against obstacles the costmaps cannot see: below the
lidar plane (16.5 cm) there is no rear sensor coverage, so Nav2's own
recoveries either trust a clear costmap and grind, or are direction-fixed
(BackUp reverses even if reverse is what jammed).

This complements Nav2 PoseProgressChecker (path following only) and catches
cases where recovery behaviors spin wheels against a wall without odom progress.
"""
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
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
        # A turning robot is making progress, not stuck. Entering a hard turn the
        # steering lag drops linear speed to near 0 for ~1-2 s while the robot
        # still rotates -> was false-flagged stuck -> needless cancel+escape
        # mid-path (2026-06-14). Count yaw rate above this as motion.
        self.odom_min_w = float(p("odom_min_yaw", 0.10))
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

        # Escape reflex: after canceling, pulse the opposite direction of the
        # jammed command. Direction comes from cmd_v sign at stall time.
        self.escape_enabled = bool(p("escape_enabled", True))
        self.escape_speed = float(p("escape_speed", 0.25))      # m/s
        self.escape_dist = float(p("escape_dist", 0.30))        # m, stop after this
        self.escape_timeout_s = float(p("escape_timeout_s", 4.0))
        self.escape_delay_s = float(p("escape_delay_s", 0.6))   # let cancel settle

        # After a successful escape, re-send the goal we canceled so the robot
        # finishes the job instead of sitting freed-but-idle. Goal is cached
        # from /goal_pose (RViz "Nav2 Goal" tool). resume_max bounds the
        # jam->escape->resume loop for genuinely unreachable goals.
        self.resume_goal = bool(p("resume_goal_after_escape", True))
        self.resume_max = int(p("resume_max", 2))
        self.last_goal = None
        self.resume_count = 0

        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.odom_v = 0.0
        self.odom_w = 0.0
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
        # Escape pulses go out as plain Twist on the bridge's fallback topic.
        self.cmd_pub = self.create_publisher(Twist, str(p("escape_cmd_topic", "/cmd_vel")), 10)
        self.create_subscription(PoseStamped, str(p("goal_topic", "/goal_pose")),
                                 self._on_goal, 10)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.escape_phase = None       # None | (start_time, start_pose, direction)
        self.escape_armed_at = None    # time cancel was sent (delay before pulsing)
        self.jam_dir = 0.0             # sign of cmd_v when the stall was confirmed
        self.create_timer(1.0 / hz, self._tick)

        self.get_logger().info(
            f"stuck_monitor: cmd>{self.cmd_min_v:.2f} m/s for {self.stuck_time_s:.1f}s "
            f"but odom<{self.odom_min_v:.2f} m/s and disp<{self.min_displacement:.2f} m"
        )

    def _on_cmd(self, msg: Twist):
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z

    def _on_goal(self, msg: PoseStamped):
        self.last_goal = msg
        self.resume_count = 0

    def _on_odom(self, msg: Odometry):
        self.odom_v = msg.twist.twist.linear.x
        self.odom_w = msg.twist.twist.angular.z
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

        # Escape reflex phases run first: while active we are the cmd source,
        # so the normal cmd-vs-odom logic must not run on our own commands.
        if self.escape_armed_at is not None or self.escape_phase is not None:
            self._run_escape(now)
            return

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
        moving = (abs(self.odom_v) >= self.odom_min_v
                  or abs(self.odom_w) >= self.odom_min_w  # turning = progress
                  or disp >= self.min_displacement)

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
                # Arm the escape reflex with the jam direction. Skip if the
                # stall was a pure-rotation command (no direction to invert).
                if self.escape_enabled and abs(self.cmd_v) >= self.cmd_min_v:
                    self.jam_dir = math.copysign(1.0, self.cmd_v)
                    self.escape_armed_at = now
            else:
                self.get_logger().warn("stuck but nav cancel service not ready",
                                       throttle_duration_sec=5.0)

    def _run_escape(self, now):
        # Phase 1: short delay so the cancel propagates and Nav2 stops publishing.
        if self.escape_phase is None:
            if (now - self.escape_armed_at).nanoseconds / 1e9 < self.escape_delay_s:
                return
            self.escape_phase = (now, self.pose, -self.jam_dir)
            self.get_logger().warn(
                f"escape reflex: jammed going {'fwd' if self.jam_dir > 0 else 'rev'}, "
                f"pulsing {'fwd' if self.jam_dir < 0 else 'rev'} "
                f"{self.escape_dist:.2f} m @ {self.escape_speed:.2f} m/s")
            return

        start_time, start_pose, direction = self.escape_phase
        elapsed = (now - start_time).nanoseconds / 1e9
        disp = self._displacement(start_pose)

        done = disp >= self.escape_dist
        gave_up = elapsed >= self.escape_timeout_s
        if done or gave_up:
            self.cmd_pub.publish(Twist())  # explicit stop
            escaped = not (gave_up and disp < 0.05)
            if escaped:
                self.get_logger().info(f"escape done: {disp:.2f} m in {elapsed:.1f}s")
            else:
                self.get_logger().error(
                    f"escape failed: {disp:.2f} m in {elapsed:.1f}s — wedged both "
                    "ways, manual help needed")
            self.escape_phase = None
            self.escape_armed_at = None
            self.jam_dir = 0.0
            self.cmd_active_since = None
            self.window_start = None
            self.window_start_pose = None
            self._set_stuck(False)
            self.cancel_sent = False
            if escaped:
                self._resume_goal()
            return

        msg = Twist()
        msg.linear.x = direction * self.escape_speed
        self.cmd_pub.publish(msg)

    def _resume_goal(self):
        """Re-send the goal we canceled, now that the robot is free to replan."""
        if not self.resume_goal:
            return
        if self.last_goal is None:
            self.get_logger().warn(
                "escaped but no cached goal to resume (goals sent outside "
                "/goal_pose, e.g. nav-through-poses, are not cached)")
            return
        if self.resume_count >= self.resume_max:
            self.get_logger().error(
                f"escaped but goal already resumed {self.resume_count}x — giving "
                "up on it (likely unreachable / repeatedly jams)")
            return
        if not self.nav_client.server_is_ready():
            self.get_logger().warn("escaped but navigate_to_pose server not ready")
            return
        self.resume_count += 1
        goal = NavigateToPose.Goal()
        goal.pose = self.last_goal
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.nav_client.send_goal_async(goal)
        px, py = goal.pose.pose.position.x, goal.pose.pose.position.y
        self.get_logger().warn(
            f"resuming goal ({px:.2f}, {py:.2f}) after escape "
            f"[{self.resume_count}/{self.resume_max}]")

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
