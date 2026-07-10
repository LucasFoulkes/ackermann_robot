"""Retrace recovery: back up along the path the robot just drove.

Drop-in replacement for the nav2 BackUp behavior (same nav2_msgs/action/BackUp
interface, action name "retrace"; the BT's <BackUp server_name="retrace">
points here). Stock BackUp reverses in a straight line — but the robot's only
low-obstacle sensing (depth_floor_scan) faces FORWARD, so straight-back can
blindly climb a floor obstacle it swerved around seconds earlier. The ground
the robot just drove over is the one strip of floor known to be safe, so a
recovery reverse should retrace it.

How: record odom breadcrumbs while driving forward (the only direction the
floor camera has cleared); on a BackUp goal, reverse pure-pursuit onto the
breadcrumb trail — target the crumb ~lookback m behind, curvature
kappa = 2*y_local/d^2 (the circle through both poses tangent to heading; the
same circle serves forward and reverse traversal), cmd v = -speed,
w = v * kappa on /cmd_vel_nav. Downstream cmd_vel_to_effort still applies the
direction interlock, ESC double-tap, stiction kick, watchdog and ceilings.

If the trail runs out (fresh boot, or already consumed) it falls back to a
straight reverse for the remaining distance — no worse than stock BackUp.
"""

import math
import time
from collections import deque

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import BackUp


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RetraceRecovery(Node):
    def __init__(self):
        super().__init__("retrace_recovery")
        p = lambda n, d: self.declare_parameter(n, d).value
        self.cmd_topic = p("cmd_topic", "/cmd_vel_nav")
        self.odom_topic = p("odom_topic", "/odom")
        self.crumb_spacing = p("crumb_spacing", 0.05)   # m between breadcrumbs
        self.max_crumbs = int(p("max_crumbs", 400))     # ~20 m of trail
        self.lookback = p("lookback", 0.40)             # m, reverse pure-pursuit carrot
        self.kappa_max = p("kappa_max", 0.8)            # 1/m, match cmd_vel_to_effort
        self.record_v_min = p("record_v_min", 0.07)     # m/s, forward motion gate
        self.consume_radius = p("consume_radius", 0.15) # m, drop crumbs we've re-passed
        self.control_hz = p("control_hz", 15.0)

        self.pose = None          # (x, y, yaw) in odom frame
        self.odom_v = 0.0
        self.crumbs = deque(maxlen=self.max_crumbs)  # (x, y), oldest first
        self._moving_ticks = 0

        cb = ReentrantCallbackGroup()
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 20,
                                 callback_group=cb)
        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, 5)
        self._server = ActionServer(
            self, BackUp, "retrace",
            execute_callback=self._execute,
            goal_callback=lambda req: GoalResponse.ACCEPT,
            cancel_callback=lambda gh: CancelResponse.ACCEPT,
            callback_group=cb)
        self.get_logger().info(
            "retrace action server up (BackUp-compatible); recording forward "
            f"breadcrumbs every {self.crumb_spacing} m, trail {self.max_crumbs} pts")

    # --- breadcrumb recording -------------------------------------------
    def _on_odom(self, msg: Odometry):
        pp = msg.pose.pose
        self.pose = (pp.position.x, pp.position.y, _yaw(pp.orientation))
        self.odom_v = msg.twist.twist.linear.x
        # Record only during sustained forward motion: 3 consecutive moving
        # samples so a single rf2o phantom-velocity spike at rest (measured
        # 12% of at-rest samples, 2026-07-02) can't drop crumbs in place.
        if self.odom_v > self.record_v_min:
            self._moving_ticks += 1
        else:
            self._moving_ticks = 0
            return
        if self._moving_ticks < 3:
            return
        x, y, _ = self.pose
        if not self.crumbs or math.hypot(x - self.crumbs[-1][0],
                                         y - self.crumbs[-1][1]) >= self.crumb_spacing:
            self.crumbs.append((x, y))

    # --- reverse pure pursuit -------------------------------------------
    def _carrot(self):
        """Newest-to-oldest: consume crumbs already re-passed, then return the
        first crumb at least `lookback` behind the robot. None = trail dry."""
        if self.pose is None:
            return None
        x, y, _ = self.pose
        while self.crumbs and math.hypot(x - self.crumbs[-1][0],
                                         y - self.crumbs[-1][1]) < self.consume_radius:
            self.crumbs.pop()
        for cx, cy in reversed(self.crumbs):
            if math.hypot(x - cx, y - cy) >= self.lookback:
                return (cx, cy)
        return None

    def _cmd(self, v, w):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.twist.linear.x = v
        m.twist.angular.z = w
        try:
            self.cmd_pub.publish(m)
        except rclpy.exceptions.InvalidHandle:
            pass  # context torn down mid-goal (SIGINT during retrace)
        except Exception as e:
            if "context is invalid" not in str(e):
                raise

    def _execute(self, goal_handle):
        goal = goal_handle.request
        dist = abs(goal.target.x) or 0.5
        speed = abs(goal.speed) or 0.20
        allowance = goal.time_allowance.sec + goal.time_allowance.nanosec * 1e-9
        if allowance <= 0.0:
            allowance = 15.0
        n_crumbs = len(self.crumbs)
        self.get_logger().info(
            f"retrace: {dist:.2f} m @ {speed:.2f} m/s, trail={n_crumbs} crumbs")

        result = BackUp.Result()
        fb = BackUp.Feedback()
        travelled = 0.0
        last = self.pose
        t0 = time.monotonic()
        dt = 1.0 / self.control_hz

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._cmd(0.0, 0.0)
                goal_handle.canceled()
                return result
            if time.monotonic() - t0 > allowance:
                self._cmd(0.0, 0.0)
                self.get_logger().warn("retrace: time allowance exceeded")
                goal_handle.abort()
                return result
            if self.pose is None:
                time.sleep(dt)
                continue

            # progress = actual ground covered (pose delta, not commanded)
            if last is not None:
                travelled += math.hypot(self.pose[0] - last[0], self.pose[1] - last[1])
            last = self.pose
            if travelled >= dist:
                break

            carrot = self._carrot()
            direction = -1.0  # default: reverse (stock BackUp semantics)
            if carrot is None:
                kappa = 0.0  # trail dry -> straight back, same as stock BackUp
            else:
                x, y, yaw = self.pose
                dx, dy = carrot[0] - x, carrot[1] - y
                xl = math.cos(yaw) * dx + math.sin(yaw) * dy
                yl = -math.sin(yaw) * dx + math.cos(yaw) * dy
                d2 = xl * xl + yl * yl
                kappa = max(-self.kappa_max, min(self.kappa_max, 2.0 * yl / d2))
                if xl > 0.0:
                    # Trail is AHEAD: we got stuck while REVERSING, so the
                    # known-safe ground (and the floor camera's view) is in
                    # front — escape FORWARD along the trail instead of
                    # backing deeper into unseen territory.
                    direction = 1.0
            v = direction * speed
            self._cmd(v, v * kappa)
            fb.distance_traveled = travelled
            goal_handle.publish_feedback(fb)
            time.sleep(dt)

        self._cmd(0.0, 0.0)
        goal_handle.succeed()
        self.get_logger().info(f"retrace: done, {travelled:.2f} m")
        return result


def main():
    rclpy.init()
    node = RetraceRecovery()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
