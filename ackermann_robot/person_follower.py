#!/usr/bin/env python3
"""Drive Nav2 to follow /person_target using the stock follow_point BT.

Glue between person_tracker and Nav2's dynamic-object-following:
  - on enable (+ a fresh /person_target): send ONE NavigateToPose goal with
    behavior_tree=follow_point.xml (replans to a moving goal at 1 Hz and
    truncates the path 1 m short of the person)
  - while following: forward each /person_target (odom) to /goal_update
    (map frame), which the BT's GoalUpdater consumes
  - person lost > lost_timeout_s, or disabled: cancel the nav goal

Safety: starts DISABLED. Enable with
  ros2 service call /follow/enable std_srvs/srv/SetBool "{data: true}"
"""
import math

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class PersonFollower(Node):
    def __init__(self):
        super().__init__("person_follower")
        p = lambda n, d: self.declare_parameter(n, d).value
        default_bt = get_package_share_directory("ackermann_robot") + \
            "/behavior_trees/follow_person.xml"
        self.bt_xml = str(p("behavior_tree", default_bt))
        self.retreat_dist = float(p("retreat_dist", 0.40))
        self.standoff = float(p("standoff_m", 0.75))
        self.resume_dist = float(p("resume_dist", 0.70))
        self.retreat_speed = float(p("retreat_speed", 0.13))
        self.retreat_max_s = float(p("retreat_max_s", 4.0))
        self.lost_timeout_s = float(p("lost_timeout_s", 8.0))
        self.goal_frame = str(p("goal_frame", "map"))
        self.update_min_period_s = float(p("update_min_period_s", 0.3))

        self.enabled = bool(p("start_enabled", True))
        self.goal_handle = None
        self.goal_pending = False
        self.last_target = None          # PoseStamped in goal_frame
        self.last_target_s = None
        self.last_update_s = 0.0

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.create_subscription(PoseStamped, "/person_target", self.on_target, 10)
        self.pub_update = self.create_publisher(PoseStamped, "/goal_update", 10)
        self.pub_status = self.create_publisher(String, "/follow/status", 5)
        self.create_service(SetBool, "/follow/enable", self.on_enable)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.retreating_since = None
        self.create_timer(0.2, self.tick)
        self.get_logger().info(
            f"person_follower ready ({'ARMED' if self.enabled else 'DISABLED'}). "
            f"BT: {self.bt_xml}")

    # --- helpers -----------------------------------------------------------

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def to_goal_frame(self, ps):
        if ps.header.frame_id == self.goal_frame:
            return ps
        try:
            t = self.tf_buf.lookup_transform(self.goal_frame, ps.header.frame_id,
                                             rclpy.time.Time())
        except Exception:
            return None
        tr = t.transform.translation
        yaw = yaw_of(t.transform.rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.goal_frame
        x, y = ps.pose.position.x, ps.pose.position.y
        px = tr.x + c * x - s * y
        py = tr.y + s * x + c * y
        # pull the goal back toward the robot: the person is an obstacle in
        # the costmap, a goal inside them can never be planned to
        gx, gy = px, py
        try:
            t2 = self.tf_buf.lookup_transform(self.goal_frame, "base_link",
                                              rclpy.time.Time())
            rx, ry = t2.transform.translation.x, t2.transform.translation.y
            d = math.hypot(px - rx, py - ry)
            if d > 1e-3:
                back = min(self.standoff, d)
                gx = px - (px - rx) / d * back
                gy = py - (py - ry) / d * back
        except Exception:
            pass
        out.pose.position.x = gx
        out.pose.position.y = gy
        # face the person from the goal point
        gyaw = math.atan2(py - gy, px - gx)
        out.pose.orientation.z = math.sin(gyaw / 2)
        out.pose.orientation.w = math.cos(gyaw / 2)
        return out

    def status(self, text):
        self.pub_status.publish(String(data=text))
        self.get_logger().info(text)

    # --- callbacks ----------------------------------------------------------

    def on_enable(self, req, res):
        self.enabled = req.data
        if not self.enabled:
            self.cancel_goal("follow disabled")
        res.success = True
        res.message = "following enabled" if self.enabled else "following disabled"
        self.status(res.message)
        return res

    def on_target(self, ps):
        goal_ps = self.to_goal_frame(ps)
        if goal_ps is None:
            return
        self.last_target = goal_ps
        self.last_target_s = self.now_s()
        if (self.goal_handle is not None
                and self.now_s() - self.last_update_s >= self.update_min_period_s):
            self.pub_update.publish(goal_ps)
            self.last_update_s = self.now_s()

    def target_distance(self):
        if self.last_target is None:
            return None
        try:
            t = self.tf_buf.lookup_transform(self.goal_frame, "base_link",
                                             rclpy.time.Time())
        except Exception:
            return None
        dx = self.last_target.pose.position.x - t.transform.translation.x
        dy = self.last_target.pose.position.y - t.transform.translation.y
        return math.hypot(dx, dy)

    def tick(self):
        if not self.enabled:
            return
        fresh = (self.last_target_s is not None
                 and self.now_s() - self.last_target_s < self.lost_timeout_s)
        dist = self.target_distance() if fresh else None

        # too close: cancel nav and back straight up until resume_dist
        if self.retreating_since is not None:
            if (not fresh or dist is None or dist >= self.resume_dist
                    or self.now_s() - self.retreating_since > self.retreat_max_s):
                self.retreating_since = None
                self.pub_cmd.publish(Twist())  # stop
                self.status("retreat done")
            else:
                cmd = Twist()
                cmd.linear.x = -abs(self.retreat_speed)
                self.pub_cmd.publish(cmd)
            return
        if fresh and dist is not None and dist < self.retreat_dist:
            self.cancel_goal("person too close — retreating")
            self.retreating_since = self.now_s()
            return

        if self.goal_handle is None and not self.goal_pending and fresh:
            self.send_goal()
        elif self.goal_handle is not None and not fresh:
            self.cancel_goal(f"person lost > {self.lost_timeout_s:.0f}s")

    # --- nav2 action --------------------------------------------------------

    def send_goal(self):
        if not self.nav_client.server_is_ready():
            return
        goal = NavigateToPose.Goal()
        goal.pose = self.last_target
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.behavior_tree = self.bt_xml
        self.goal_pending = True
        fut = self.nav_client.send_goal_async(goal)
        fut.add_done_callback(self.on_goal_response)
        self.status("following person")

    def on_goal_response(self, fut):
        self.goal_pending = False
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.status("nav goal rejected")
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(self.on_result)

    def on_result(self, fut):
        self.goal_handle = None
        self.status("follow goal ended")

    def cancel_goal(self, why):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
            self.status(f"follow canceled: {why}")


def main():
    rclpy.init()
    node = PersonFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
