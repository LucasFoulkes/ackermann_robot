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
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                       QoSReliabilityPolicy)
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
        self.standoff = float(p("standoff_m", 0.45))
        self.min_chase_dist = float(p("min_chase_dist", 0.9))
        self.lost_timeout_s = float(p("lost_timeout_s", 8.0))
        self.goal_frame = str(p("goal_frame", "map"))
        self.update_min_period_s = float(p("update_min_period_s", 0.3))
        # only START a chase toward a recently-seen person; the longer
        # lost_timeout_s only governs when an active goal is cancelled
        self.send_max_age_s = float(p("send_max_age_s", 1.5))
        self.abort_backoff_s = float(p("abort_backoff_s", 1.0))

        self.enabled = bool(p("start_enabled", True))
        self.goal_handle = None
        self.goal_pending = False
        self.last_target = None          # PoseStamped in goal_frame
        self.last_target_s = None
        self.last_update_s = 0.0
        self.next_send_s = 0.0
        self.map_bounds = None           # (x0, y0, x1, y1) of global costmap

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.create_subscription(PoseStamped, "/person_target", self.on_target, 10)
        cm_qos = QoSProfile(depth=1,
                            reliability=QoSReliabilityPolicy.RELIABLE,
                            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                                 self.on_costmap, cm_qos)
        self.pub_update = self.create_publisher(PoseStamped, "/goal_update", 10)
        self.pub_status = self.create_publisher(String, "/follow/status", 5)
        self.create_service(SetBool, "/follow/enable", self.on_enable)
        self.create_timer(0.2, self.tick)
        self.create_timer(2.0, self.heartbeat)
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
        except Exception as e:
            self.get_logger().warning(f"TF {ps.header.frame_id}->{self.goal_frame} failed: {e}",
                                      throttle_duration_sec=5.0)
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
            # Smac hard-fails on goals outside the (SLAM-map-sized) global
            # costmap; walk the goal toward the robot until inside bounds
            if self.map_bounds is not None:
                x0, y0, x1, y1 = self.map_bounds
                clamped = False
                for _ in range(25):
                    if x0 <= gx <= x1 and y0 <= gy <= y1:
                        break
                    gx = rx + (gx - rx) * 0.85
                    gy = ry + (gy - ry) * 0.85
                    clamped = True
                if clamped:
                    self.get_logger().info(
                        "person outside mapped area — goal clamped into map "
                        "bounds", throttle_duration_sec=5.0)
        except Exception:
            pass
        out.pose.position.x = gx
        out.pose.position.y = gy
        # face the person from the goal point
        gyaw = math.atan2(py - gy, px - gx)
        out.pose.orientation.z = math.sin(gyaw / 2)
        out.pose.orientation.w = math.cos(gyaw / 2)
        return out

    def clamp_into_bounds(self, ps):
        if self.map_bounds is None:
            return
        try:
            t = self.tf_buf.lookup_transform(self.goal_frame, "base_link",
                                             rclpy.time.Time())
        except Exception:
            return
        rx, ry = t.transform.translation.x, t.transform.translation.y
        x0, y0, x1, y1 = self.map_bounds
        gx, gy = ps.pose.position.x, ps.pose.position.y
        for _ in range(25):
            if x0 <= gx <= x1 and y0 <= gy <= y1:
                break
            gx = rx + (gx - rx) * 0.85
            gy = ry + (gy - ry) * 0.85
        ps.pose.position.x, ps.pose.position.y = gx, gy

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

    def on_costmap(self, m):
        margin = 0.15
        x0 = m.info.origin.position.x
        y0 = m.info.origin.position.y
        self.map_bounds = (x0 + margin, y0 + margin,
                           x0 + m.info.width * m.info.resolution - margin,
                           y0 + m.info.height * m.info.resolution - margin)

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

    def heartbeat(self):
        fresh = (self.last_target_s is not None
                 and self.now_s() - self.last_target_s < self.lost_timeout_s)
        d = self.target_distance()
        goal = ("ACTIVE" if self.goal_handle is not None
                else "pending" if self.goal_pending else "none")
        self.get_logger().info(
            f"[hb] {'armed' if self.enabled else 'DISABLED'} | target "
            f"{'fresh' if fresh else 'stale/none'}"
            + (f" {d:.2f} m" if d is not None else "")
            + f" | nav goal {goal}")
        try:
            n = self.get_node_names().count("person_follower")
            if n > 1:
                self.get_logger().error(
                    f"{n} person_follower nodes running! They will preempt each "
                    "other's goals — kill the extras.")
        except Exception:
            pass

    def tick(self):
        if not self.enabled:
            return
        age = (None if self.last_target_s is None
               else self.now_s() - self.last_target_s)
        fresh = age is not None and age < self.lost_timeout_s
        recent = age is not None and age < self.send_max_age_s

        d = self.target_distance()
        if (self.goal_handle is None and not self.goal_pending and recent
                and self.now_s() >= self.next_send_s
                and (d is None or d >= self.min_chase_dist)):
            self.send_goal()
        elif self.goal_handle is not None and not fresh:
            self.cancel_goal(f"person lost > {self.lost_timeout_s:.0f}s")
        elif (self.goal_handle is not None and not recent
                and self.last_target is not None
                and self.now_s() - self.last_update_s >= 1.0):
            # target stale but goal still active: re-stamp the last-seen pose
            # so the BT GoalUpdater never holds an update older than the goal
            # (that mismatch made bt_navigator warn at tick rate)
            self.last_target.header.stamp = self.get_clock().now().to_msg()
            # the rolling global costmap follows the robot: a goal stored
            # in-bounds can be outside by now and Smac would fail it forever
            self.clamp_into_bounds(self.last_target)
            self.pub_update.publish(self.last_target)
            self.last_update_s = self.now_s()

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
        self.pub_update.publish(goal.pose)  # GoalUpdater cache >= goal stamp
        self.last_update_s = self.now_s()
        d = self.target_distance()
        self.status(
            f"sending follow goal ({goal.pose.pose.position.x:.2f}, "
            f"{goal.pose.pose.position.y:.2f})"
            + (f", person {d:.2f} m away" if d is not None else ""))

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
        try:
            st = fut.result().status
        except Exception:
            st = -1
        names = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
        if st == 6:
            self.next_send_s = self.now_s() + self.abort_backoff_s
        self.status(f"follow goal ended: {names.get(st, f'status {st}')}")

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
