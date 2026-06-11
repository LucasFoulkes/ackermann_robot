#!/usr/bin/env python3
"""Enrollment shim: leg_tracker_ros2 people -> /person_target.

leg_tracker_ros2 (Leigh ICRA'15) does detection + tracking; this node only
decides WHO to follow: the first tracked person inside the front cone
(their tracker already requires 0.5 m of paired-leg travel, so only walkers
exist). Sticky to that id until lost; auto re-enrolls; /follow/enroll
forces a re-lock. Also republishes all people as /people_poses for
scan_mask (costmap/odometry person masking).
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from leg_tracker_ros2.msg import PersonArray


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class EnrollShim(Node):
    def __init__(self):
        super().__init__("enroll_shim")
        p = lambda n, d: self.declare_parameter(n, d).value
        self.dist_min = float(p("enroll_dist_min", 0.4))
        self.dist_max = float(p("enroll_dist_max", 2.5))
        self.half_angle = float(p("enroll_half_angle_rad", 0.5))
        self.lost_timeout = float(p("lost_timeout_s", 6.0))

        self.target_id = None
        self.target_last_s = 0.0

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.create_subscription(PersonArray, "people_tracked",
                                 self.on_people, 10)
        self.pub_target = self.create_publisher(PoseStamped, "/person_target", 5)
        self.pub_people = self.create_publisher(PoseArray, "/people_poses", 5)
        self.create_service(Trigger, "/follow/enroll", self.on_enroll_srv)
        self.get_logger().info(
            f"enroll_shim: WALK into the zone {self.dist_min}-{self.dist_max} m "
            "in front of the robot to be followed")

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_enroll_srv(self, req, res):
        self.target_id = None
        res.success = True
        res.message = "re-enrolling: walk into the front zone"
        self.get_logger().info(res.message)
        return res

    def in_base(self, pose, frame):
        try:
            t = self.tf_buf.lookup_transform("base_link", frame,
                                             rclpy.time.Time())
        except Exception:
            return None
        tr = t.transform.translation
        yaw = yaw_of(t.transform.rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        x, y = pose.position.x, pose.position.y
        return (tr.x + c * x - s * y, tr.y + s * x + c * y)

    def on_people(self, msg):
        pa = PoseArray()
        pa.header = msg.header
        pa.poses = [q.pose for q in msg.people]
        self.pub_people.publish(pa)

        target = next((q for q in msg.people if q.id == self.target_id), None)
        if target is None and self.target_id is not None:
            if self.now_s() - self.target_last_s > self.lost_timeout:
                self.get_logger().info(
                    f"person {self.target_id} lost -- walk into the front "
                    "zone to re-enroll")
                self.target_id = None

        if self.target_id is None:
            best, best_d = None, None
            for q in msg.people:
                b = self.in_base(q.pose, msg.header.frame_id)
                if b is None:
                    continue
                d = math.hypot(*b)
                if not (self.dist_min <= d <= self.dist_max):
                    continue
                if abs(math.atan2(b[1], b[0])) > self.half_angle:
                    continue
                if best is None or d < best_d:
                    best, best_d = q, d
            if best is not None:
                self.target_id = best.id
                self.get_logger().info(
                    f"ENROLLED person {best.id} at {best_d:.2f} m in front "
                    "-- following")
                target = best

        if target is not None:
            self.target_last_s = self.now_s()
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = target.pose
            self.pub_target.publish(ps)


def main():
    rclpy.init()
    node = EnrollShim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
