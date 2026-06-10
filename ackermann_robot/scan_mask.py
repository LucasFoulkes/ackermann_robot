#!/usr/bin/env python3
"""Mask the followed person out of the scans that feed the costmaps.

The person being chased must never be a costmap obstacle: their legs mark
lethal cost at the goal ("no valid path", RPP collision aborts) and beside
the robot once close ("Start occupied"), and the marks ghost-trail as they
walk. Beams hitting within mask_radius of any confirmed person (from
/people_poses) become NaN -- the obstacle layers ignore them entirely.

Lives in the DRIVE stack: when the follow stack is down or poses go stale,
scans pass through unmasked.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class ScanMask(Node):
    def __init__(self):
        super().__init__("scan_mask")
        p = lambda n, d: self.declare_parameter(n, d).value
        self.mask_radius = float(p("mask_radius", 0.5))
        self.max_pose_age = float(p("max_pose_age_s", 1.5))

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.people = None
        self.people_s = 0.0

        self.create_subscription(PoseArray, "/people_poses", self.on_people, 5)
        for src, dst in (("/scan", "/scan_nav"),
                         ("/camera/scan", "/camera/scan_nav")):
            pub = self.create_publisher(LaserScan, dst, qos_profile_sensor_data)
            self.create_subscription(LaserScan, src,
                                     lambda m, pub=pub: self.on_scan(m, pub),
                                     qos_profile_sensor_data)
        self.get_logger().info(
            f"scan_mask: /scan,/camera/scan -> *_nav; confirmed people "
            f"masked within {self.mask_radius} m")

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_people(self, msg):
        self.people = msg
        self.people_s = self.now_s()

    def person_points(self, frame):
        """Confirmed-person positions in the scan frame, [] when stale."""
        if (self.people is None or not self.people.poses
                or self.now_s() - self.people_s > self.max_pose_age):
            return []
        try:
            t = self.tf_buf.lookup_transform(frame, self.people.header.frame_id,
                                             rclpy.time.Time())
        except Exception:
            return []
        tr = t.transform.translation
        yaw = yaw_of(t.transform.rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        return [(tr.x + c * q.position.x - s * q.position.y,
                 tr.y + s * q.position.x + c * q.position.y)
                for q in self.people.poses]

    def on_scan(self, msg, pub):
        pts = self.person_points(msg.header.frame_id)
        if pts:
            r = np.asarray(msg.ranges, dtype=np.float32)
            ang = msg.angle_min + msg.angle_increment * np.arange(len(r))
            masked = np.zeros(len(r), dtype=bool)
            for px, py in pts:
                d = math.hypot(px, py)
                if d < 1e-3:
                    continue
                half = math.atan2(self.mask_radius, d) + abs(msg.angle_increment)
                dang = np.angle(np.exp(1j * (ang - math.atan2(py, px))))
                masked |= ((np.abs(dang) <= half)
                           & (r >= d - self.mask_radius)
                           & (r <= d + self.mask_radius))
            if masked.any():
                r = r.copy()
                r[masked] = float("nan")
                msg.ranges = r.tolist()
        pub.publish(msg)


def main():
    rclpy.init()
    node = ScanMask()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
