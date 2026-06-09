#!/usr/bin/env python3
"""Depth -> floor-free LaserScan via RANSAC ground-plane segmentation.

Subscribes to the D435i organized point cloud, fits the floor with RANSAC, and
publishes obstacle points in a height band as a LaserScan in base_link so Nav2's
local costmap can use the stable odom->base_link TF chain.
"""
import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time

from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Float32, Header
from geometry_msgs.msg import Vector3
from tf2_ros import Buffer, TransformListener


def ransac_ground(pts, iters, thresh, rng):
    """Return (normal, d, inlier_mask) for the largest plane n.p + d = 0."""
    n_pts = len(pts)
    best_mask = None
    best_count = 0
    for _ in range(iters):
        idx = rng.integers(0, n_pts, size=3)
        p0, p1, p2 = pts[idx]
        n = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n = n / norm
        d = -np.dot(n, p0)
        dist = np.abs(pts @ n + d)
        mask = dist < thresh
        count = int(mask.sum())
        if count > best_count:
            best_count, best_mask = count, mask
    if best_mask is None:
        return None
    inl = pts[best_mask]
    c = inl.mean(axis=0)
    _, _, vt = np.linalg.svd(inl - c, full_matrices=False)
    n = vt[-1]
    n = n / np.linalg.norm(n)
    d = -np.dot(n, c)
    return n, d, best_mask


def quat_to_matrix(qx, qy, qz, qw):
    """Quaternion (x,y,z,w) -> 3x3 rotation matrix."""
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ])


class DepthFloorScan(Node):
    def __init__(self):
        super().__init__("depth_floor_scan")
        p = lambda n, d: self.declare_parameter(n, d).value

        self.cloud_topic = p("cloud_topic", "/camera/camera/depth/color/points")
        self.scan_frame = p("scan_frame", "base_link")
        self.max_points = int(p("max_points", 7000))
        self.iters = int(p("ransac_iterations", 60))
        self.thresh = float(p("ransac_distance", 0.02))
        self.min_inliers = int(p("min_floor_inliers", 600))
        self.floor_seed_min_y = float(p("floor_seed_min_y", 0.05))
        self.floor_seed_min_z = float(p("floor_seed_min_z", 0.20))
        self.min_h = float(p("min_obstacle_height", 0.02))
        self.max_h = float(p("max_obstacle_height", 0.195))
        self.angle_min = float(p("angle_min", -1.0))
        self.angle_max = float(p("angle_max", 1.0))
        self.angle_inc = float(p("angle_increment", 0.0075))
        self.range_min = float(p("range_min", 0.15))
        self.range_max = float(p("range_max", 6.0))
        process_hz = float(p("process_hz", 8.0))
        self.max_floor_tilt = float(p("max_floor_tilt_deg", 20.0))
        self.floor_h_min = float(p("floor_height_min_m", 0.08))
        self.floor_h_max = float(p("floor_height_max_m", 0.22))

        self.rng = np.random.default_rng(0)
        self.latest = None
        self.n_bins = max(1, int(round((self.angle_max - self.angle_min) / self.angle_inc)))

        self.pub_scan = self.create_publisher(LaserScan, p("scan_topic", "/camera/scan"), 5)
        self.pub_dist = self.create_publisher(Float32, "/camera/floor/distance", 5)
        self.pub_tilt = self.create_publisher(Vector3, "/camera/floor/tilt", 5)
        self.pub_obst = self.create_publisher(PointCloud2, "/camera/floor/obstacles", 5)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        cloud_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PointCloud2, self.cloud_topic, self._on_cloud, cloud_qos)
        self.create_timer(1.0 / process_hz, self._process)
        self.get_logger().info(
            f"depth_floor_scan up: {self.cloud_topic} -> {self.scan_frame} scan"
        )

    def _on_cloud(self, msg):
        self.latest = msg

    def _process(self):
        try:
            self._process_cloud()
        except Exception as exc:
            self.get_logger().error(f"depth_floor_scan failed: {exc}")

    def _floor_ransac_sample(self, sample):
        """Bias RANSAC toward the floor, not the nearest wall."""
        mask = (sample[:, 1] > self.floor_seed_min_y) & (sample[:, 2] > self.floor_seed_min_z)
        floor_pts = sample[mask]
        return floor_pts if floor_pts.shape[0] >= self.min_inliers else sample

    def _to_scan_frame(self, pts, source_frame):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.scan_frame, source_frame, Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception as exc:
            self.get_logger().warn(
                f"TF {source_frame} -> {self.scan_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return None
        tr = tf.transform.translation
        q = tf.transform.rotation
        rot = quat_to_matrix(q.x, q.y, q.z, q.w)
        trans = np.array([tr.x, tr.y, tr.z])
        return pts @ rot.T + trans

    def _process_cloud(self):
        msg = self.latest
        if msg is None:
            return

        xyz = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        if xyz.shape[0] < self.min_inliers:
            return
        xyz = xyz.astype(np.float64)

        if xyz.shape[0] > self.max_points:
            sel = self.rng.choice(xyz.shape[0], self.max_points, replace=False)
            sample = xyz[sel]
        else:
            sample = xyz

        fit = ransac_ground(self._floor_ransac_sample(sample), self.iters, self.thresh, self.rng)
        if fit is None or int(fit[2].sum()) < self.min_inliers:
            self.get_logger().warn("no floor plane found", throttle_duration_sec=2.0)
            self._publish_empty()
            return
        n, d, _ = fit

        if d < 0.0:
            n, d = -n, -d
        height = d

        roll = math.degrees(math.atan2(n[0], -n[1]))
        pitch = math.degrees(math.atan2(n[2], -n[1]))
        total = math.degrees(math.acos(max(-1.0, min(1.0, -n[1]))))

        if total > self.max_floor_tilt or not (self.floor_h_min <= height <= self.floor_h_max):
            self.get_logger().warn(
                f"rejecting non-floor plane: height {height:.3f} m, tilt {total:.1f} deg",
                throttle_duration_sec=2.0,
            )
            self._publish_empty()
            return

        self.pub_dist.publish(Float32(data=float(height)))
        self.pub_tilt.publish(Vector3(x=float(roll), y=float(pitch), z=float(total)))
        self.get_logger().info(
            f"floor: {height:.3f} m | roll {roll:+.1f}  pitch {pitch:+.1f}  tilt {total:.1f} deg",
            throttle_duration_sec=1.0,
        )

        haf = xyz @ n + d
        obst = xyz[(haf >= self.min_h) & (haf <= self.max_h)]
        # Stamp the scan with the cloud's CAPTURE time, not processing time, so the
        # costmap can time-correct it (camera->base_link is static, so transforming
        # the points at latest TF is still valid). Using 'now' made stale obstacles
        # look current on a moving robot -> ghosts.
        now = msg.header.stamp
        if obst.shape[0] == 0:
            self._publish_scan(now, np.full(self.n_bins, np.inf))
            return

        body = self._to_scan_frame(obst, msg.header.frame_id)
        if body is None:
            return

        xy = body[:, :2]
        bearing = np.arctan2(xy[:, 1], xy[:, 0])
        dist = np.hypot(xy[:, 0], xy[:, 1])

        valid = (bearing >= self.angle_min) & (bearing < self.angle_max) \
            & (dist >= self.range_min) & (dist <= self.range_max)
        bearing, dist = bearing[valid], dist[valid]

        ranges = np.full(self.n_bins, np.inf)
        if bearing.size:
            bins = ((bearing - self.angle_min) / self.angle_inc).astype(int)
            bins = np.clip(bins, 0, self.n_bins - 1)
            np.minimum.at(ranges, bins, dist)

        self._publish_scan(now, ranges)
        h = Header(stamp=now, frame_id=self.scan_frame)
        self.pub_obst.publish(pc2.create_cloud_xyz32(h, body.astype(np.float32)))

    def _publish_empty(self):
        self._publish_scan(self.get_clock().now().to_msg(), np.full(self.n_bins, np.inf))

    def _publish_scan(self, stamp, ranges):
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.scan_frame
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_inc
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = [float(r) for r in ranges]
        self.pub_scan.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = DepthFloorScan()
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
