#!/usr/bin/env python3
"""Depth -> floor-free LaserScan, gravity-aligned (default) or RANSAC fallback.

GRAVITY method (default, ~5-10x cheaper than RANSAC, the 2026-06-13 CPU fix):
  base_link is held level by the IMU-fed EKF (two_d_mode), and base_link->camera
  is a static URDF TF -- so transforming the cloud into base_link makes "height
  above floor" simply the z coordinate. Floor height is the low percentile of z
  (O(n)); obstacles are the [min_h, max_h] band above it. No plane SEARCH, and it
  can't be fooled into locking onto a wall/ramp the way RANSAC can.
  This works precisely because we HAVE a fused IMU; most RealSense pipelines
  don't, which is why they fall back to RANSAC.

RANSAC method (method:=ransac): the original plane-fit, kept as a fallback.

Either way, obstacle points in a height band are published as a LaserScan in
base_link so Nav2's local costmap uses the stable odom->base_link TF chain.
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

        self.method = str(p("method", "gravity")).lower()  # "gravity" | "ransac"
        self.cloud_topic = p("cloud_topic", "/camera/camera/depth/color/points")
        self.scan_frame = p("scan_frame", "base_link")
        self.max_points = int(p("max_points", 7000))
        self.iters = int(p("ransac_iterations", 60))
        self.thresh = float(p("ransac_distance", 0.02))
        self.min_inliers = int(p("min_floor_inliers", 600))
        self.floor_seed_min_y = float(p("floor_seed_min_y", 0.05))
        self.floor_seed_min_z = float(p("floor_seed_min_z", 0.20))
        # GRAVITY method: floor z = this percentile of base_link z (robust to a
        # few stray low points); sanity-gate the estimate to a plausible band.
        self.floor_pctile = float(p("floor_percentile", 5.0))
        self.floor_z_min = float(p("floor_z_min", -0.10))
        self.floor_z_max = float(p("floor_z_max", 0.10))
        # Temporal floor filter (2026-07-03): the raw per-frame percentile
        # wandered -0.008..-0.080 m within seconds (accelerations tilt the
        # IMU gravity direction; scene content shifts the percentile), and
        # every dip pushed real floor above min_obstacle_height -> "floor is
        # an obstacle". The chassis-to-floor distance is physically near-
        # constant, so: slow EMA + single-frame jump rejection; the obstacle
        # threshold always uses the FILTERED estimate.
        self.floor_alpha = float(p("floor_filter_alpha", 0.08))
        self.floor_jump_gate = float(p("floor_jump_gate", 0.03))
        self._floor_filt = None
        self._floor_boot = []
        self._floor_rejects = []
        # plausible chassis-to-floor band (base_link z of true floor):
        self.floor_plaus_min = float(p("floor_plausible_min", -0.12))
        self.floor_plaus_max = float(p("floor_plausible_max", 0.02))
        self.min_h = float(p("min_obstacle_height", 0.06))
        self.max_h = float(p("max_obstacle_height", 0.195))
        self.angle_min = float(p("angle_min", -1.0))
        self.angle_max = float(p("angle_max", 1.0))
        self.angle_inc = float(p("angle_increment", 0.0075))
        self.range_min = float(p("range_min", 0.15))
        self.range_max = float(p("range_max", 6.0))
        # D435i depth noise clusters near range_max -> phantom costmap ring if marked.
        # Keep range_max for inf clear rays; never MARK obstacles beyond max_mark_range.
        self.max_mark_range = float(p("max_mark_range", 1.65))
        self.far_arc_ratio = float(p("far_arc_ratio", 0.75))
        self.far_arc_neighbor_m = float(p("far_arc_neighbor_m", 0.12))
        self.ring_spread_max = float(p("ring_spread_max", 0.18))
        self.ring_min_coverage = float(p("ring_min_coverage", 0.30))
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
        except Exception:
            import traceback
            self.get_logger().error(
                "depth_floor_scan failed:\n" + traceback.format_exc())

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

        if self.method == "gravity":
            self._process_gravity(msg, sample)
        else:
            self._process_ransac(msg, sample)

    def _process_gravity(self, msg, sample):
        """Transform cloud into level base_link; floor = low-percentile z; obstacles
        = [min_h, max_h] above it. No plane search (the CPU win)."""
        body_all = self._to_scan_frame(sample, msg.header.frame_id)
        if body_all is None:
            return
        z = body_all[:, 2]
        raw_floor = float(np.percentile(z, self.floor_pctile))
        if not (self.floor_z_min <= raw_floor <= self.floor_z_max):
            self.get_logger().warn(
                f"gravity: floor z {raw_floor:.3f} m outside [{self.floor_z_min}, "
                f"{self.floor_z_max}] -- bad TF or robot tilted; skipping",
                throttle_duration_sec=2.0)
            self._publish_empty()
            return
        # Robust temporal filter: bootstrap with a median, then slow EMA with
        # single-frame jump rejection — PLUS divergence recovery: if raw
        # disagrees with the filter for many consecutive frames, the filter
        # is the wrong one (first deploy latched at a bad startup bootstrap
        # of -0.135 while raw sat at -0.05, and the gate rejected truth
        # forever -> whole floor became "obstacle"). Re-bootstrap from the
        # recent rejected frames instead of trusting the latch.
        if self._floor_filt is None:
            self._floor_boot.append(raw_floor)
            if len(self._floor_boot) >= 5:
                self._floor_filt = float(np.median(self._floor_boot))
            floor_z = raw_floor
        else:
            if abs(raw_floor - self._floor_filt) <= self.floor_jump_gate:
                self._floor_filt += self.floor_alpha * (raw_floor - self._floor_filt)
                self._floor_rejects = []
            else:
                self._floor_rejects.append(raw_floor)
                if len(self._floor_rejects) >= 10:   # ~2.5 s of consistent disagreement
                    new = float(np.median(self._floor_rejects))
                    # Re-bootstrap ONLY to physically plausible floor heights.
                    # First deploy chased pitch transients (hard accel tips
                    # the chassis, raw slid -0.05 -> -0.21 within seconds,
                    # 2026-07-04 11:28) down to impossible depths, dropping
                    # the obstacle threshold onto real floor. The chassis-to-
                    # floor distance can only live in a narrow band.
                    if self.floor_plaus_min <= new <= self.floor_plaus_max:
                        self.get_logger().warn(
                            f"floor filter diverged (held {self._floor_filt:.3f}, "
                            f"reality ~{new:.3f}) -- re-bootstrapping")
                        self._floor_filt = new
                    else:
                        self.get_logger().warn(
                            f"floor filter: ignoring implausible re-bootstrap "
                            f"target {new:.3f} (pitch transient?)",
                            throttle_duration_sec=10.0)
                    self._floor_rejects = []
            floor_z = self._floor_filt
        self.pub_dist.publish(Float32(data=float(-floor_z)))
        self.pub_tilt.publish(Vector3(x=0.0, y=0.0, z=0.0))  # base_link is leveled
        self.get_logger().info(
            f"floor(grav): z={floor_z:.3f} m (raw {raw_floor:.3f}), "
            f"{sample.shape[0]} pts",
            throttle_duration_sec=2.0)
        haf = z - floor_z
        body = body_all[(haf >= self.min_h) & (haf <= self.max_h)]
        self._emit_scan(msg, body)

    def _process_ransac(self, msg, sample):
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

        haf = sample @ n + d
        obst = sample[(haf >= self.min_h) & (haf <= self.max_h)]
        body = self._to_scan_frame(obst, msg.header.frame_id) if obst.shape[0] else \
            np.empty((0, 3))
        if obst.shape[0] and body is None:
            return
        self._emit_scan(msg, body)

    def _emit_scan(self, msg, body):
        """Project base_link-frame obstacle points to a LaserScan and publish.
        body: Nx3 points already in scan_frame (base_link). Shared by both methods."""
        # Stamp the scan with the cloud's CAPTURE time, not processing time, so the
        # costmap can time-correct it (camera->base_link is static, so transforming
        # the points at latest TF is still valid). Using 'now' made stale obstacles
        # look current on a moving robot -> ghosts.
        now = msg.header.stamp
        if body is None or body.shape[0] == 0:
            self._publish_scan(now, np.full(self.n_bins, np.inf))
            return

        xy = body[:, :2]
        bearing = np.arctan2(xy[:, 1], xy[:, 0])
        dist = np.hypot(xy[:, 0], xy[:, 1])

        mark_max = min(self.range_max, self.max_mark_range)
        valid = (bearing >= self.angle_min) & (bearing < self.angle_max) \
            & (dist >= self.range_min) & (dist <= mark_max)
        bearing, dist = bearing[valid], dist[valid]

        ranges = np.full(self.n_bins, np.inf)
        if bearing.size:
            bins = ((bearing - self.angle_min) / self.angle_inc).astype(int)
            bins = np.clip(bins, 0, self.n_bins - 1)
            np.minimum.at(ranges, bins, dist)

        ranges = self._suppress_range_ring(ranges)
        ranges = self._filter_far_arc_dots(ranges)
        self._publish_scan(now, ranges)
        h = Header(stamp=now, frame_id=self.scan_frame)
        self.pub_obst.publish(pc2.create_cloud_xyz32(h, body.astype(np.float32)))

    def _suppress_range_ring(self, ranges):
        """Drop equidistant far marks (D435i noise / Nav2 inf-at-range_max artifact)."""
        arr = np.asarray(ranges, dtype=np.float64)
        finite = arr[np.isfinite(arr) & (arr > self.range_min + 0.05)]
        if finite.size < self.n_bins * self.ring_min_coverage:
            return ranges
        spread = float(np.percentile(finite, 90) - np.percentile(finite, 10))
        med = float(np.median(finite))
        if spread <= self.ring_spread_max and med >= self.range_min + 0.35:
            self.get_logger().warn(
                f"suppressing camera scan ring: median={med:.2f} m spread={spread:.2f} m",
                throttle_duration_sec=2.0,
            )
            return np.full(self.n_bins, np.inf)
        return ranges

    def _filter_far_arc_dots(self, ranges):
        """Drop isolated far hits on the depth FOV edge (dotted front arc on costmap)."""
        arr = np.asarray(ranges, dtype=np.float64)
        out = arr.copy()
        threshold = self.max_mark_range * self.far_arc_ratio
        for i in range(self.n_bins):
            if not np.isfinite(arr[i]) or arr[i] <= threshold:
                continue
            neighbors = 0
            for j in (i - 2, i - 1, i + 1, i + 2):
                if 0 <= j < self.n_bins and np.isfinite(arr[j]):
                    if abs(arr[j] - arr[i]) <= self.far_arc_neighbor_m:
                        neighbors += 1
            if neighbors < 1:
                out[i] = np.inf
        return out

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
