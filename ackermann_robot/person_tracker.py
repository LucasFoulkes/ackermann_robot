#!/usr/bin/env python3
"""Lidar person tracker: leg-pair detection + motion-confirmed Kalman tracks.

Pipeline (parameters from leg_tracker / Leigh et al. ICRA 2015 defaults):
  /scan -> jump-distance clustering -> leg-width gate -> static-map veto
        -> pair legs (or merged-legs blob) -> person candidates (odom frame)
        -> NN-associated constant-velocity Kalman tracks
        -> confirmed once track has travelled >= confirm_travel_m
Confirmed tracks stay people while tracked (standing still is fine after
lock-on; confirmation requires walking once). Tracks live in the odom frame
so the robot's own motion does not look like target motion.

Outputs:
  /people        MarkerArray (RViz: cylinder per confirmed person + id + vel)
  /people_poses  PoseArray of confirmed people (odom frame)
  /person_target PoseStamped of the followed person (nearest confirmed;
                 sticky to the same track id while it survives)
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Track:
    _next_id = 0

    def __init__(self, xy, now_s, sigma_obs):
        self.id = Track._next_id
        Track._next_id += 1
        self.path_len = 0.0
        self.prev_xy = np.array(xy)
        self.x = np.array([xy[0], xy[1], 0.0, 0.0])
        self.P = np.diag([sigma_obs**2, sigma_obs**2, 1.0, 1.0])
        self.birth_xy = np.array(xy)
        self.last_seen_s = now_s
        self.confirmed = False
        self.hits = 1
        self.still_since_s = None
        self.sep_hist = []               # observed leg separation / blob width

    def predict(self, dt, sigma_acc):
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        # white-acceleration process noise
        q11 = 0.25 * dt**4
        q13 = 0.5 * dt**3
        q33 = dt**2
        Q = sigma_acc**2 * np.array([
            [q11, 0, q13, 0],
            [0, q11, 0, q13],
            [q13, 0, q33, 0],
            [0, q13, 0, q33]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, xy, sigma_obs, sep=None):
        H = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])
        R = np.eye(2) * sigma_obs**2
        y = np.asarray(xy) - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P
        self.hits += 1
        self.path_len += float(np.linalg.norm(self.x[:2] - self.prev_xy))
        self.prev_xy = self.x[:2].copy()
        if sep is not None:
            self.sep_hist.append(float(sep))
            if len(self.sep_hist) > 40:
                self.sep_hist.pop(0)

    @property
    def xy(self):
        return self.x[:2]

    @property
    def speed(self):
        return float(np.hypot(self.x[2], self.x[3]))

    @property
    def travelled(self):
        return float(np.linalg.norm(self.x[:2] - self.birth_xy))

    @property
    def straightness(self):
        """Displacement / path length: ~1 for walking, low for cluster jitter."""
        return self.travelled / self.path_len if self.path_len > 0.05 else 0.0

    @property
    def gait_osc(self):
        """Leg-separation oscillation (m). Walking legs swing apart/together
        every step; static pairs (gap edges, posts) keep constant spacing."""
        if len(self.sep_hist) < 5:
            return 0.0
        return max(self.sep_hist) - min(self.sep_hist)


class PersonTracker(Node):
    def __init__(self):
        super().__init__("person_tracker")
        p = lambda n, d: self.declare_parameter(n, d).value

        # clustering / leg gates (leg_tracker defaults where noted)
        self.jump_dist = float(p("cluster_jump_dist", 0.13))      # leg_tracker
        self.min_points = int(p("cluster_min_points", 3))         # leg_tracker
        self.leg_w_min = float(p("leg_width_min", 0.03))
        self.leg_w_max = float(p("leg_width_max", 0.25))
        self.blob_w_max = float(p("merged_legs_width_max", 0.40))  # feet together
        self.max_range = float(p("max_range", 5.0))
        self.pair_dist = float(p("max_leg_pairing_dist", 0.45))   # single-scan stance width
        # tracking
        self.sigma_obs = float(p("sigma_obs", 0.10))              # leg_tracker
        self.sigma_acc = float(p("sigma_acc", 1.0))               # m/s^2 walking
        self.gate_dist = float(p("association_gate", 0.35))
        self.confirm_travel = float(p("confirm_travel_m", 0.5))   # leg_tracker
        self.confirm_straightness = float(p("confirm_straightness", 0.6))
        self.confirm_speed_min = float(p("confirm_speed_min", 0.25))
        self.gait_osc_min = float(p("gait_osc_min", 0.07))
        self.still_defect_s = float(p("still_defect_s", 5.0))
        self.drop_after_s = float(p("drop_after_s", 1.5))
        self.drop_confirmed_after_s = float(p("drop_confirmed_after_s", 3.0))
        self.max_speed = float(p("max_person_speed", 3.0))
        # static-map veto
        self.use_map_veto = bool(p("use_map_veto", True))
        self.debug = bool(p("debug", True))
        self.veto_occ = int(p("map_veto_occupancy", 65))

        self.fixed_frame = str(p("fixed_frame", "odom"))
        self.tracks = []
        self.target_id = None
        self.map_msg = None
        self.last_scan_s = None

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

        self.create_subscription(LaserScan, "/scan", self.on_scan,
                                 qos_profile_sensor_data)
        map_qos = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)

        self.pub_markers = self.create_publisher(MarkerArray, "/people", 5)
        self.pub_poses = self.create_publisher(PoseArray, "/people_poses", 5)
        self.pub_target = self.create_publisher(PoseStamped, "/person_target", 5)
        self.get_logger().info(
            f"person_tracker: legs {self.leg_w_min}-{self.leg_w_max} m, pair "
            f"<= {self.pair_dist} m, confirm after {self.confirm_travel} m of "
            f"travel, map veto {'on' if self.use_map_veto else 'off'}")

    def on_map(self, msg):
        self.map_msg = msg

    # --- detection -------------------------------------------------------

    def clusters_from_scan(self, msg):
        r = np.asarray(msg.ranges, dtype=np.float32)
        n = len(r)
        ang = msg.angle_min + msg.angle_increment * np.arange(n)
        valid = np.isfinite(r) & (r >= msg.range_min) & (r <= self.max_range)
        idx = np.where(valid)[0]
        if len(idx) < self.min_points:
            return []
        xs = r[idx] * np.cos(ang[idx])
        ys = r[idx] * np.sin(ang[idx])
        pts = np.stack([xs, ys], axis=1)
        gaps = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        breaks = np.where(gaps > self.jump_dist)[0]
        clusters = np.split(pts, breaks + 1)
        # the scan wraps at +-pi: merge first/last clusters if they touch
        if len(clusters) > 1:
            if np.linalg.norm(clusters[0][0] - clusters[-1][-1]) <= self.jump_dist:
                clusters[0] = np.vstack([clusters[-1], clusters[0]])
                clusters.pop()
        return [c for c in clusters if len(c) >= self.min_points]

    def leg_candidates(self, clusters):
        legs, blobs, blob_w = [], [], []
        for c in clusters:
            width = float(np.linalg.norm(c[0] - c[-1]))
            centroid = c.mean(axis=0)
            if self.leg_w_min <= width <= self.leg_w_max:
                legs.append(centroid)
            elif self.leg_w_max < width <= self.blob_w_max:
                blobs.append(centroid)   # possibly two legs close together
                blob_w.append(width)
        return legs, blobs, blob_w

    def pair_candidates(self, legs, blobs, blob_w):
        """Person candidates (pair midpoints + merged-legs blobs) and the
        leftover single legs. Singles can UPDATE an existing track (feet
        apart / one leg occluded) but never create one — keeps clutter out
        while stopping track dropouts that made the marker coast away."""
        cands, seps = [], []
        used = set()
        for i in range(len(legs)):
            if i in used:
                continue
            best_j, best_d = None, self.pair_dist
            for j in range(i + 1, len(legs)):
                if j in used:
                    continue
                d = float(np.linalg.norm(legs[i] - legs[j]))
                if d < best_d:
                    best_j, best_d = j, d
            if best_j is not None:
                used.add(i)
                used.add(best_j)
                cands.append((legs[i] + legs[best_j]) / 2.0)
                seps.append(best_d)
        cands.extend(blobs)
        seps.extend(blob_w)
        singles = [legs[i] for i in range(len(legs)) if i not in used]
        return cands, seps, singles

    # --- frames / map veto -------------------------------------------------

    def tf2d(self, target, source, stamp):
        try:
            t = self.tf_buf.lookup_transform(target, source, stamp,
                                             rclpy.duration.Duration(seconds=0.1))
        except Exception:
            try:
                t = self.tf_buf.lookup_transform(target, source, rclpy.time.Time())
            except Exception:
                return None
        tr = t.transform.translation
        yaw = yaw_of(t.transform.rotation)
        return (tr.x, tr.y, math.cos(yaw), math.sin(yaw))

    @staticmethod
    def apply2d(tf, pts):
        x0, y0, c, s = tf
        out = []
        for px, py in pts:
            out.append(np.array([x0 + c * px - s * py, y0 + s * px + c * py]))
        return out

    def on_static_map(self, xy_map):
        """True if this map-frame point lies on a mapped obstacle (1 cell)."""
        m = self.map_msg
        res = m.info.resolution
        col = int((xy_map[0] - m.info.origin.position.x) / res)
        row = int((xy_map[1] - m.info.origin.position.y) / res)
        if 0 <= row < m.info.height and 0 <= col < m.info.width:
            return m.data[row * m.info.width + col] >= self.veto_occ
        return False

    # --- main loop ---------------------------------------------------------

    def on_scan(self, msg):
        now_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.1 if self.last_scan_s is None else max(0.01, min(0.5, now_s - self.last_scan_s))
        self.last_scan_s = now_s

        legs, blobs, blob_w = self.leg_candidates(self.clusters_from_scan(msg))

        tf_fix = self.tf2d(self.fixed_frame, msg.header.frame_id, msg.header.stamp)
        if tf_fix is None:
            return  # no odom TF yet
        legs = self.apply2d(tf_fix, legs)
        blobs = self.apply2d(tf_fix, blobs)

        # veto each leg/blob on the static map BEFORE pairing: edges of mapped
        # objects (walls, pots) otherwise pair across a small gap and the pair
        # midpoint sits in free space where a midpoint-veto cannot see it
        n_legs_pre, n_blobs_pre = len(legs), len(blobs)
        vetoed = []
        if self.use_map_veto and self.map_msg is not None and (legs or blobs):
            tf_map = self.tf2d(self.map_msg.header.frame_id, self.fixed_frame,
                               msg.header.stamp)
            if tf_map is not None:
                keep_l = [not self.on_static_map(m)
                          for m in self.apply2d(tf_map, legs)]
                keep_b = [not self.on_static_map(m)
                          for m in self.apply2d(tf_map, blobs)]
                # vetoed points may still UPDATE an existing track — slam
                # absorbs a lingering person into the map, which otherwise
                # starves + drops their confirmed track — but never create one
                vetoed = ([l for l, k in zip(legs, keep_l) if not k]
                          + [b for b, k in zip(blobs, keep_b) if not k])
                legs = [l for l, k in zip(legs, keep_l) if k]
                blobs = [b for b, k in zip(blobs, keep_b) if k]
                blob_w = [w for w, k in zip(blob_w, keep_b) if k]

        cands, cand_seps, singles = self.pair_candidates(legs, blobs, blob_w)
        singles = singles + vetoed
        if self.debug:
            self.get_logger().info(
                f"legs {n_legs_pre}->{len(legs)} blobs {n_blobs_pre}->{len(blobs)} "
                f"(map veto) | cands {len(cands)} singles {len(singles)} "
                f"tracks {len(self.tracks)} "
                f"confirmed {sum(1 for t in self.tracks if t.confirmed)}",
                throttle_duration_sec=2.0)

        # predict, associate (greedy NN), update
        for t in self.tracks:
            t.predict(dt, self.sigma_acc)
        unmatched = list(range(len(cands)))
        for t in sorted(self.tracks, key=lambda t: -t.hits):
            if not unmatched:
                break
            ds = [float(np.linalg.norm(cands[i] - t.xy)) for i in unmatched]
            k = int(np.argmin(ds))
            if ds[k] <= self.gate_dist:
                t.update(cands[unmatched[k]], self.sigma_obs,
                         cand_seps[unmatched[k]])
                t.last_seen_s = now_s
                unmatched.pop(k)
        # second pass: lone legs may update tracks that got no pair this scan
        free_singles = list(range(len(singles)))
        for t in sorted(self.tracks, key=lambda t: -t.hits):
            if not free_singles or t.last_seen_s == now_s:
                continue
            ds = [float(np.linalg.norm(singles[i] - t.xy)) for i in free_singles]
            k = int(np.argmin(ds))
            if ds[k] <= self.gate_dist:
                t.update(singles[free_singles[k]], self.sigma_obs)
                t.last_seen_s = now_s
                free_singles.pop(k)
        # coasting tracks: damp velocity so the marker cannot glide away
        for t in self.tracks:
            if t.last_seen_s != now_s:
                t.x[2:] *= 0.85
        for i in unmatched:
            self.tracks.append(Track(cands[i], now_s, self.sigma_obs))

        # lifecycle: confirm by travel, drop stale or implausible
        kept = []
        for t in self.tracks:
            age = now_s - t.last_seen_s
            limit = self.drop_confirmed_after_s if t.confirmed else self.drop_after_s
            if age > limit or t.speed > self.max_speed:
                if t.confirmed:
                    why = (f"unseen {age:.1f}s" if age > limit
                           else f"speed {t.speed:.1f} m/s")
                    self.get_logger().info(f"person {t.id} dropped ({why})")
                continue
            if (not t.confirmed and t.travelled >= self.confirm_travel
                    and t.hits >= 8
                    and t.straightness >= self.confirm_straightness
                    and t.gait_osc >= self.gait_osc_min
                    and self.confirm_speed_min <= t.speed <= self.max_speed):
                t.confirmed = True
                self.get_logger().info(
                    f"person {t.id} confirmed at ({t.xy[0]:.2f}, {t.xy[1]:.2f}) "
                    f"odom (travelled {t.travelled:.2f} m, straightness "
                    f"{t.straightness:.2f}, {t.speed:.2f} m/s, gait "
                    f"{t.gait_osc:.2f} m, hits {t.hits})")
            if t.speed < 0.25:
                if t.still_since_s is None:
                    t.still_since_s = now_s
            else:
                t.still_since_s = None
            kept.append(t)
        self.tracks = kept

        self.publish(msg.header.stamp)

    # --- output ------------------------------------------------------------

    def publish(self, stamp):
        people = [t for t in self.tracks if t.confirmed]

        pa = PoseArray()
        pa.header.stamp = stamp
        pa.header.frame_id = self.fixed_frame
        ma = MarkerArray()
        wipe = Marker()
        wipe.action = Marker.DELETEALL
        ma.markers.append(wipe)
        for t in people:
            pose = Pose()
            pose.position.x, pose.position.y = float(t.xy[0]), float(t.xy[1])
            yaw = math.atan2(t.x[3], t.x[2]) if t.speed > 0.1 else 0.0
            pose.orientation.z = math.sin(yaw / 2)
            pose.orientation.w = math.cos(yaw / 2)
            pa.poses.append(pose)

            m = Marker()
            m.header.frame_id = self.fixed_frame
            m.header.stamp = stamp
            m.ns, m.id, m.type = "people", t.id, Marker.CYLINDER
            m.pose = pose
            m.pose.position.z = 0.6
            m.scale.x = m.scale.y = 0.3
            m.scale.z = 1.2
            if t.id == self.target_id:
                m.color.r, m.color.g, m.color.b = 1.0, 0.1, 0.1   # target = RED
            elif t.speed < 0.25:
                m.color.r = m.color.g = m.color.b = 0.5   # standing = gray
            else:
                import colorsys
                hue = (t.id * 0.618) % 1.0                # distinct per id
                m.color.r, m.color.g, m.color.b = colorsys.hsv_to_rgb(hue, 0.9, 0.95)
            m.color.a = 0.8
            ma.markers.append(m)
            txt = Marker()
            txt.header = m.header
            txt.ns, txt.id, txt.type = "ids", t.id, Marker.TEXT_VIEW_FACING
            txt.pose.position.x, txt.pose.position.y = pose.position.x, pose.position.y
            txt.pose.position.z = 1.4
            txt.scale.z = 0.25
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            txt.text = f"id {t.id}  {t.speed:.1f} m/s"
            ma.markers.append(txt)
        self.pub_poses.publish(pa)
        self.pub_markers.publish(ma)

        # target choice: sticky, but prefer moving people. Defect from a
        # target that has stood still > still_defect_s when another is moving.
        now_s = stamp.sec + stamp.nanosec * 1e-9
        cur = next((t for t in people if t.id == self.target_id), None)
        moving = [t for t in people if t.speed >= 0.25]
        bx = by = None
        tf_base = self.tf2d(self.fixed_frame, "base_link", stamp)
        if tf_base is not None:
            bx, by = tf_base[0], tf_base[1]
        dist = (lambda t: np.hypot(t.xy[0] - bx, t.xy[1] - by)) if bx is not None             else (lambda t: 0.0)
        target = cur
        cur_idle = (cur is not None and cur.still_since_s is not None
                    and now_s - cur.still_since_s > self.still_defect_s)
        others_moving = [t for t in moving if cur is None or t.id != cur.id]
        if (cur is None or cur_idle) and others_moving:
            target = min(others_moving, key=dist)
        elif cur is None and people:
            target = min(people, key=dist)
        new_id = target.id if target is not None else None
        if new_id != self.target_id:
            if new_id is None:
                self.get_logger().info("target lost (no confirmed people)")
            else:
                self.get_logger().info(
                    f"target -> person {new_id} (dist {dist(target):.2f} m, "
                    f"speed {target.speed:.2f} m/s)")
        self.target_id = new_id
        if target is not None:
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = self.fixed_frame
            ps.pose.position.x = float(target.xy[0])
            ps.pose.position.y = float(target.xy[1])
            yaw = math.atan2(target.x[3], target.x[2]) if target.speed > 0.1 else 0.0
            ps.pose.orientation.z = math.sin(yaw / 2)
            ps.pose.orientation.w = math.cos(yaw / 2)
            self.pub_target.publish(ps)


def main():
    rclpy.init()
    node = PersonTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
