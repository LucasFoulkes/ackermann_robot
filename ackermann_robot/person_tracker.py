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
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from ament_index_python.packages import get_package_share_directory
from ackermann_robot.leg_classifier import LegClassifier


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
        self.leg_conf = 0.5              # EWMA of classifier P(leg)
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

    def update(self, xy, sigma_obs, sep=None, conf=None):
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
        if conf is not None:
            self.leg_conf = 0.9 * self.leg_conf + 0.1 * float(conf)

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
    def gait_alternations(self):
        """Sign reversals of significant leg-separation change. Walking opens
        and closes the legs every step; the slow width drift of a static
        object edge seen from a moving robot is monotonic."""
        n, direction, ref = 0, 0, None
        for s in self.sep_hist:
            if ref is None:
                ref = s
                continue
            d = s - ref
            if abs(d) < 0.04:
                continue
            new_dir = 1 if d > 0 else -1
            if direction != 0 and new_dir != direction:
                n += 1
            direction = new_dir
            ref = s
        return n

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
        self.fg_margin = float(p("foreground_margin", 0.2))
        self.fg_search_beams = int(p("foreground_search_beams", 3))
        # tracking
        self.sigma_obs = float(p("sigma_obs", 0.20))              # leg_tracker
        self.sigma_acc = float(p("sigma_acc", 0.45))               # m/s^2 walking
        self.gate_dist = float(p("association_gate", 0.35))
        self.merge_dist = float(p("merge_dist", 0.35))
        self.target_pub_max_unseen_s = float(p("target_pub_max_unseen_s", 0.6))
        self.min_range = float(p("min_range", 0.30))
        self.confirm_travel = float(p("confirm_travel_m", 0.5))   # leg_tracker
        self.confirm_straightness = float(p("confirm_straightness", 0.6))
        self.confirm_speed_min = float(p("confirm_speed_min", 0.25))
        self.gait_osc_min = float(p("gait_osc_min", 0.07))
        self.still_defect_s = float(p("still_defect_s", 5.0))
        self.drop_after_s = float(p("drop_after_s", 1.5))
        self.drop_confirmed_after_s = float(p("drop_confirmed_after_s", 6.0))
        self.drop_pos_std = float(p("drop_pos_std", 2.0))
        self.confirm_robot_lin_max = float(p("confirm_robot_lin_max", 0.08))
        self.confirm_robot_ang_max = float(p("confirm_robot_ang_max", 0.15))
        # enrollment (the default): the target is whoever stands in the front
        # cone, like every shipped follow-me robot. auto_confirm re-enables
        # the old confirm-arbitrary-walkers gates.
        self.enroll_mode = bool(p("enroll_mode", True))
        self.auto_confirm = bool(p("auto_confirm", False))
        self.enroll_dist_min = float(p("enroll_dist_min", 0.4))
        self.enroll_dist_max = float(p("enroll_dist_max", 2.5))
        self.enroll_half_angle = float(p("enroll_half_angle_rad", 0.5))
        self.enroll_min_hits = int(p("enroll_min_hits", 5))
        self.enroll_min_travel = float(p("enroll_min_travel", 0.25))
        self.leg_conf_create_min = float(p("leg_conf_create_min", 0.15))
        self.leg_conf_enroll_min = float(p("leg_conf_enroll_min", 0.35))
        default_forest = os.path.join(
            get_package_share_directory("ackermann_robot"), "config",
            "trained_leg_detector_res=0.33.yaml")
        self.clf = LegClassifier(str(p("leg_forest_file", default_forest)))
        self.reid_window_s = float(p("reid_window_s", 3.0))
        self.reid_dist = float(p("reid_dist", 0.75))
        self.confirm_free_occ_max = float(p("confirm_free_occ_max", 10.0))
        self.max_speed = float(p("max_person_speed", 3.0))
        # static-map veto
        self.use_map_veto = bool(p("use_map_veto", True))
        self.debug = bool(p("debug", True))
        self.veto_occ = int(p("map_veto_occupancy", 65))

        self.fixed_frame = str(p("fixed_frame", "odom"))
        self.tracks = []
        self.dead_people = []            # (id, xy, drop_time) for re-id
        self.target_id = None
        self.map_msg = None
        self.last_scan_s = None
        self.last_gate_log_s = 0.0
        self.robot_pose_prev = None      # (x, y, yaw, t) for self-motion
        self.robot_lin = 0.0
        self.robot_ang = 0.0

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

        sub_scan = self.create_subscription(LaserScan, "/scan", self.on_scan,
                                            qos_profile_sensor_data)
        self.get_logger().info(f"listening on {sub_scan.topic_name}")
        map_qos = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)

        self.create_service(Trigger, "/follow/enroll", self.on_enroll_srv)
        self.create_timer(5.0, self.check_duplicates)
        self.pub_markers = self.create_publisher(MarkerArray, "/people", 5)
        self.pub_poses = self.create_publisher(PoseArray, "/people_poses", 5)
        self.pub_target = self.create_publisher(PoseStamped, "/person_target", 5)
        self.get_logger().info(
            f"person_tracker: legs {self.leg_w_min}-{self.leg_w_max} m, pair "
            f"<= {self.pair_dist} m, "
            + (f"ENROLL mode (stand {self.enroll_dist_min}-"
               f"{self.enroll_dist_max} m in front to be followed)"
               if self.enroll_mode else
               f"auto-confirm after {self.confirm_travel} m of travel")
            + f", map veto {'on' if self.use_map_veto else 'off'}, leg "
            f"classifier {'LOADED' if self.clf.ok else 'MISSING (neutral 0.5)'}")

    def on_map(self, msg):
        self.map_msg = msg

    # --- detection -------------------------------------------------------

    def clusters_from_scan(self, msg):
        r = np.asarray(msg.ranges, dtype=np.float32)
        n = len(r)
        ang = msg.angle_min + msg.angle_increment * np.arange(n)
        valid = (np.isfinite(r) & (r >= max(msg.range_min, self.min_range))
                 & (r <= self.max_range))
        idx = np.where(valid)[0]
        if len(idx) < self.min_points:
            return []
        xs = r[idx] * np.cos(ang[idx])
        ys = r[idx] * np.sin(ang[idx])
        pts = np.stack([xs, ys], axis=1)
        gaps = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        breaks = np.where(gaps > self.jump_dist)[0]
        parts = np.split(np.arange(len(idx)), breaks + 1)
        # the scan wraps at +-pi: merge first/last clusters if they touch
        if len(parts) > 1:
            if np.linalg.norm(pts[parts[0][0]] - pts[parts[-1][-1]]) <= self.jump_dist:
                parts[0] = np.concatenate([parts[-1], parts[0]])
                parts.pop()
        out = []
        for part in parts:
            if len(part) < self.min_points:
                continue
            p_ = pts[part]
            width = float(np.linalg.norm(p_[0] - p_[-1]))
            # classifier P(leg): only score leg/blob-width clusters (walls
            # are dropped by the width gate anyway; saves ~0.8 ms each)
            conf = (self.clf.score(p_, idx[part], r, msg.range_min,
                                   msg.range_max)
                    if width <= self.blob_w_max else 0.0)
            # foreground flag gates track CREATION only -- a person's legs
            # brushing past furniture must still UPDATE their track
            out.append((p_, self.foreground_ok(r, idx, part), conf))
        return out

    def foreground_ok(self, r, idx, part):
        """True if the cluster stands in FRONT of its surroundings: the first
        valid beam just outside each end must be deeper by fg_margin (or give
        no return). Real legs always pass (floor/wall behind them); slivers
        of background revealed past a moving occluder -- the phantom 'people'
        that appear whenever the robot or a person moves -- have a nearer
        neighbor on at least one side and fail."""
        n = len(r)
        cr = float(np.median(r[idx[part]]))
        for step, end in ((-1, int(idx[part[0]])), (1, int(idx[part[-1]]))):
            for k in range(1, self.fg_search_beams + 1):
                v = r[(end + step * k) % n]
                if not np.isfinite(v) or v <= 0.0:
                    continue          # no return = open space = clear side
                if v - cr <= self.fg_margin:
                    return False
                break
        return True

    def leg_candidates(self, clusters):
        legs, blobs, blob_w = [], [], []
        leg_fg, blob_fg = [], []
        leg_cf, blob_cf = [], []
        for c, fg, conf in clusters:
            width = float(np.linalg.norm(c[0] - c[-1]))
            centroid = c.mean(axis=0)
            if self.leg_w_min <= width <= self.leg_w_max:
                legs.append(centroid)
                leg_fg.append(fg)
                leg_cf.append(conf)
            elif self.leg_w_max < width <= self.blob_w_max:
                blobs.append(centroid)   # possibly two legs close together
                blob_fg.append(fg)
                blob_cf.append(conf)
                blob_w.append(width)
        return legs, leg_fg, leg_cf, blobs, blob_fg, blob_cf, blob_w

    def pair_candidates(self, legs, leg_fg, leg_cf, blobs, blob_fg, blob_cf,
                        blob_w):
        """Person candidates (pair midpoints + merged-legs blobs) and the
        leftover single legs. Singles can UPDATE an existing track (feet
        apart / one leg occluded) but never create one — keeps clutter out
        while stopping track dropouts that made the marker coast away."""
        cands, seps, cand_fg, cand_cf = [], [], [], []
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
                cand_fg.append(leg_fg[i] and leg_fg[best_j])
                cand_cf.append(0.5 * (leg_cf[i] + leg_cf[best_j]))
        cands.extend(blobs)
        seps.extend(blob_w)
        cand_fg.extend(blob_fg)
        cand_cf.extend(blob_cf)
        singles = [legs[i] for i in range(len(legs)) if i not in used]
        singles_cf = [leg_cf[i] for i in range(len(legs)) if i not in used]
        return cands, seps, cand_fg, cand_cf, singles, singles_cf

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

    def free_space_ok(self, xy_fixed, stamp):
        """Mean occupancy of the 5x5 map neighborhood must be low for a track
        to CONFIRM as a person. People walk through free space; parallax
        phantoms hug mapped obstacles. Gates confirmation only (never
        updates), and unknown/unmapped space is allowed."""
        m = self.map_msg
        if m is None:
            return True
        tf_map = self.tf2d(m.header.frame_id, self.fixed_frame, stamp)
        if tf_map is None:
            return True
        pt = self.apply2d(tf_map, [xy_fixed])[0]
        res = m.info.resolution
        col = int((pt[0] - m.info.origin.position.x) / res)
        row = int((pt[1] - m.info.origin.position.y) / res)
        vals = []
        for dr in range(-2, 3):
            for dc in range(-2, 3):
                r_, c_ = row + dr, col + dc
                if 0 <= r_ < m.info.height and 0 <= c_ < m.info.width:
                    v = m.data[r_ * m.info.width + c_]
                    if v >= 0:
                        vals.append(v)
        if not vals:
            return True
        return (sum(vals) / len(vals)) < self.confirm_free_occ_max

    def gate_report(self, t, stamp):
        """(failed_gate_names, one-line metrics) for the confirm decision."""
        checks = [
            ("travel", t.travelled >= self.confirm_travel,
             f"trav {t.travelled:.2f}"),
            ("hits", t.hits >= 8, f"hits {t.hits}"),
            ("straight", t.straightness >= self.confirm_straightness,
             f"str {t.straightness:.2f}"),
            ("gait", t.gait_osc >= self.gait_osc_min,
             f"osc {t.gait_osc:.2f}"),
            ("alt", t.gait_alternations >= 2,
             f"alt x{t.gait_alternations}"),
            ("speed", self.confirm_speed_min <= t.speed <= self.max_speed,
             f"spd {t.speed:.2f}"),
            ("freespace", self.free_space_ok(t.xy, stamp), "occ"),
            ("legconf", t.leg_conf >= self.leg_conf_enroll_min,
             f"conf {t.leg_conf:.2f}"),
            ("robotstill",
             self.robot_lin <= self.confirm_robot_lin_max
             and abs(self.robot_ang) <= self.confirm_robot_ang_max,
             f"bot {self.robot_lin:.2f}/{self.robot_ang:.2f}"),
        ]
        fails = [name for name, passed, _ in checks if not passed]
        detail = " ".join(d for _, _, d in checks)
        return fails, detail

    def try_enroll(self, stamp):
        """Lock onto the nearest stable track in the front cone. Runs every
        scan while no target is enrolled; the robot must be (near) still so
        a drive-by chair pair cannot enroll itself."""
        if (self.robot_lin > self.confirm_robot_lin_max
                or abs(self.robot_ang) > self.confirm_robot_ang_max):
            return
        tf_b = self.tf2d("base_link", self.fixed_frame, stamp)
        if tf_b is None:
            return
        best, best_d = None, None
        for t in self.tracks:
            if t.hits < self.enroll_min_hits:
                continue
            bx, by = self.apply2d(tf_b, [t.xy])[0]
            d = math.hypot(bx, by)
            if not (self.enroll_dist_min <= d <= self.enroll_dist_max):
                continue
            if abs(math.atan2(by, bx)) > self.enroll_half_angle:
                continue
            # walking into the cone is the enrollment signature: furniture
            # tracks never displace; a standing person gets absorbed into the
            # SLAM map (so a map check wrongly rejects them -- learned 17:38)
            if t.travelled < self.enroll_min_travel:
                continue
            if t.leg_conf < self.leg_conf_enroll_min:
                continue   # walked, but doesn't look like legs to the forest
            if best is None or d < best_d:
                best, best_d = t, d
        if best is None:
            self.get_logger().info(
                f"no target: WALK into the zone {self.enroll_dist_min:.1f}-"
                f"{self.enroll_dist_max:.1f} m in front of the robot to "
                "enroll", throttle_duration_sec=10.0)
            return
        best.confirmed = True
        self.target_id = best.id
        self.get_logger().info(
            f"ENROLLED person {best.id} at {best_d:.2f} m in front "
            f"(leg conf {best.leg_conf:.2f}) -- following")

    def check_duplicates(self):
        try:
            n = self.get_node_names().count("person_tracker")
        except Exception:
            return
        if n > 1:
            self.get_logger().error(
                f"{n} person_tracker nodes running! Orphans from old launches "
                "publish conflicting /person_target -- kill the extras "
                "(pgrep -af person_tracker).")

    def on_enroll_srv(self, req, res):
        for t in self.tracks:
            t.confirmed = False
        self.dead_people.clear()
        self.target_id = None
        res.success = True
        res.message = (f"re-enrolling: stand {self.enroll_dist_min:.1f}-"
                       f"{self.enroll_dist_max:.1f} m in front of the robot")
        self.get_logger().info(res.message)
        return res

    # --- main loop ---------------------------------------------------------

    def on_scan(self, msg):
        now_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.1 if self.last_scan_s is None else max(0.01, min(0.5, now_s - self.last_scan_s))
        self.last_scan_s = now_s

        (legs, leg_fg, leg_cf, blobs, blob_fg, blob_cf,
         blob_w) = self.leg_candidates(self.clusters_from_scan(msg))

        tf_fix = self.tf2d(self.fixed_frame, msg.header.frame_id, msg.header.stamp)
        if tf_fix is None:
            return  # no odom TF yet

        # robot self-motion: every phantom confirm in field runs happened
        # while the robot was driving (occlusion edges sweep through free
        # space with walking-like motion). Track it to gate confirmation.
        tf_b = self.tf2d(self.fixed_frame, "base_link", msg.header.stamp)
        if tf_b is not None:
            bx, by, byaw = tf_b[0], tf_b[1], math.atan2(tf_b[3], tf_b[2])
            if self.robot_pose_prev is not None:
                px, py, pyaw, pt = self.robot_pose_prev
                rdt = max(0.02, now_s - pt)
                dyaw = math.atan2(math.sin(byaw - pyaw), math.cos(byaw - pyaw))
                # low-pass: TF jitter must not look like motion
                self.robot_lin = (0.6 * self.robot_lin
                                  + 0.4 * math.hypot(bx - px, by - py) / rdt)
                self.robot_ang = (0.6 * self.robot_ang
                                  + 0.4 * abs(dyaw) / rdt)
            self.robot_pose_prev = (bx, by, byaw, now_s)
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
                leg_fg = [f for f, k in zip(leg_fg, keep_l) if k]
                blob_fg = [f for f, k in zip(blob_fg, keep_b) if k]
                leg_cf = [f for f, k in zip(leg_cf, keep_l) if k]
                blob_cf = [f for f, k in zip(blob_cf, keep_b) if k]
                legs = [l for l, k in zip(legs, keep_l) if k]
                blobs = [b for b, k in zip(blobs, keep_b) if k]
                blob_w = [w for w, k in zip(blob_w, keep_b) if k]

        cands, cand_seps, cand_fg, cand_cf, singles, singles_cf = \
            self.pair_candidates(legs, leg_fg, leg_cf, blobs, blob_fg,
                                 blob_cf, blob_w)
        singles = singles + vetoed
        if self.debug:
            self.get_logger().info(
                f"legs {n_legs_pre}->{len(legs)} blobs {n_blobs_pre}->{len(blobs)} "
                f"(map veto) | cands {len(cands)} singles {len(singles)} "
                f"tracks {len(self.tracks)} "
                f"confirmed {sum(1 for t in self.tracks if t.confirmed)}",
                throttle_duration_sec=2.0)

        cands_all = [np.asarray(c) for c in cands]
        singles_all = [np.asarray(s) for s in singles]

        # predict, associate (greedy NN), update
        for t in self.tracks:
            t.predict(dt, self.sigma_acc)
        unmatched = list(range(len(cands)))
        for t in sorted(self.tracks, key=lambda t: (not t.confirmed, -t.hits)):  # confirmed first: clutter tracks must not steal the target's candidate
            if not unmatched:
                break
            ds = [float(np.linalg.norm(cands[i] - t.xy)) for i in unmatched]
            k = int(np.argmin(ds))
            gate = (self.gate_dist + 1.5 * dt if t.confirmed
                    else self.gate_dist + min(t.speed, 1.2) * dt)
            if ds[k] <= gate:
                t.update(cands[unmatched[k]], self.sigma_obs,
                         cand_seps[unmatched[k]], cand_cf[unmatched[k]])
                t.last_seen_s = now_s
                unmatched.pop(k)
        # second pass: lone legs may update tracks that got no pair this scan
        free_singles = list(range(len(singles)))
        for t in sorted(self.tracks, key=lambda t: (not t.confirmed, -t.hits)):  # confirmed first: clutter tracks must not steal the target's candidate
            if not free_singles or t.last_seen_s == now_s:
                continue
            ds = [float(np.linalg.norm(singles[i] - t.xy)) for i in free_singles]
            k = int(np.argmin(ds))
            gate = (self.gate_dist + 1.5 * dt if t.confirmed
                    else self.gate_dist + min(t.speed, 1.2) * dt)
            if ds[k] <= gate:
                # a lone leg sits ~half a stance off the person center --
                # treat as a LOW-WEIGHT observation or the target pose hops
                # left-leg/right-leg at scan rate (0.4 m/s phantom motion
                # measured on a standing person, bag 192543)
                t.update(singles[free_singles[k]], self.sigma_obs * 2.5,
                         conf=singles_cf[free_singles[k]])
                t.last_seen_s = now_s
                free_singles.pop(k)
        # the single most important diagnostic: WHY did the target miss
        for t in self.tracks:
            if not t.confirmed or t.last_seen_s == now_s:
                continue
            gate = self.gate_dist + 1.5 * dt
            dc = min((float(np.linalg.norm(c - t.xy)) for c in cands_all),
                     default=float("inf"))
            ds_ = min((float(np.linalg.norm(s - t.xy)) for s in singles_all),
                      default=float("inf"))
            self.get_logger().info(
                f"target {t.id} NO UPDATE: nearest cand {dc:.2f} m, nearest "
                f"single {ds_:.2f} m, gate {gate:.2f} m "
                f"({len(cands_all)} cands/{len(singles_all)} singles, "
                f"track spd {t.speed:.2f})",
                throttle_duration_sec=1.0)

        # merge duplicates: a fresh track that forms on top of a confirmed
        # person steals their updates and the confirmed track starves (seen
        # as the enrolled id coasting while a new id rides the person)
        confirmed = [t for t in self.tracks if t.confirmed]
        if confirmed:
            merged = []
            for t in self.tracks:
                if t.confirmed:
                    merged.append(t)
                    continue
                near = min(confirmed,
                           key=lambda c: float(np.linalg.norm(c.xy - t.xy)))
                if float(np.linalg.norm(near.xy - t.xy)) <= self.merge_dist:
                    if t.last_seen_s >= near.last_seen_s:
                        # absorb as a MEASUREMENT -- teleporting the track to
                        # the duplicate's raw centroid made the marker jump
                        # between legs/blob and blew up the speed estimate
                        near.update(np.asarray(t.xy), self.sigma_obs * 2.0)
                        near.last_seen_s = t.last_seen_s
                    continue
                merged.append(t)
            self.tracks = merged

        # coasting tracks: damp velocity so the marker cannot glide away
        for t in self.tracks:
            if t.last_seen_s != now_s:
                t.x[2:] *= 0.85
        self.dead_people = [d for d in self.dead_people
                            if now_s - d[2] <= self.reid_window_s]
        for i in unmatched:
            if not cand_fg[i]:
                continue   # occluded-edge clusters may update, never create
            if cand_cf[i] < self.leg_conf_create_min:
                continue   # classifier says "not legs" -- no track
            t = Track(cands[i], now_s, self.sigma_obs)
            t.leg_conf = cand_cf[i]
            # re-id: a confirmed person who briefly vanished (occlusion, scan
            # dropout) comes back as the same id, already confirmed
            for k, (pid, xy, ts) in enumerate(self.dead_people):
                if float(np.linalg.norm(np.asarray(cands[i]) - xy)) <= self.reid_dist:
                    t.id, t.confirmed = pid, True
                    self.dead_people.pop(k)
                    self.get_logger().info(
                        f"person {pid} re-identified after {now_s - ts:.1f}s")
                    break
            self.tracks.append(t)

        # lifecycle: confirm by travel, drop stale or implausible
        kept = []
        for t in self.tracks:
            age = now_s - t.last_seen_s
            limit = self.drop_confirmed_after_s if t.confirmed else self.drop_after_s
            # coasting diffuses P; drop once the estimate is too uncertain to
            # chase rather than blindly trusting it for the full time limit
            pos_std = math.sqrt(max(0.0, float(t.P[0, 0] + t.P[1, 1])))
            diffuse = t.confirmed and age > 0.5 and pos_std > self.drop_pos_std
            if age > limit or t.speed > self.max_speed or diffuse:
                if t.confirmed:
                    why = (f"speed {t.speed:.1f} m/s" if t.speed > self.max_speed
                           else f"unseen {age:.1f}s, pos std {pos_std:.2f} m")
                    self.get_logger().info(f"person {t.id} dropped ({why})")
                    self.dead_people.append((t.id, t.xy.copy(), now_s))
                continue
            if (self.auto_confirm and not t.confirmed
                    and t.travelled >= self.confirm_travel
                    and t.hits >= 8
                    and t.straightness >= self.confirm_straightness
                    and t.gait_osc >= self.gait_osc_min
                    and t.gait_alternations >= 2
                    and self.confirm_speed_min <= t.speed <= self.max_speed
                    and self.free_space_ok(t.xy, msg.header.stamp)
                    and self.robot_lin <= self.confirm_robot_lin_max
                    and abs(self.robot_ang) <= self.confirm_robot_ang_max):
                t.confirmed = True
                self.get_logger().info(
                    f"person {t.id} confirmed at ({t.xy[0]:.2f}, {t.xy[1]:.2f}) "
                    f"odom (travelled {t.travelled:.2f} m, straightness "
                    f"{t.straightness:.2f}, {t.speed:.2f} m/s, gait "
                    f"{t.gait_osc:.2f} m x{t.gait_alternations}, "
                    f"hits {t.hits})")
            if t.speed < 0.25:
                if t.still_since_s is None:
                    t.still_since_s = now_s
            else:
                t.still_since_s = None
            kept.append(t)
        self.tracks = kept

        if self.enroll_mode and not any(t.confirmed for t in self.tracks):
            self.try_enroll(msg.header.stamp)

        # decision log: which gates each serious track passes/fails. This is
        # the ground truth for tuning false/missed person detections.
        if self.debug and now_s - self.last_gate_log_s >= 2.0:
            self.last_gate_log_s = now_s
            lines = []
            for t in sorted(self.tracks, key=lambda t: -t.hits)[:6]:
                if t.hits < 5:
                    continue
                fails, detail = self.gate_report(t, msg.header.stamp)
                verdict = ("PERSON" if t.confirmed
                           else ("CONFIRMABLE" if not fails
                                 else "no: " + ",".join(fails)))
                lines.append(f"  id {t.id} ({t.xy[0]:.2f},{t.xy[1]:.2f}) "
                             f"{detail} -> {verdict}")
            if lines:
                self.get_logger().info("track gates:\n" + "\n".join(lines))

        self.publish(msg.header.stamp)

    # --- output ------------------------------------------------------------

    def publish(self, stamp):
        stamp_s = stamp.sec + stamp.nanosec * 1e-9
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
            # transparent = coasting on no data (ghost), solid = seen now
            m.color.a = 0.8 if stamp_s - t.last_seen_s <= 0.3 else 0.25
            ma.markers.append(m)
            txt = Marker()
            txt.header = m.header
            txt.ns, txt.id, txt.type = "ids", t.id, Marker.TEXT_VIEW_FACING
            txt.pose.position.x, txt.pose.position.y = pose.position.x, pose.position.y
            txt.pose.position.z = 1.4
            txt.scale.z = 0.25
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            txt.text = f"id {t.id}  {t.speed:.1f} m/s  p{t.leg_conf:.2f}"
            ma.markers.append(txt)
        self.pub_poses.publish(pa)
        self.pub_markers.publish(ma)

        # target choice: sticky, but prefer moving people. Defect from a
        # target that has stood still > still_defect_s when another is moving.
        now_s = stamp_s
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
        if (target is not None
                and now_s - target.last_seen_s <= self.target_pub_max_unseen_s):
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
