#!/usr/bin/env python3
"""Stage-1 person detector from 2D lidar (legs), odom-frame native.

Pipeline per scan (design discussed 2026-07-13):
  1. Transform scan points into the odom frame (robot pose from /odom +
     lidar extrinsic from the birth certificate). Static world stays put
     regardless of ego-motion; person velocity is true odom-frame velocity.
  2. Persistence grid (5 cm cells, odom frame): cells occupied consistently
     for background_promote_s become STATIC background; cells that were
     observed free (beam passed through) and are now occupied are DYNAMIC
     evidence. Cells owned by an active track never get absorbed into the
     background (a long-standing person must not become furniture).
  3. Foreground = returns in non-static cells. Cluster by range jump,
     score clusters as legs (width/count/range gates), pair legs into
     person candidates (or accept one wide merged-legs cluster).
  4. Constant-velocity Kalman track per person, nearest-neighbour
     association. A track is CONFIRMED once its legs have travelled
     min_travel_confirm_m together (motion is the signature — a static
     pair of returns is indistinguishable from chair legs). Tentative
     tracks are still published so a standing person is visible.
  5. Output: MarkerArray (yellow = tentative, green = confirmed, text =
     id + speed) and PoseArray of confirmed+tentative people.

Honest limits of stage 1: a person already standing there at boot never
confirms (yellow forever) until they move ~0.4 m; raw scans are not
deskewed (~4 cm smear at 0.4 m/s — tolerable vs 12 cm legs).
"""

import json
import math
import os

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Track:
    _next_id = 1

    def __init__(self, x, y, stamp):
        self.id = Track._next_id
        Track._next_id += 1
        self.state = np.array([x, y, 0.0, 0.0])          # x y vx vy
        self.cov = np.diag([0.05, 0.05, 0.25, 0.25])
        self.last_update = stamp
        self.born = stamp
        self.origin = np.array([x, y])
        self.last_moving = stamp
        self.travel = 0.0        # max NET displacement from birth position:
        self.confirmed = False   # summed increments would accumulate jitter
        self.missed = 0          # (~0.2 m/s of phantom travel at 10 Hz)
        self.confirm_travel = None   # None = use the global parameter
        self.shadowed_until = 0.0    # occlusion-suspect horizon
        self.person_score = 0.0      # neural referee EMA (0 = furniture)
        self.moving_time = 0.0       # cumulative seconds at walking speed

    def predict(self, dt):
        f = np.eye(4)
        f[0, 2] = f[1, 3] = dt
        # Velocity process noise sized for human acceleration (~1 m/s^2),
        # not for chasing per-stride centroid swings.
        q = np.diag([0.01, 0.01, 0.06, 0.06]) * dt
        self.state = f @ self.state
        self.cov = f @ self.cov @ f.T + q
        if self.missed > 0:
            # Unobserved: halve velocity each coasting frame so the track
            # brakes to a stop instead of projecting the last stride
            # forever (classic CV-Kalman runaway).
            self.state[2:] *= 0.5

    def update(self, zx, zy, stamp, trust_motion=True):
        h = np.zeros((2, 4))
        h[0, 0] = h[1, 1] = 1.0
        # The 'measurement' is the leg centroid: gait swings it 10-20 cm
        # per stride and single-leg occlusion jumps it to one leg. Trusting
        # it at +-3 cm made tracks visibly jumpy; +-12 cm averages strides.
        r = np.eye(2) * 0.12 ** 2
        innovation = np.array([zx, zy]) - h @ self.state
        # Teleport gate (2026-07-15, chairs-around-a-table): cluster
        # association SLIDES across furniture legs as a person walks
        # past — the track 'travels' without anything moving and a chair
        # inherits a confirmed identity. A live track cannot jump more
        # than a walker moves in one frame; the allowance grows with
        # missed frames so a fast walker re-acquires after occlusion.
        # 0.20: tighter than chair-leg spacing (~0.3-0.5 m) and looser
        # than a walker's per-frame motion (~0.15 m at 10 Hz) — slides
        # across furniture are rejected, humans are not.
        if float(np.linalg.norm(innovation)) > 0.20 + 0.15 * self.missed:
            self.missed += 1
            return False
        s = h @ self.cov @ h.T + r
        k = self.cov @ h.T @ np.linalg.inv(s)
        self.state = self.state + k @ innovation
        self.cov = (np.eye(4) - k @ h) @ self.cov
        if trust_motion:
            if self.speed > 0.30:
                self.moving_time += min(0.3, max(0.0,
                                                 stamp - self.last_update))
            # Ego-motion attribution gate (2026-07-15): during hard robot
            # maneuvers, ICP frame micro-shifts displace STATIC clutter
            # coherently in the odom frame — ghosts earned 'moving'
            # status and the follower ping-ponged between phantoms.
            # Position tracking always runs; motion EVIDENCE (travel,
            # last_moving) accrues only while the ego frame is calm.
            self.travel = max(self.travel, float(
                np.linalg.norm(self.state[:2] - self.origin)))
            # 0.30, not 0.15: ghost jitter reaches Kalman speed 0.22
            # (measured p90, 21:17 bag) and endlessly refreshed
            # 'recently moving' — the staleness clock never fired and a
            # ghost could hold followership forever. 0.30 = walking.
            if self.speed > 0.30:
                self.last_moving = stamp
        self.last_update = stamp
        self.missed = 0
        return True

    @property
    def speed(self):
        return float(np.hypot(self.state[2], self.state[3]))


class PersonTracker(Node):
    def __init__(self):
        super().__init__('person_tracker')
        defaults = {
            'birth_certificate_path': (
                '~/ros2_ws/src/ackermann_robot/ackermann_robot/config/'
                'birth_certificate.yaml'),
            'cell_m': 0.05,
            # Cell occupied this long (and seen in most scans) -> furniture.
            'background_promote_s': 4.0,
            'max_person_range_m': 5.0,
            'cluster_jump_m': 0.13,
            'leg_width_min_m': 0.03,
            'leg_width_max_m': 0.30,
            'leg_pair_min_m': 0.10,
            'leg_pair_max_m': 0.50,
            'merged_width_min_m': 0.20,
            'merged_width_max_m': 0.45,
            'association_gate_m': 0.45,
            'min_travel_confirm_m': 0.40,
            'min_moving_confirm_s': 0.8,
            # Ego calm thresholds: motion evidence and follow-target
            # switching only while the robot itself is below these (the
            # odom frame is stable enough to attribute motion to OTHERS).
            'ego_calm_speed_mps': 0.35,
            'ego_calm_yaw_radps': 0.5,
            'min_track_age_confirm_s': 1.0,
            'track_timeout_s': 2.5,   # coast through occlusions (velocity damps)
            'publish_debug_markers': True,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        self.lidar_x, self.lidar_y, self.lidar_yaw = (
            self._lidar_extrinsic())

        self.pose = None                    # (x, y, yaw) odom frame
        self.robot_speed = 0.0
        self.robot_yaw_rate = 0.0
        self._odom_stamp = None
        self.ego_calm = True
        self.first_scan_stamp = None
        self.last_scan_stamp = None
        self.cells = {}                     # cell -> [first_hit, last_hit,
                                            #          hits, frees, static]
        self.tracks = []
        self.recent_deaths = []       # (x, y, stamp) of dead CONFIRMED tracks
        self.furniture_spots = []     # (x, y, stamp): places where a
        # confirmed track died furniture-shaped (no walking, no referee
        # endorsement). The user's TIME FILTER: places that repeatedly
        # produce non-person confirmations get harder to confirm from.
        self.followed_id = None
        self.last_switch_stamp = 0.0
        self.scan_count = 0

        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, '/scan', self._scan, qos)
        self.create_subscription(Odometry, '/odom', self._odom, 20)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/person_tracker/markers', 5)
        # Structured forensics (2026-07-15): every ghost hunt so far ran
        # on sparse INFO lines. Full per-track state, bagged, makes the
        # next one a table lookup.
        self.debug_pub = self.create_publisher(
            String, '/person_tracker/debug', 5)
        # Neural referee (DR-SPAAM at ~1 Hz): the judgment geometry
        # cannot make — a STATIC human vs static furniture. Benchmarked
        # 2026-07-16: ranks the human 4-10x above ghosts even standing
        # still. Scores are a RANKING signal, not ground truth.
        self.create_subscription(
            String, '/person_referee/detections', self._referee, 5)
        self.people_pub = self.create_publisher(
            PoseArray, '/person_tracker/people', 5)
        # Best single person for the follower: CONFIRMED tracks only
        # (never chase an unconfirmed candidate). Odometry so the follower
        # gets the Kalman velocity too (speed matching).
        self.person_pub = self.create_publisher(
            Odometry, '/person_tracker/person', 5)
        self.get_logger().info('person_tracker up (stage 1: legs + motion '
                               'confirmation, odom frame)')

    def _lidar_extrinsic(self):
        try:
            path = os.path.expanduser(self.p['birth_certificate_path'])
            with open(path) as handle:
                lidar = yaml.safe_load(handle)['lidar']
            return (float(lidar['x_m']), float(lidar['y_m']),
                    float(lidar['yaw_rad']))
        except Exception as error:
            self.get_logger().warning(
                f'birth certificate unavailable ({error}); '
                'using measured defaults')
            return 0.237, 0.0, math.pi

    def _referee(self, msg):
        try:
            data = json.loads(msg.data)
            points = data.get('points', [])
        except (ValueError, TypeError):
            return
        for track in self.tracks:
            best = 0.0
            for x, y, conf in points:
                if math.hypot(track.state[0] - x,
                              track.state[1] - y) < 0.5:
                    best = max(best, float(conf))
            track.person_score += 0.30 * (best - track.person_score)

    def _odom(self, msg):
        pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                yaw_from_quaternion(msg.pose.pose.orientation))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # tf_odom_bridge publishes POSE ONLY — twist is always zero, so
        # reading it left the ego gate blind since birth: the tracker
        # believed the robot was permanently stationary (ego_calm 100%
        # through whole 3-point turns, 22:25 bag) and every maneuver's
        # ICP shift minted 'moving' ghosts unguarded. Derive ego motion
        # from consecutive poses, exactly like the controller does.
        if self.pose is not None and self._odom_stamp is not None:
            dt = stamp - self._odom_stamp
            if 1e-3 < dt < 0.5:
                speed = math.hypot(pose[0] - self.pose[0],
                                   pose[1] - self.pose[1]) / dt
                yaw_rate = abs(math.atan2(
                    math.sin(pose[2] - self.pose[2]),
                    math.cos(pose[2] - self.pose[2]))) / dt
                self.robot_speed += 0.4 * (speed - self.robot_speed)
                self.robot_yaw_rate += 0.4 * (yaw_rate - self.robot_yaw_rate)
        self.pose = pose
        self._odom_stamp = stamp

    # ------------------------------------------------------------- scan --
    def _scan(self, msg):
        if self.pose is None:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.first_scan_stamp is None:
            self.first_scan_stamp = stamp
        self.last_scan_stamp = stamp
        self.scan_count += 1
        self.ego_calm = (self.robot_speed < self.p['ego_calm_speed_mps'] and
                         self.robot_yaw_rate < self.p['ego_calm_yaw_radps'])
        px, py, pyaw = self.pose
        syaw = pyaw + self.lidar_yaw
        sx = px + math.cos(pyaw) * self.lidar_x - math.sin(pyaw) * self.lidar_y
        sy = py + math.sin(pyaw) * self.lidar_x + math.cos(pyaw) * self.lidar_y
        self.sensor_xy = (sx, sy)

        ranges = np.asarray(msg.ranges, dtype=np.float64)
        n = ranges.size
        angles = msg.angle_min + np.arange(n) * msg.angle_increment + syaw
        valid = np.isfinite(ranges) & (ranges > msg.range_min) \
            & (ranges < msg.range_max)
        r = ranges[valid]
        a = angles[valid]
        xs = sx + r * np.cos(a)
        ys = sy + r * np.sin(a)

        self._update_grid(sx, sy, r, a, xs, ys, stamp)
        clusters = self._clusters(xs, ys, r, np.where(valid)[0])
        legs = [c for c in clusters if self._leg_score(c) > 0.0]
        candidates = self._pair_legs(legs)
        self._track(candidates, stamp)
        self._publish(msg.header.stamp, legs, candidates)

    # ------------------------------------------------------------- grid --
    def _cell(self, x, y):
        c = self.p['cell_m']
        return (int(math.floor(x / c)), int(math.floor(y / c)))

    def _update_grid(self, sx, sy, r, a, xs, ys, stamp):
        cell_m = self.p['cell_m']
        # Free-space sampling along each beam (vectorized, 1 sample/cell):
        # a cell seen free that later shows a return is dynamic evidence.
        steps = np.maximum(1, (r / cell_m).astype(int) - 1)
        free_cells = set()
        # Sample at most every other cell to bound cost; adequate evidence.
        for beam in range(0, r.size, 2):
            dists = np.arange(1, steps[beam], 2) * cell_m
            if dists.size == 0:
                continue
            fx = sx + dists * math.cos(a[beam])
            fy = sy + dists * math.sin(a[beam])
            free_cells.update(zip((fx / cell_m).astype(int),
                                  (fy / cell_m).astype(int)))
        # Only CONFIRMED tracks protect their cells from background
        # promotion. Tentative tracks used to as well — so a chair that
        # fooled us once protected its own cells forever and was never
        # learned as furniture (the immortal-ghost loop, 2026-07-16).
        protected = {self._cell(t.state[0] + dx, t.state[1] + dy)
                     for t in self.tracks if t.confirmed
                     for dx in (-0.2, 0.0, 0.2) for dy in (-0.2, 0.0, 0.2)}
        for cell in free_cells:
            entry = self.cells.get(cell)
            if entry is None:
                # Open floor gets an entry too: a person stepping onto
                # never-occupied ground must read was-free-now-occupied.
                # (Without this, detection secretly depended on legs
                # re-crossing their own trail — 110 s blind spells after
                # the robot drove around, 2026-07-14 00:51 session.)
                self.cells[cell] = [stamp, 0.0, 0, 1, False]
            elif entry[4]:
                # Moved furniture heals: sustained free sightings with no
                # fresh hits demote the cell back to unknown, so the spot
                # doesn't stay a person-blind zone forever.
                entry[3] += 1
                if entry[3] > 40 and stamp - entry[1] > 10.0:
                    # Lazy on purpose: grazing beams (near-parallel to a
                    # wall) mark wall cells free every scan while the
                    # wall is briefly unhit — an eager threshold demoted
                    # 15% of legit walls in replay. Real moved furniture
                    # accumulates this in ~5 s of clear line of sight.
                    self.cells.pop(cell, None)
            elif (entry[2] > 0 and entry[3] > 4
                    and stamp - entry[1] > 1.0):
                # Occupied in the past but repeatedly seen free since AND
                # no recent hit: forget the stale occupancy (departed
                # person's trail). The recency guard keeps grazing beams
                # (near-parallel to walls) from wiping cells that are
                # still being hit every scan — that wipe/re-hit cycle
                # made WALLS read as perpetually-fresh dynamic returns.
                self.cells.pop(cell, None)
            else:
                entry[3] = min(entry[3] + 1, 10)
        hit_cells = set(zip((xs / cell_m).astype(int),
                            (ys / cell_m).astype(int)))
        promote = self.p['background_promote_s']
        for cell in hit_cells:
            entry = self.cells.setdefault(cell, [stamp, stamp, 0, 0, False])
            entry[1] = stamp
            entry[2] += 1
            if entry[4]:
                entry[3] = 0        # furniture re-seen: clear demotion
            elif entry[2] > 8 and entry[3] > 0:
                # A cell hit for 8+ scans is a surface, not an arrival:
                # its 'was free' evidence is grazing-beam noise. Without
                # this, wall cells carried free-counts forever and every
                # wall point classified DYNAMIC (blue leg markers on
                # walls, 2026-07-16). A stepping person births within
                # ~5 hits, untouched by this.
                entry[3] = 0
            if (not entry[4] and cell not in protected
                    and stamp - entry[0] > promote
                    and entry[2] > 0.4 * promote * 8):   # ~8 scans/s min
                entry[4] = True                          # static furniture

    def _point_class(self, x, y):
        """2 = dynamic (was free, now occupied), 1 = unknown, 0 = static."""
        cell = self._cell(x, y)
        entry = self.cells.get(cell)
        if entry is None:
            return 1
        if entry[4]:
            return 0
        if entry[3] < 2:
            return 1
        # Free-then-hit ADJACENT to known furniture is edge noise, not
        # an arrival: object-boundary cells oscillate hit/free forever
        # (range jitter + grazing beams), minting phantom dynamic
        # evidence that re-birthed candidates on static furniture every
        # few seconds (live session 2026-07-16 10:45). A walking person
        # crosses open floor; their cells do not hug a static boundary.
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                near = self.cells.get((cell[0] + dx, cell[1] + dy))
                if near is not None and near[4]:
                    return 1
        return 2

    # --------------------------------------------------------- clusters --
    def _clusters(self, xs, ys, r, indices):
        clusters = []
        current = [0]
        jump = self.p['cluster_jump_m']
        for i in range(1, xs.size):
            gap = math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
            contiguous = indices[i] - indices[i - 1] <= 2
            if gap <= jump and contiguous:
                current.append(i)
            else:
                clusters.append(current)
                current = [i]
        clusters.append(current)
        out = []
        for members in clusters:
            if len(members) < 3:
                continue
            member_xs = xs[members]
            member_ys = ys[members]
            width = math.hypot(member_xs[-1] - member_xs[0],
                               member_ys[-1] - member_ys[0])
            classes = [self._point_class(x, y)
                       for x, y in zip(member_xs, member_ys)]
            out.append({
                'x': float(member_xs.mean()), 'y': float(member_ys.mean()),
                'n': len(members), 'width': width,
                'range': float(r[members].mean()),
                'static_ratio': classes.count(0) / len(classes),
                'dynamic_ratio': classes.count(2) / len(classes),
            })
        return out

    def _leg_score(self, cluster):
        if cluster['range'] > self.p['max_person_range_m']:
            return 0.0
        if not (self.p['leg_width_min_m'] <= cluster['width']
                <= self.p['merged_width_max_m']):
            return 0.0
        if cluster['static_ratio'] > 0.5:
            return 0.0                       # mostly known furniture
        score = 0.5
        if cluster['width'] <= self.p['leg_width_max_m']:
            score += 0.2                     # single-leg-sized
        score += 0.3 * cluster['dynamic_ratio']
        return score

    def _pair_legs(self, legs):
        candidates = []
        used = set()
        order = sorted(range(len(legs)),
                       key=lambda i: -self._leg_score(legs[i]))
        for i in order:
            if i in used:
                continue
            best, best_d = None, math.inf
            for j in order:
                if j == i or j in used:
                    continue
                d = math.hypot(legs[i]['x'] - legs[j]['x'],
                               legs[i]['y'] - legs[j]['y'])
                if self.p['leg_pair_min_m'] <= d <= self.p['leg_pair_max_m'] \
                        and d < best_d:
                    best, best_d = j, d
            if best is not None:
                used.update((i, best))
                candidates.append({
                    'x': (legs[i]['x'] + legs[best]['x']) / 2.0,
                    'y': (legs[i]['y'] + legs[best]['y']) / 2.0,
                    'dynamic': max(legs[i]['dynamic_ratio'],
                                   legs[best]['dynamic_ratio'])})
            elif (self.p['merged_width_min_m'] <= legs[i]['width']
                    <= self.p['merged_width_max_m']):
                used.add(i)                  # legs together / long coat
                candidates.append({'x': legs[i]['x'], 'y': legs[i]['y'],
                                   'dynamic': legs[i]['dynamic_ratio']})
        return candidates

    # ------------------------------------------------------------ track --
    def _track(self, candidates, stamp):
        for track in self.tracks:
            dt = max(0.0, min(0.5, stamp - track.last_update))
            track.predict(dt)
        gate = self.p['association_gate_m']
        # Followed-track identity persistence (2026-07-15): a walking
        # person occluded for a few frames left the gate, re-birthed as
        # a new id, and the old id coasted behind them as a confirmed
        # trail-ghost (ids 1,2,3,6 in 10 s). The FOLLOWED track may
        # re-acquire further out, scaled by missed frames (Kalman decays
        # coasting velocity, so prediction lags a walker).
        unclaimed = list(range(len(candidates)))
        for track in sorted(self.tracks, key=lambda t: not t.confirmed):
            effective_gate = gate
            if track.id == self.followed_id and track.missed > 0:
                effective_gate = min(0.90, gate + 0.15 * track.missed)
            best, best_d = None, effective_gate
            for i in unclaimed:
                d = math.hypot(candidates[i]['x'] - track.state[0],
                               candidates[i]['y'] - track.state[1])
                if d < best_d:
                    best, best_d = i, d
            if best is not None:
                # Occlusion-shadow gate (2026-07-15, the calm-window ghost
                # factory): a cluster standing in (or within 1 s of) the
                # lidar shadow of a CLOSER object is being partially
                # eclipsed — its centroid morphs as the occluder moves,
                # which no per-frame jump gate can catch (measured: 38
                # confirms unchanged by the ego gate; walker's shadow
                # morphed chair clusters at 5-15 cm/frame). Tracking
                # continues; motion evidence is inadmissible.
                cx, cy = candidates[best]['x'], candidates[best]['y']
                if self.sensor_xy is not None:
                    ox, oy = self.sensor_xy
                    cand_d = math.hypot(cx - ox, cy - oy)
                    cand_b = math.atan2(cy - oy, cx - ox)
                    for other in self.tracks:
                        if other is track or other.missed > 2:
                            continue
                        od = math.hypot(other.state[0] - ox,
                                        other.state[1] - oy)
                        if od >= cand_d - 0.30 or od < 0.05:
                            continue
                        ob = math.atan2(other.state[1] - oy,
                                        other.state[0] - ox)
                        half = math.atan2(0.30, od) + 0.05
                        if abs(math.atan2(math.sin(cand_b - ob),
                                          math.cos(cand_b - ob))) < half:
                            track.shadowed_until = stamp + 1.0
                            break
            if best is not None and track.update(
                    candidates[best]['x'], candidates[best]['y'],
                    stamp, trust_motion=(self.ego_calm and
                                         stamp >= track.shadowed_until)):
                # claim only on ACCEPTED update: a rejected teleport must
                # leave the cluster available for birth/other tracks
                unclaimed.remove(best)
                needed = (track.confirm_travel
                          if track.confirm_travel is not None
                          else self.p['min_travel_confirm_m'])
                # Behavioral filter (user's insight 2026-07-16): people
                # SUSTAIN walking; furniture 'movement' is artifact
                # blips. Confirmation needs displacement AND cumulative
                # time at walking speed. Resurrection zones (earned by a
                # proven walker) keep their fast single-step semantics.
                walked_enough = (
                    (track.confirm_travel is not None
                     and track.moving_time >= 0.3) or
                    track.moving_time >= self.p['min_moving_confirm_s'])
                near_furniture = any(
                    stamp - fs < 600.0 and math.hypot(
                        track.state[0] - fx, track.state[1] - fy) < 0.45
                    for fx, fy, fs in self.furniture_spots)
                if near_furniture and track.person_score < 0.30:
                    # a known furniture spot: demand double the walking
                    # evidence or a referee endorsement
                    walked_enough = (track.moving_time >=
                                     2.0 * self.p['min_moving_confirm_s'])
                if (track.travel >= needed and walked_enough
                        and stamp - track.born
                        >= self.p['min_track_age_confirm_s']):
                    if not track.confirmed:
                        self.get_logger().info(
                            f'person {track.id} CONFIRMED by motion '
                            f'({track.travel:.2f} m travelled)')
                    track.confirmed = True
            else:
                track.missed += 1
        for track in self.tracks:
            if (track.confirmed and
                    stamp - track.last_update >= self.p['track_timeout_s']):
                # Resurrection is for PEOPLE: only a track that earned
                # walker-grade travel leaves a resurrection zone. Ghost
                # deaths were seeding fast-confirm (0.15 m) zones all
                # over the room — a resurrection CASCADE (confirms at
                # 0.17-0.21 m travel, ids past 110, 22:41 session).
                if (track.person_score < 0.15
                        and track.moving_time < 2.0):
                    self.furniture_spots.append(
                        (float(track.state[0]), float(track.state[1]),
                         stamp))
                    self.furniture_spots = self.furniture_spots[-40:]
                if track.travel >= 0.80:
                    self.recent_deaths.append(
                        (float(track.state[0]), float(track.state[1]),
                         stamp))
                # Purge the grid around the death: a person who stood still
                # long enough for their track to die has been absorbed into
                # the static background — which then classifies their next
                # step as furniture and blinds re-detection (observed as
                # 40 s of 'holding: no confirmed person' on 2026-07-14).
                cx, cy = float(track.state[0]), float(track.state[1])
                cell_m = self.p['cell_m']
                reach = int(0.6 / cell_m)
                base = self._cell(cx, cy)
                for ix in range(base[0] - reach, base[0] + reach + 1):
                    for iy in range(base[1] - reach, base[1] + reach + 1):
                        self.cells.pop((ix, iy), None)
        self.recent_deaths = [d for d in self.recent_deaths
                              if stamp - d[2] < 6.0]
        # Candidate expiry (2026-07-16): tentative tracks born from
        # boot-time cell jitter sat on furniture for 84 s — never
        # confirmable (walking gate), never dying (the furniture cluster
        # re-feeds them every frame). A real approaching person confirms
        # within ~2 s of walking; 15 s without sustained motion is a
        # thing, not a person. By then its cells are static, so it
        # cannot re-birth (birth needs dynamic evidence).
        self.tracks = [t for t in self.tracks
                       if stamp - t.last_update < self.p['track_timeout_s']
                       and not (not t.confirmed
                                and stamp - t.born > 15.0
                                and t.moving_time
                                < self.p['min_moving_confirm_s'])]
        # Twin suppression (2026-07-15): one person's identity kept
        # splitting into id PAIRS (1<->30, 63<->66, 90<->110 in one
        # session) — association re-births a fresh track on the person,
        # the old id coasts 'stale', and the follower ping-pongs between
        # the person's own twins ('stuck on a ghost' = often the stale
        # twin). Two confirmed tracks this close are one person: keep
        # the freshest, inherit followership.
        keep = []
        for t in sorted(self.tracks, key=lambda t: stamp - t.last_update):
            dup = next((k for k in keep if t.confirmed and k.confirmed and
                        float(np.linalg.norm(t.state[:2] - k.state[:2]))
                        < 0.55), None)
            if dup is None:
                keep.append(t)
            elif t.id == self.followed_id:
                self.followed_id = dup.id
        self.tracks = keep
        # No new tracks until the background grid has matured: during
        # warm-up every furniture cell is still 'unknown' and clutter pairs
        # freely into phantom people.
        warmed_up = (self.first_scan_stamp is not None and
                     stamp - self.first_scan_stamp
                     > self.p['background_promote_s'] + 2.0)
        if warmed_up and self.robot_speed < 0.1:
            # Creation is also gated on the ROBOT being still: un-deskewed
            # scans smear static returns along the ego-motion, minting
            # phantom 'moving' clusters (the ghost-follow bug). Existing
            # tracks keep updating while driving; only births pause.
            for i in unclaimed:
                # Furniture never steps into previously-free space: require
                # dynamic-cell evidence to birth a track.
                near_death = any(
                    math.hypot(candidates[i]['x'] - x,
                               candidates[i]['y'] - y) < 0.7
                    for x, y, _ in self.recent_deaths)
                # Resurrection births skip the dynamic-evidence gate: the
                # purged region reads 'unknown', which yields zero dynamic
                # ratio even for a genuinely stepping person.
                if candidates[i]['dynamic'] >= 0.3 or near_death:
                    track = Track(candidates[i]['x'], candidates[i]['y'],
                                  stamp)
                    # Resurrection: born where a confirmed person just died
                    # (they paused, their track timed out) -> confirm on a
                    # single step instead of a full 0.4 m re-walk. Fixes the
                    # robot going silent after the person stands a while.
                    if near_death:
                        track.confirm_travel = 0.15
                    self.tracks.append(track)

    def _select_person(self):
        """Sticky selection of the person to follow: keep the current
        target while it lives (pausing must not cause a switch); when
        choosing fresh, prefer a MOVING confirmed track over a stationary
        one, nearest first."""
        confirmed = [t for t in self.tracks if t.confirmed]
        if not confirmed:
            self.followed_id = None
            return None
        current = next((t for t in confirmed if t.id == self.followed_id),
                       None)
        if current is not None:
            stale = (self.last_scan_stamp is not None and
                     self.last_scan_stamp - current.last_moving > 2.5)
            # A follow-switch needs a SUSTAINED walker, not one frame of
            # Kalman jitter: real speed AND real accumulated distance
            # (with the teleport gate, furniture cannot fake 0.8 m).
            movers = [t for t in confirmed
                      if t.id != current.id and t.speed > 0.35
                      and t.travel > 0.80]
            # Referee override (2026-07-16): geometry cannot prefer a
            # STATIC human over static furniture — if the neural referee
            # consistently sees a person elsewhere and none where we're
            # staring, switch even without motion.
            humans = [t for t in confirmed
                      if t.id != current.id and t.person_score > 0.35]
            # SENIORITY (2026-07-16 03:00 ping-pong: referee endorsed
            # the standing person at 0.51, the motion rule stole
            # followership back 3x in 300 ms for a 0.10-scoring
            # artifact): shape evidence OUTRANKS motion evidence. A
            # referee-endorsed target cannot be un-followed by mere
            # motion elsewhere; and no switch of any kind more than
            # once per 2 s.
            if seconds_now := (self.last_scan_stamp or 0.0):
                if seconds_now - self.last_switch_stamp < 2.0:
                    return current
            if current.person_score > 0.35:
                movers = []
            if (self.ego_calm and humans
                    and current.person_score < 0.12):
                best = max(humans, key=lambda t: t.person_score)
                self.get_logger().info(
                    f'referee override: track {current.id} scores '
                    f'{current.person_score:.2f}, track {best.id} scores '
                    f'{best.person_score:.2f} — switching')
                self.followed_id = best.id
                self.last_switch_stamp = self.last_scan_stamp or 0.0
                return best
            if not (stale and movers and self.ego_calm):
                return current
            self.get_logger().info(
                f'follow target {current.id} stationary >2.5 s while '
                f'track {movers[0].id} is moving: switching')
            self.last_switch_stamp = self.last_scan_stamp or 0.0
        endorsed = [t for t in confirmed if t.person_score > 0.35]
        moving = [t for t in confirmed if t.speed > 0.15]
        pool = endorsed or moving or confirmed
        if self.pose is None:
            best = pool[0]
        else:
            best = min(pool, key=lambda t: math.hypot(
                t.state[0] - self.pose[0], t.state[1] - self.pose[1]))
        self.followed_id = best.id
        return best

    # ---------------------------------------------------------- publish --
    def _publish(self, stamp, legs, candidates):
        markers = MarkerArray()
        wipe = Marker()
        wipe.header.frame_id = 'odom'
        wipe.header.stamp = stamp
        wipe.action = Marker.DELETEALL
        markers.markers.append(wipe)
        people = PoseArray()
        people.header.frame_id = 'odom'
        people.header.stamp = stamp
        selected = self._select_person()
        for track in self.tracks:
            body = Marker()
            body.header.frame_id = 'odom'
            body.header.stamp = stamp
            body.ns = 'people'
            body.id = track.id
            body.type = Marker.CYLINDER
            body.pose.position.x = float(track.state[0])
            body.pose.position.y = float(track.state[1])
            body.pose.position.z = 0.4
            body.scale.x = body.scale.y = 0.30
            body.scale.z = 0.8
            if selected is not None and track.id == selected.id:
                body.color = ColorRGBA(r=0.95, g=0.15, b=0.1, a=0.9)
            elif track.confirmed:
                body.color = ColorRGBA(r=0.1, g=0.9, b=0.2, a=0.8)
            else:
                body.color = ColorRGBA(r=0.9, g=0.8, b=0.1, a=0.6)
            markers.markers.append(body)
            label = Marker()
            label.header = body.header
            label.ns = 'labels'
            label.id = track.id
            label.type = Marker.TEXT_VIEW_FACING
            label.pose.position.x = float(track.state[0])
            label.pose.position.y = float(track.state[1])
            label.pose.position.z = 1.0
            label.scale.z = 0.15
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
            if selected is not None and track.id == selected.id:
                state = 'FOLLOWING'
            else:
                state = 'person' if track.confirmed else 'candidate'
            label.text = f'{state} {track.id} {track.speed:.2f} m/s'
            markers.markers.append(label)
            pose = Pose()
            pose.position.x = float(track.state[0])
            pose.position.y = float(track.state[1])
            pose.orientation.w = 1.0
            people.poses.append(pose)
        if self.p['publish_debug_markers']:
            for i, leg in enumerate(legs):
                dot = Marker()
                dot.header.frame_id = 'odom'
                dot.header.stamp = stamp
                dot.ns = 'legs'
                dot.id = i
                dot.type = Marker.SPHERE
                dot.pose.position.x = leg['x']
                dot.pose.position.y = leg['y']
                dot.pose.position.z = 0.1
                dot.scale.x = dot.scale.y = dot.scale.z = 0.08
                dot.color = ColorRGBA(r=0.2, g=0.5, b=1.0, a=0.8)
                markers.markers.append(dot)
        self.marker_pub.publish(markers)
        now = self.last_scan_stamp or 0.0
        self.debug_pub.publish(String(data=json.dumps({
            'stamp': round(now, 3),
            'ego_calm': self.ego_calm,
            'robot_speed': round(self.robot_speed, 3),
            'robot_yaw_rate': round(self.robot_yaw_rate, 3),
            'followed_id': self.followed_id,
            'legs': len(legs), 'candidates': len(candidates),
            'tracks': [{
                'id': t.id, 'x': round(float(t.state[0]), 3),
                'y': round(float(t.state[1]), 3),
                'speed': round(float(t.speed), 3),
                'travel': round(float(t.travel), 3),
                'missed': t.missed, 'confirmed': t.confirmed,
                'age_s': round(now - t.born, 2),
                'since_moving_s': round(now - t.last_moving, 2),
                'person_score': round(float(t.person_score), 2),
            } for t in self.tracks],
        }, allow_nan=False)))
        self.people_pub.publish(people)
        if not any(t.confirmed for t in self.tracks):
            self.get_logger().info(
                f'no confirmed person: {len(legs)} leg clusters, '
                f'{len(self.tracks)} tentative tracks, robot_speed '
                f'{self.robot_speed:.2f}', throttle_duration_sec=10.0)
        if selected is not None:
            person = Odometry()
            person.header.frame_id = 'odom'
            person.header.stamp = stamp
            person.pose.pose.position.x = float(selected.state[0])
            person.pose.pose.position.y = float(selected.state[1])
            person.pose.pose.orientation.w = 1.0
            person.twist.twist.linear.x = float(selected.state[2])
            person.twist.twist.linear.y = float(selected.state[3])
            self.person_pub.publish(person)


def main(args=None):
    rclpy.init(args=args)
    node = PersonTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
