#!/usr/bin/env python3
"""Follow the confirmed person (v11: streaming law + Smac, each at its
proven job).

The week of floor evidence, condensed:
- The streaming Lyapunov law is the ONLY thing that ever made close-range
  following feel right (09:14 session: sustained tail-first reverse, zero
  planner asks) — but it is blind, and greedy arc evasion around it
  produced a new corner-case every session (v8).
- Smac executes long maneuvers at 0.02-0.075 m XTE — but SHORT goals live
  inside its min-turning-radius blind spot: grind or maxit-fail (23:15
  lesson), and pure goal-following (v10) eats short goals for every meal.
  Its worst grind starved the bond heartbeat and the lifecycle manager
  executed the whole nav stack (01:23).

So: LAW when the arc to the person is clear (all close-range pursuit);
SMAC when geometry blocks the line at range >1.3 m (route-arounds and
lost-person search — goals long enough to be planned instantly and driven
cleanly); HOLD when blocked closer (sub-radius blind spot; the person
resolves it faster than any planner). Ring-escape arcs survive for one
job: backing out of inscribed cost the robot is standing in.

Everything below encodes a paid-for lesson; see the per-line comments and
the memory file before 'simplifying' any of it.
"""

import math

import json
import os

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry  # person topic carries velocity too
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class PersonFollower(Node):
    def _publish_debug(self):
        now = self.get_clock().now()
        person_age = ((now - self.person_stamp).nanoseconds / 1e9
                      if self.person_stamp is not None else None)
        self.debug_pub.publish(String(data=json.dumps({
            'mode': self.mode,
            'band': self.band,
            'gear': self.gear,
            'person_age_s': None if person_age is None
            else round(person_age, 2),
            'person_speed': round(self.person_speed, 2),
            'last_cmd_v': round(getattr(self, 'dbg_cmd_v', 0.0), 3),
            'last_cmd_kappa': round(getattr(self, 'dbg_cmd_kappa', 0.0), 3),
            'distance': round(getattr(self, 'dbg_distance', -1.0), 3),
            'bearing': round(getattr(self, 'dbg_bearing', 0.0), 3),
            'jam_hold_active': False if self.pose is None else
            (now.nanoseconds / 1e9 < self.jam_hold_until),
            'ring_escape': self.dbg_ring_escape,
            'search_done': self.search_done,
            'retry_backoff_active': bool(
                self.retry_after is not None and now < self.retry_after),
            'costmap_age_s': None if self.grid is None else round(
                (now.nanoseconds - (self.grid.header.stamp.sec * 10**9
                                    + self.grid.header.stamp.nanosec))
                / 1e9, 1),
            'max_curvature': self.p['max_curvature_1pm'],
        }, allow_nan=False)))

    def _limits(self, msg):
        try:
            limits = json.loads(msg.data)
            kappa = float(limits['max_curvature_1pm'])
        except (ValueError, KeyError, TypeError):
            return
        if kappa > 0.2 and abs(kappa - self.p['max_curvature_1pm']) > 1e-3:
            self.p['max_curvature_1pm'] = kappa
            self.p['min_turn_radius_m'] = 1.0 / kappa
            self.get_logger().info(
                f'capability update from controller: kappa {kappa:.2f} '
                f'(min turn radius {1.0 / kappa:.2f} m)')

    def __init__(self):
        super().__init__('person_follower')
        self.jam_hold_until = 0.0
        self.gear = 1
        self.gear_since = 0.0
        defaults = {
            'person_topic': '/person_tracker/person',
            'command_topic': '/cmd_vel_nav_raw',
            'desired_distance_m': 0.5,
            'arrive_tolerance_m': 0.10,
            # Distance band with hysteresis: closer than band_inner ->
            # retreat until band_retreat_stop; farther than band_outer
            # -> chase back to desired; in between -> hold still.
            'band_inner_m': 0.30,
            'band_retreat_stop_m': 0.45,
            'band_outer_m': 0.75,
            'max_speed_mps': 0.50,
            'speed_gain': 0.8,           # m/s per metre of distance error
            'max_curvature_1pm': 1.15,
            'min_turn_radius_m': 0.80,
            # ACBB'95 Lyapunov pursuit gains + gear hysteresis
            'lyap_ke': 1.3,
            'lyap_ka': 2.5,
            'gear_dwell_s': 0.5,
            'lost_timeout_s': 2.0,
            'control_rate_hz': 10.0,
            # Ring escape: robot's own cell in inscribed cost -> probes
            # run escape rules, best clear arc backs it out. Collision
            # monitor is the hard backstop.
            'evade_horizon_m': 0.8,
            # Person = obstacle + 0.55 m inflation. Bubble skips their
            # INFLATION (cost <= 99); real cells (100) skip only in the
            # leg core — a box next to the person must stay visible.
            'person_bubble_m': 0.80,
            'person_core_m': 0.45,
            # Stall: commanding but no motion. Must exceed a reverse
            # launch (settle + breakaway = 5-6 s before 5 cm).
            'stall_hold_after_s': 6.5,
            'stall_hold_s': 3.0,
            # Below this person-distance a blocked chase HOLDS instead
            # of planning: Smac cannot solve sub-radius hops and there
            # is no detour pursuit couldn't drive.
            'planner_min_detour_m': 1.3,
            # Person-feasibility arithmetic: inflation 0.55 + body 0.25
            # + tracking lag 0.15 ~= 0.95; goals at 0.95 sat exactly ON
            # the boundary and flickered unplannable. 1.15 = margin.
            'planner_goal_clearance_m': 1.15,
            'standoff_m': 0.5,
            # 7 s no-displacement = the real failure detector; 60 s cap
            # only backstops (5 s and 30 s caps each killed WORKING
            # maneuvers).
            'reorient_timeout_s': 60.0,
            'reorient_stall_s': 7.0,
            'retry_backoff_s': 2.5,
            # Lost-person search: debounced (identity flips re-confirm
            # in 0.4-3 s), once per sighting, warm window only.
            # Corners: search extrapolates along the last velocity — a
            # vantage INTO the corridor they turned into; retry falls
            # back to the plain last-seen point. Small pullback: nobody
            # stands at a search goal.
            'search_after_s': 4.0,
            'search_max_age_s': 20.0,
            'search_lead_s': 1.5,
            'search_lead_max_m': 1.5,
            'search_clearance_m': 0.35,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}

        self.person = None
        self.person_speed = 0.0
        self.person_stamp = None
        self.pose = None
        self.robot_speed = 0.0
        self.mode = 'chase'              # chase | avoid | search
        self.band = 'hold'
        self.was_commanding = False
        self.stalled_since = None
        self.grid = None
        self.dbg_ring_escape = False
        self.goal_handle = None
        self.goal_pending = False
        self.superseded = None
        self.reorient_started = None
        self.avoid_progress = None
        self.reorient_strikes = 0
        self.retry_after = None
        self.last_person_xy = None
        self.last_person_vel = (0.0, 0.0)
        self.search_done = False
        self.search_attempt = 0

        self.create_subscription(Odometry, self.p['person_topic'],
                                 self._person, 10)
        self.create_subscription(Odometry, '/odom', self._odom, 20)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                 self._costmap, 1)
        self.debug_pub = self.create_publisher(
            String, '/person_follower/debug', 5)
        self.create_timer(0.2, self._publish_debug)
        # Live capability: the controller publishes its learned
        # executable limits; hand-frozen copies went stale the day
        # capability was promoted. Latched topic.
        limit_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, '/controller/limits', self._limits, limit_qos)
        self.command_pub = self.create_publisher(
            Twist, self.p['command_topic'], 10)
        self.navigator = ActionClient(self, NavigateToPose,
                                      'navigate_to_pose')
        # Maneuvers use a BT whose FollowPath selects
        # follow_goal_checker: arrival heading free, position ball
        # loose — alignment tails never get driven.
        self.follow_bt = ''
        try:
            candidate = os.path.join(
                get_package_share_directory('ackermann_robot'),
                'config', 'navigate_to_pose_follow.xml')
            if os.path.exists(candidate):
                self.follow_bt = candidate
            else:
                self.get_logger().warning(
                    f'{candidate} missing; maneuvers use the default BT')
        except Exception as error:
            self.get_logger().warning(
                f'follow BT unavailable ({error}); using default BT')
        self.create_timer(1.0 / self.p['control_rate_hz'], self._tick)
        self.get_logger().info(
            'person_follower v11: streaming law when clear, Smac when '
            'blocked at range, hold when blocked close — hybrid')

    # ------------------------------------------------------ costmap arcs --
    def _costmap(self, msg):
        self.grid = msg

    def _cost_at(self, x, y):
        info = self.grid.info
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)
        if not (0 <= col < info.width and 0 <= row < info.height):
            return 0
        return max(0, self.grid.data[row * info.width + col])

    def _arc_probe(self, gear, kappa, person_xy, escaping=False):
        """Sweep one arc through the local costmap.

        (worst_cost, end_x, end_y) or None if not drivable. 100 = real
        obstacle: always veto. 99 = inscribed ring: veto only from a
        clear start — a robot standing IN the ring may drive through 99
        as long as the arc exits it. Person's leg core skipped; their
        inflation skipped only below lethal.
        """
        x, y, th = self.pose
        # One sample per 5 cm costmap cell: no diagonal cell-hopping.
        ds = self.p['evade_horizon_m'] / 16.0
        worst = 0
        last_cost = 0
        for _ in range(16):
            th += gear * ds * kappa
            x += gear * ds * math.cos(th)
            y += gear * ds * math.sin(th)
            cost = self._cost_at(x, y)
            if person_xy is not None:
                pd = math.hypot(x - person_xy[0], y - person_xy[1])
                if pd < self.p['person_core_m']:
                    continue    # the person's own legs
                if pd < self.p['person_bubble_m'] and cost < 100:
                    continue    # their inflation — never a real cell
            if cost >= 100:
                return None
            if cost >= 99 and not escaping:
                return None
            worst = max(worst, cost)
            last_cost = cost
        if escaping and last_cost >= 99:
            return None                  # an escape arc must ESCAPE
        return worst, x, y

    def _ring_escape_arc(self, person_xy, allow_flip):
        """Best clear ESCAPING arc over both gears, or None (boxed)."""
        k_max = self.p['max_curvature_1pm']
        best, best_score = None, None
        for gear in (self.gear, -self.gear):
            for i in range(9):
                kappa = k_max * (i - 4) / 4.0
                probe = self._arc_probe(gear, kappa, person_xy,
                                        escaping=True)
                if probe is None:
                    continue
                worst = probe[0]
                flip_pen = 0.0
                if gear != self.gear:
                    # Flips inside the dwell only as a last resort (10 Hz
                    # re-decision flapped D<->R every tick, 23:47).
                    flip_pen = 0.3 if allow_flip else 1000.0
                score = -worst / 100.0 - flip_pen
                if best_score is None or score > best_score:
                    best, best_score = (gear, kappa), score
        return best

    def _law_curvature(self, gear, alpha, error):
        """Pursuit law in the frame of a gear: forward -> nose bearing,
        reverse -> tail bearing (v7: 'person behind' is a stable
        equilibrium tail-first, not a J-turn). Returns (v_law,
        curvature)."""
        fb = alpha if gear > 0 else wrap(alpha - math.pi)
        v_law = self.p['lyap_ke'] * error * math.cos(fb)
        w_law = (self.p['lyap_ka'] * fb +
                 self.p['lyap_ke'] * math.sin(fb) * math.cos(fb))
        v_for = max(abs(v_law), 0.15, min(0.30, 0.5 * abs(error)))
        curvature = max(-self.p['max_curvature_1pm'],
                        min(self.p['max_curvature_1pm'],
                            w_law / (gear * v_for)))
        return v_law, curvature

    # ------------------------------------------------------------ inputs --
    def _person(self, msg):
        self.person = msg
        self.person_speed = math.hypot(msg.twist.twist.linear.x,
                                       msg.twist.twist.linear.y)
        self.person_stamp = self.get_clock().now()
        self.last_person_xy = (msg.pose.pose.position.x,
                               msg.pose.pose.position.y)
        self.last_person_vel = (msg.twist.twist.linear.x,
                                msg.twist.twist.linear.y)
        self.search_done = False
        self.search_attempt = 0

    def _odom(self, msg):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     yaw_from_quaternion(msg.pose.pose.orientation))
        self.robot_speed = abs(msg.twist.twist.linear.x)

    # ----------------------------------------------------------- control --
    def _stop_stream(self):
        if self.was_commanding:
            self.command_pub.publish(Twist())
            self.was_commanding = False

    def _cancel_maneuver(self, reason):
        if self.goal_handle is not None:
            self.get_logger().info(f'maneuver canceled: {reason}')
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.reorient_started = None
        self.avoid_progress = None
        self.mode = 'chase'

    def _maneuver_supervision(self, seconds):
        """Shared stall/timeout watch for avoid and search drives.
        Returns True when the maneuver was killed."""
        if self.avoid_progress is None:
            self.avoid_progress = (self.pose[0], self.pose[1], seconds)
        elif math.hypot(self.pose[0] - self.avoid_progress[0],
                        self.pose[1] - self.avoid_progress[1]) > 0.08:
            self.avoid_progress = (self.pose[0], self.pose[1], seconds)
        stalled = (seconds - self.avoid_progress[2]
                   > self.p['reorient_stall_s'])
        expired = (self.reorient_started is not None and
                   seconds - self.reorient_started
                   > self.p['reorient_timeout_s'])
        if stalled or expired:
            self._cancel_maneuver('stalled (no progress)' if stalled
                                  else 'timeout')
            return True
        return False

    def _blocked_response(self, seconds, distance):
        """Chase cannot drive at the person: close -> hold for them,
        far -> one Smac route-around ask (respecting the backoff)."""
        self._stop_stream()
        self.stalled_since = None
        if distance < self.p['planner_min_detour_m']:
            self.get_logger().info(
                'chase blocked at close range: holding — person is '
                'across an obstacle, waiting for them',
                throttle_duration_sec=5.0)
            self.jam_hold_until = seconds + self.p['stall_hold_s']
            return
        self.get_logger().info(
            'chase blocked: asking the planner to route around')
        if self._send_goal(self.person.pose.pose.position.x,
                           self.person.pose.pose.position.y, seconds):
            self.mode = 'avoid'

    def _tick(self):
        now = self.get_clock().now()
        seconds = now.nanoseconds / 1e9
        fresh = (self.person_stamp is not None and
                 (now - self.person_stamp).nanoseconds / 1e9
                 < self.p['lost_timeout_s'])
        if not fresh or self.pose is None:
            self._stop_stream()
            self.stalled_since = None
            age = ((now - self.person_stamp).nanoseconds / 1e9
                   if self.person_stamp is not None else math.inf)
            if self.mode == 'avoid':
                self._cancel_maneuver('person lost')
            if self.mode == 'search':
                if self.goal_handle is None and not self.goal_pending:
                    self.mode = 'chase'   # rejected/finished elsewhere
                elif (self.goal_handle is not None
                        and self._maneuver_supervision(seconds)):
                    pass                  # killed; retry logic below
                else:
                    return                # keep driving to the vantage
            if age < self.p['search_after_s']:
                return
            if (self.mode == 'chase'
                    and age < self.p['search_max_age_s']
                    and self.last_person_xy is not None
                    and not self.search_done
                    and not self.goal_pending
                    and (self.retry_after is None
                         or now >= self.retry_after)):
                tx, ty = self.last_person_xy
                speed = math.hypot(*self.last_person_vel)
                if self.search_attempt == 0 and speed > 0.15:
                    # Corners: vantage along their heading — INTO the
                    # corridor they turned into; retry falls back to
                    # the plain last-seen point.
                    lead = min(self.p['search_lead_max_m'],
                               speed * self.p['search_lead_s'])
                    tx += self.last_person_vel[0] / speed * lead
                    ty += self.last_person_vel[1] / speed * lead
                    self.get_logger().info(
                        'person lost: searching ahead along their path')
                else:
                    self.get_logger().info(
                        'person lost: driving to last seen position')
                if self._send_goal(
                        tx, ty, seconds,
                        clearance=self.p['search_clearance_m']):
                    self.mode = 'search'
                    self.search_attempt += 1
                return
            self.get_logger().info(
                'holding: no confirmed person (walk a step to '
                're-confirm)', throttle_duration_sec=10.0)
            return
        dx = self.person.pose.pose.position.x - self.pose[0]
        dy = self.person.pose.pose.position.y - self.pose[1]
        distance = math.hypot(dx, dy)
        self.dbg_distance = distance
        self.dbg_bearing = math.atan2(dy, dx) - self.pose[2]
        if seconds < self.jam_hold_until:
            self._stop_stream()
            return
        error = distance - self.p['desired_distance_m']
        bearing = wrap(math.atan2(dy, dx) - self.pose[2])
        if self.mode == 'search':
            self._cancel_maneuver('person reacquired')
        if self.mode == 'avoid':
            # Exits: person arriving dissolves the mission; stall and
            # total cap catch failure; completion via _result.
            if distance < self.p['band_outer_m']:
                self._cancel_maneuver('person within reach')
                self.reorient_strikes = 0
                return
            if self._maneuver_supervision(seconds):
                self.reorient_strikes += 1
                if self.reorient_strikes >= 2:
                    self.get_logger().info(
                        'route-around struck out twice: waiting for '
                        'the person to come around')
                    self.jam_hold_until = seconds + 8.0
                    self.reorient_strikes = 0
            elif self.goal_handle is None and not self.goal_pending:
                self.mode = 'chase'
            else:
                return
        # ------------------------------------------------- distance band --
        if distance < self.p['band_inner_m']:
            self.band = 'retreat'
        elif self.band == 'retreat':
            if distance >= self.p['band_retreat_stop_m']:
                self.band = 'hold'
        elif self.band == 'chase':
            if error < self.p['arrive_tolerance_m']:
                self.band = 'hold'
        if self.band == 'hold' and distance > self.p['band_outer_m']:
            self.band = 'chase'
        if self.band == 'retreat':
            # Straight away from the person, whichever end they crowd.
            command = Twist()
            magnitude = min(0.35, self.p['speed_gain']
                            * (self.p['band_retreat_stop_m'] - distance))
            command.linear.x = float(
                -magnitude if abs(bearing) < math.pi / 2 else magnitude)
            self.command_pub.publish(command)
            self.was_commanding = True
            self.stalled_since = None
            return
        if self.band == 'hold':
            self._stop_stream()
            self.stalled_since = None
            return
        # --------------------------------------------------- pursuit law --
        alpha = bearing
        person_xy = (self.person.pose.pose.position.x,
                     self.person.pose.pose.position.y)
        escaping = (self.grid is not None and
                    self._cost_at(self.pose[0], self.pose[1]) >= 99)
        k_need = 2.0 * abs(math.sin(alpha)) / max(distance, 0.2)
        k_max = self.p['max_curvature_1pm']
        # Symmetric gear latch (+/-0.20 band + dwell = chatter guard,
        # not preference); never latch into a blocked direction.
        if seconds - self.gear_since > self.p['gear_dwell_s']:
            cos_a = math.cos(alpha)
            deep_inside = (k_need > 1.6 * k_max and distance < 1.4
                           and error > 0.1)
            candidate = 0
            if self.gear > 0 and (cos_a < -0.20 or deep_inside):
                candidate = -1
            elif self.gear < 0 and cos_a > 0.20 and not deep_inside:
                candidate = 1
            if candidate != 0 and self.grid is not None:
                _, cand_kappa = self._law_curvature(candidate, alpha,
                                                    error)
                if self._arc_probe(candidate, cand_kappa, person_xy,
                                   escaping) is None:
                    self.get_logger().info(
                        'bearing prefers %s but that way is blocked' %
                        ('reverse' if candidate < 0 else 'forward'),
                        throttle_duration_sec=2.0)
                    candidate = 0
            if candidate != 0:
                self.gear = candidate
                self.gear_since = seconds
                self.stalled_since = None
                self.get_logger().info(
                    'gear latch: %s' %
                    ('forward' if candidate > 0
                     else 'reverse (person behind/beside)'))
        v_law, curvature = self._law_curvature(self.gear, alpha, error)
        # ------------------------------------------- blocked-arc branch --
        self.dbg_ring_escape = False
        if (self.grid is not None
                and self._arc_probe(self.gear, curvature, person_xy,
                                    escaping) is None):
            if escaping:
                choice = self._ring_escape_arc(
                    person_xy,
                    allow_flip=(seconds - self.gear_since
                                > self.p['gear_dwell_s']))
                if choice is None:
                    self.get_logger().info(
                        'boxed in: every arc blocked — holding',
                        throttle_duration_sec=5.0)
                    self._stop_stream()
                    self.stalled_since = None
                    self.jam_hold_until = seconds + 1.0
                    return
                self.dbg_ring_escape = True
                escape_gear, curvature = choice
                if escape_gear != self.gear:
                    self.gear = escape_gear
                    self.gear_since = seconds
                    self.stalled_since = None
                    self.get_logger().info(
                        'gear latch: %s (ring escape)' %
                        ('forward' if escape_gear > 0 else 'reverse'))
                self.get_logger().info(
                    f'ring escape: gear {self.gear:+d} '
                    f'kappa {curvature:+.2f}',
                    throttle_duration_sec=2.0)
            else:
                # Clear start, blocked arc: Smac's job (at range).
                self._blocked_response(seconds, distance)
                return
        # ------------------------------------------------------- command --
        floor = max(0.15, min(0.30, 0.5 * abs(error)))
        magnitude = max(floor, min(self.p['max_speed_mps'], abs(v_law)))
        # Speed matching in EITHER gear — back and forth symmetric.
        magnitude = min(magnitude,
                        self.person_speed + self.p['speed_gain'] * error)
        if self.dbg_ring_escape:
            magnitude = min(magnitude, 0.25)
        speed = self.gear * magnitude
        command = Twist()
        command.linear.x = float(speed)
        command.angular.z = float(command.linear.x * curvature)
        self.dbg_cmd_v = command.linear.x
        self.dbg_cmd_kappa = curvature
        self.command_pub.publish(command)
        self.was_commanding = True
        if abs(command.linear.x) > 0.05 and self.robot_speed < 0.08:
            # Reset only on GENUINE motion below — standstill odom
            # jitter restarted the clock forever under <0.03.
            if self.stalled_since is None:
                self.stalled_since = seconds
            elif (seconds - self.stalled_since
                  > self.p['stall_hold_after_s']):
                # Probe-clear commands, no motion: the gate sees what
                # the probes cannot. Same decision as a blocked arc.
                self.get_logger().info(
                    'chase pinned: command produces no motion')
                self._blocked_response(seconds, distance)
        else:
            self.stalled_since = None

    # ---------------------------------------------------------- maneuver --
    def _send_goal(self, target_x, target_y, seconds, clearance=None):
        """Send a NavigateToPose goal; True when actually dispatched.

        clearance defaults to the person-follow pullback; searches pass
        a small one (nobody stands at a search goal)."""
        if not self.navigator.server_is_ready():
            self.get_logger().warning('navigate_to_pose not ready',
                                      throttle_duration_sec=5.0)
            return False
        if (self.retry_after is not None
                and self.get_clock().now() < self.retry_after):
            return False
        dx = target_x - self.pose[0]
        dy = target_y - self.pose[1]
        distance = max(math.hypot(dx, dy), 0.05)
        if clearance is None:
            clearance = max(self.p['standoff_m'],
                            self.p['planner_goal_clearance_m'])
        scale = max(0.0, (distance - clearance) / distance)
        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.pose[0] + dx * scale
        pose.pose.position.y = self.pose[1] + dy * scale
        # Goal heading = the forward-bias lever: Smac (1.3.11, no
        # goal_heading_mode) must PLAN to this yaw even though the loose
        # checker ignores it at arrival. Target behind -> rear-first
        # heading, so straight reverse is the cheap plan instead of a
        # forward loop-around.
        bearing = math.atan2(dy, dx)
        rel = wrap(bearing - self.pose[2])
        yaw = bearing if math.cos(rel) >= 0.0 else wrap(bearing + math.pi)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.behavior_tree = self.follow_bt  # yaw-free arrival
        if self.goal_handle is not None:
            # About to preempt: the predecessor reports ABORTED; that
            # result must be ignored even if it arrives before the new
            # goal's acceptance.
            self.superseded = self.goal_handle
        self.goal_pending = True
        self.reorient_started = seconds
        self.avoid_progress = None
        future = self.navigator.send_goal_async(goal)
        future.add_done_callback(self._goal_response)
        return True

    def _goal_response(self, future):
        self.goal_pending = False
        handle = future.result()
        if handle is None or not handle.accepted:
            # Rejections back off like aborts: a downed action server
            # rejects instantly; without this the search reject-spun at
            # 5 Hz against a dead stack (01:23 run).
            self.retry_after = self.get_clock().now() + \
                rclpy.duration.Duration(seconds=self.p['retry_backoff_s'])
            self.get_logger().warning(
                f"goal rejected; backing off {self.p['retry_backoff_s']} s")
            if self.mode in ('avoid', 'search'):
                self.mode = 'chase'
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda fut, h=handle: self._result(fut, h))

    def _result(self, future, handle):
        # Results attributed to THEIR goal (preempted predecessors
        # report ABORTED — bookkeeping, not failure).
        if handle is self.superseded:
            self.superseded = None
            return
        if handle is not self.goal_handle:
            return
        try:
            status = future.result().status
        except Exception:
            status = GoalStatus.STATUS_UNKNOWN
        self.goal_handle = None
        if status == GoalStatus.STATUS_ABORTED:
            self.retry_after = self.get_clock().now() + \
                rclpy.duration.Duration(seconds=self.p['retry_backoff_s'])
            self.get_logger().warning(
                f"maneuver aborted; backing off {self.p['retry_backoff_s']} s")
        elif status == GoalStatus.STATUS_SUCCEEDED:
            self.reorient_strikes = 0
            if self.mode == 'search':
                self.search_done = True   # one search per sighting
                self.get_logger().info(
                    'search finished; holding for the person')
        if self.mode in ('avoid', 'search'):
            self.mode = 'chase'


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ctrl-C safety: cancel any active maneuver (bt_navigator would
        # keep driving it) and leave a zero stream command.
        try:
            if node.goal_handle is not None:
                future = node.goal_handle.cancel_goal_async()
                for _ in range(20):
                    rclpy.spin_once(node, timeout_sec=0.1)
                    if future.done():
                        break
            node.command_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
