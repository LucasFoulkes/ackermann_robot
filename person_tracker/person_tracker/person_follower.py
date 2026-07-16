#!/usr/bin/env python3
"""Follow the confirmed person (v5: forward-first, geometric trigger).

Two modes, each using the subsystem proven at that job:

CHASE (person single-arc reachable): direct velocity streaming using the
Park–Kuipers graceful-motion control law — the same law Nav2's Following/
Docking servers use. Curvature from the egocentric polar error, speed
automatically collapsing with curvature demand. Reaction latency is one
10 Hz tick. Commands flow through the normal safety chain
(/cmd_vel_nav_raw -> adaptive limits -> collision monitor).

The mode trigger is GEOMETRIC, not a bearing heuristic: the closed-form
Ackermann arc radius through the person's body-frame position (the PRM
paper's edge test, a2=0 for our rear-axle frame). If ONE forward arc
within the turning limit reaches them, pursue; otherwise run the REACTIVE
3-POINT TURN — alternating full-lock forward/reverse arcs that rotate the
nose toward the person while translations cancel (Reeds-Shepp cusp logic,
executed reactively). Phases end on: person becomes arc-reachable, phase
blocked (gate/no motion -> switch immediately; walls truncate phases so
the maneuver squeezes into available space), or timeout. The person's
live bearing steers every phase — nothing is ever frozen.

Smac remains only for obstacle route-arounds (chase jammed) and
search-at-last-seen. All commands flow through the normal safety chain.
"""

import math

import json

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry  # person topic carries velocity too
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
import json as _json


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
        self.debug_pub.publish(String(data=_json.dumps({
            'mode': self.mode,
            'kturn_phase': self.kturn_phase,
            'search_done': self.search_done,
            'person_age_s': None if person_age is None
            else round(person_age, 2),
            'person_speed': round(getattr(self, 'person_speed', 0.0), 2),
            'last_cmd_v': round(getattr(self, 'dbg_cmd_v', 0.0), 3),
            'last_cmd_kappa': round(getattr(self, 'dbg_cmd_kappa', 0.0), 3),
            'distance': round(getattr(self, 'dbg_distance', -1.0), 3),
            'bearing': round(getattr(self, 'dbg_bearing', 0.0), 3),
            'retry_backoff_active': bool(
                self.retry_after is not None and now < self.retry_after),
            'jam_hold_active': False if self.pose is None else
            (now.nanoseconds / 1e9 < self.jam_hold_until),
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
            # Distance band with hysteresis (no oscillating at boundaries):
            # closer than band_inner -> retreat until band_retreat_stop;
            # farther than band_outer -> chase back to desired; in between ->
            # hold still.
            'band_inner_m': 0.30,
            'band_retreat_stop_m': 0.45,
            'band_outer_m': 0.75,
            'max_speed_mps': 0.50,
            'speed_gain': 0.8,           # m/s per metre of distance error
            'max_curvature_1pm': 1.15,
            # Park-Kuipers law: kappa = (k2*bearing + sin(bearing)) / r,
            # v = vmax / (1 + beta*|kappa|^lambda) — speed collapses as
            # curvature demand rises (graceful approach, no orbiting).
            'k2': 2.0,
            # beta 0.4 allowed ~0.39 m/s at full lock; proven tight-curve
            # tracking speed is ~0.17-0.20 (same regression as RPP's
            # regulated radius). 1.5 gives ~0.20 at |kappa|=1.15.
            # 1.5 collapsed speed to ~0.26 at moderate curvature — with
            # the 0.40 envelope, falling behind a walking person was
            # arithmetic (user-observed). 1.0 keeps graceful slowdown.
            'beta': 1.0,
            'lambda': 2.0,
            # Geometric mode trigger: minimum radius of a single forward
            # arc that counts as 'reachable' (chassis limit ~0.87 m at
            # kappa 1.15; margin keeps the boundary honest). Exit the
            # 3-point turn only when reachable with extra margin
            # (hysteresis so the boundary cannot dither).
            # 0.90 predated the measured-capability promotion: the plant
            # now executes R 0.77 (kappa 1.30). 0.90 judged persons at
            # R 0.80-0.89 'unreachable' and fired needless 3-point turns
            # that CHATTERED against pursuit every ~1 s (19:37 session)
            # — the 'so bad at turning / can't match speed' complaint.
            'min_turn_radius_m': 0.80,
            'reach_exit_margin': 1.15,
            # Reactive 3-point turn: full-lock alternating arcs.
            'kturn_crawl_mps': 0.18,
            'kturn_phase_max_s': 2.5,
            'kturn_blocked_after_s': 1.2,
            # 25 s let a doomed plan freeze the robot; a reachable goal
            # plans in ~1-2 s on this Pi. Fail fast, fall back to the
            # reactive 3-point (which handled it fine when the planner
            # finally gave up).
            'reorient_timeout_s': 10.0,
            'retry_backoff_s': 2.5,
            'standoff_m': 0.5,
            # ACBB'95 Lyapunov pursuit gains + gear hysteresis (v6)
            'lyap_ke': 1.3,
            'lyap_ka': 2.5,
            'gear_dwell_s': 0.5,
            # Below this person-distance a blocked chase HOLDS instead of
            # planning (Smac cannot solve sub-radius hops; see 23:15).
            'planner_min_detour_m': 1.3,
            # inflation_radius (0.55) + footprint half-length + margin
            'planner_goal_clearance_m': 0.95,
            'lost_timeout_s': 2.0,
            # Person vanished: drive once to where they were last seen
            # (if the sighting is recent enough) before giving up.
            'search_max_age_s': 20.0,
            'control_rate_hz': 10.0,
            # Chase-mode jam escape (nose pinned by the obstacle gate).
            'stall_escape_after_s': 3.0,
            'escape_duration_s': 2.0,
            'escape_speed_mps': 0.14,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}

        self.person = None
        self.person_speed = 0.0
        self.person_stamp = None
        self.last_person_xy = None
        self.search_done = False
        self.pose = None
        self.robot_speed = 0.0
        self.mode = 'chase'
        self.kturn_phase = None           # None | 'fwd' | 'rev'
        self.kturn_phase_until = 0.0
        self.kturn_blocked_since = None
        self.band = 'hold'
        self.was_commanding = False
        self.stalled_since = None
        self.escape_until = None
        self.jam_count = 0
        self.goal_handle = None
        self.goal_pending = False
        self.reorient_started = None
        self.retry_after = None

        self.create_subscription(Odometry, self.p['person_topic'],
                                 self._person, 10)
        self.create_subscription(Odometry, '/odom', self._odom, 20)
        self.debug_pub = self.create_publisher(
            String, '/person_follower/debug', 5)
        self.create_timer(0.2, self._publish_debug)
        # Live capability (user: 'it is not learning that it can turn
        # more'): the controller PUBLISHES its learned executable limits;
        # hand-frozen copies here went stale the day capability was
        # promoted (1.15 vs executable 1.30). Latched topic.
        limit_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, '/controller/limits', self._limits, limit_qos)
        self.command_pub = self.create_publisher(
            Twist, self.p['command_topic'], 10)
        self.navigator = ActionClient(self, NavigateToPose,
                                      'navigate_to_pose')
        self.create_timer(1.0 / self.p['control_rate_hz'], self._tick)
        self.get_logger().info(
            'person_follower v6: ACBB95 Lyapunov pursuit — one law, '
            'hysteretic gear, no modes')

    # ------------------------------------------------------------ inputs --
    def _person(self, msg):
        self.person = msg
        self.person_speed = math.hypot(msg.twist.twist.linear.x,
                                       msg.twist.twist.linear.y)
        self.person_stamp = self.get_clock().now()
        self.last_person_xy = (msg.pose.pose.position.x,
                               msg.pose.pose.position.y)
        self.search_done = False

    def _odom(self, msg):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     yaw_from_quaternion(msg.pose.pose.orientation))
        self.robot_speed = abs(msg.twist.twist.linear.x)

    # ------------------------------------------------------------- modes --
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
        self.mode = 'chase'

    def _tick(self):
        now = self.get_clock().now()
        seconds = now.nanoseconds / 1e9
        fresh = (self.person_stamp is not None and
                 (now - self.person_stamp).nanoseconds / 1e9
                 < self.p['lost_timeout_s'])
        if not fresh or self.pose is None:
            self._stop_stream()
            if self.mode == 'avoid':
                self._cancel_maneuver('person lost')
            if self.mode == 'search':
                return                   # keep driving to the last sighting
            if (self.mode == 'chase' and self.pose is not None
                    and self.last_person_xy is not None
                    and not self.search_done
                    and self.person_stamp is not None
                    and (now - self.person_stamp).nanoseconds / 1e9
                    < self.p['search_max_age_s']
                    and (self.retry_after is None
                         or now >= self.retry_after)):
                self.get_logger().info(
                    'person lost: searching at last seen position')
                self._start_maneuver(self.last_person_xy[0],
                                     self.last_person_xy[1], mode='search')
            else:
                self.get_logger().info(
                    'holding: no confirmed person (walk a step to '
                    're-confirm)', throttle_duration_sec=10.0)
            return
        dx = self.person.pose.pose.position.x - self.pose[0]
        dy = self.person.pose.pose.position.y - self.pose[1]
        distance = math.hypot(dx, dy)
        self.dbg_distance = distance
        self.dbg_bearing = math.atan2(dy, dx) - (
            self.pose[2] if self.pose else 0.0)
        if seconds < self.jam_hold_until:
            # close-range jam hold: person is across an obstacle; wait
            # for them instead of bumping the gate or asking Smac for
            # sub-radius miracles. Re-evaluates every tick after expiry.
            self._stop_stream()
            return
        error = distance - self.p['desired_distance_m']
        bearing = wrap(math.atan2(dy, dx) - self.pose[2])

        if self.mode == 'search':
            # Person reacquired while searching: back to direct behavior.
            self._cancel_maneuver('person reacquired')
        if self.mode == 'avoid':
            # Route-around: the person IS in the front cone (that is why
            # chase was running), so a bearing-based exit would cancel it
            # instantly (00:50 session: five aborted route-arounds in 16 s).
            # Exit only on completion, timeout, or the person moving far.
            if (self.reorient_started is not None and
                    seconds - self.reorient_started
                    > self.p['reorient_timeout_s']):
                self._cancel_maneuver('route-around timeout')
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
            # Straight away from the person, whichever end they crowd
            # (front person -> back up; rear person -> pull forward).
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
        if self.escape_until is not None:
            if seconds < self.escape_until:
                command = Twist()
                command.linear.x = -self.p['escape_speed_mps']
                self.command_pub.publish(command)
                self.was_commanding = True
                return
            self.escape_until = None
            self.stalled_since = None
        # ---- Follower v6: ACBB'95 Lyapunov pursuit (from the user's
        # simulation). ONE law, no modes: v = ke*d*cos(a), w = ka*a +
        # ke*sin(a)*cos(a). Reversal is not a special case — cos(a) < 0
        # makes v negative when the person is behind/sharply aside, and
        # swing-backs / 3-point-like moves EMERGE. The law is Lyapunov-
        # stable: the distance to the target never increases. This
        # replaces the arc-reachability test, the kturn phase machine,
        # and the Park-Kuipers law (v5) — the modes we spent two nights
        # debugging individually.
        alpha = bearing
        v_law = self.p['lyap_ke'] * error * math.cos(alpha)
        w_law = (self.p['lyap_ka'] * alpha +
                 self.p['lyap_ke'] * math.sin(alpha) * math.cos(alpha))
        # A car cannot pivot at the cusp, so the gear is a HYSTERETIC
        # commitment to the law's sign: reverse only when the person is
        # clearly behind (cos a < -0.25), forward once the nose is
        # within ~30 deg, minimum dwell so noise cannot chatter D<->R
        # (the controller's own 0.5 s debounce still backstops).
        # kNeed vs kMax (user's sim): the arc curvature required to hit
        # the target vs the tightest arc the body is BELIEVED to drive
        # (live from /controller/limits). A target inside the turning
        # circle is fastest reached by backing up with wheels turned —
        # even though it is ahead.
        k_need = 2.0 * abs(math.sin(alpha)) / max(distance, 0.2)
        k_max = self.p['max_curvature_1pm']
        if seconds - self.gear_since > self.p['gear_dwell_s']:
            if self.gear > 0 and (
                    math.cos(alpha) < -0.25 or
                    (k_need > 0.95 * k_max and distance < 2.2
                     and error > 0.1)):
                self.gear = -1
                self.gear_since = seconds
                self.get_logger().info(
                    'gear latch: reverse (person behind/beside)')
            elif self.gear < 0 and abs(alpha) < 1.2 and k_need <= 0.7 * k_max:
                self.gear = 1
                self.gear_since = seconds
                self.get_logger().info('gear latch: forward')
        # Divide by the speed we will actually DRIVE, not a 0.1 floor:
        # the tiny floor exploded w/v and pinned the clamp on the very
        # first field run (|kappa| med = p90 = 1.27, arcs at full lock
        # everywhere = 'steering terrible, very slow'). Saturation still
        # happens near cusps — that is the law meeting car physics —
        # but mid-pursuit steering is now proportional.
        v_for_kappa = max(abs(v_law), 0.15, min(0.30, 0.5 * abs(error)))
        curvature = max(-self.p['max_curvature_1pm'],
                        min(self.p['max_curvature_1pm'],
                            w_law / (self.gear * v_for_kappa)))
        # Speed magnitude from the law; the floor (which shrinks near the
        # standoff so it cannot orbit) keeps a car that must move to turn
        # moving, and stays above the drivetrain's drag-loaded band.
        floor = max(0.15, min(0.30, 0.5 * abs(error)))
        cap = (self.p['max_speed_mps'] if self.gear > 0
               else min(self.p['max_speed_mps'], 0.35))
        magnitude = max(floor, min(cap, abs(v_law)))
        if self.gear > 0:
            # Speed matching: the person's pace plus a catch-up term.
            magnitude = min(magnitude,
                            self.person_speed
                            + self.p['speed_gain'] * error)
        speed = self.gear * magnitude
        command = Twist()
        command.linear.x = float(speed)
        command.angular.z = float(command.linear.x * curvature)
        self.dbg_cmd_v = command.linear.x
        self.dbg_cmd_kappa = curvature
        self.command_pub.publish(command)
        self.was_commanding = True
        if abs(command.linear.x) > 0.05 and self.robot_speed < 0.03:
            if self.stalled_since is None:
                self.stalled_since = seconds
            elif seconds - self.stalled_since > self.p['stall_escape_after_s']:
                self.stalled_since = None
                # Smac Hybrid is structurally bad at sub-meter goals (the
                # pose sits inside its own min turning radius: 'exceeded
                # maximum iterations' on a 0.91 m hop, 23:15 session — the
                # robot froze in 10 s planning loops). Within short range
                # there is no detour pursuit couldn't drive: the honest
                # move is to hold facing the person until they come
                # around. Planner detours only for genuinely distant
                # blockages.
                if distance < self.p['planner_min_detour_m']:
                    self.get_logger().info(
                        'chase blocked at close range: holding — person '
                        'is across an obstacle, waiting for them')
                    self._stop_stream()
                    self.stalled_since = None
                    self.jam_hold_until = seconds + 3.0
                    return
                # Pursuit is blocked at range: hand it to Smac — the
                # planner sees the costmap and routes around, while blind
                # pursuit can only bump the obstacle gate.
                self.get_logger().info(
                    'chase blocked: asking the planner to route around')
                self._stop_stream()
                self._start_maneuver(self.person.pose.pose.position.x,
                                     self.person.pose.pose.position.y,
                                     mode='avoid')
                return
        else:
            self.stalled_since = None
            if self.robot_speed > 0.25:
                self.jam_count = 0

    # ---------------------------------------------------------- maneuver --
    def _start_maneuver(self, target_x, target_y, mode):
        if not self.navigator.server_is_ready():
            self.get_logger().warning('navigate_to_pose not ready',
                                      throttle_duration_sec=5.0)
            return
        dx = target_x - self.pose[0]
        dy = target_y - self.pose[1]
        distance = max(math.hypot(dx, dy), 0.05)
        # PLANNER goals must stand clear of the person's own inflation
        # bubble: a goal at the 0.5 m standoff sits inside the 0.55 m
        # inflation of the person-as-obstacle, so Smac ground its full
        # timeout on an unreachable pose, over and over (23:07 session:
        # three 25 s route-around timeouts back to back, robot frozen).
        # Direct pursuit keeps the intimate standoff; the planner gets a
        # goal it can actually reach and pursuit closes the rest.
        planner_standoff = max(self.p['standoff_m'],
                               self.p['planner_goal_clearance_m'])
        scale = max(0.0, (distance - planner_standoff) / distance)
        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.pose[0] + dx * scale
        pose.pose.position.y = self.pose[1] + dy * scale
        yaw = math.atan2(dy, dx)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        # retry_after was SET on every abort but never READ — the
        # 'backing off 2.5 s' was decorative, and an unplannable search
        # goal re-fired at tick rate (3 full cycles in 300 ms, 23:26
        # session). Central enforcement: no maneuver starts during the
        # backoff window.
        if (self.retry_after is not None
                and self.get_clock().now() < self.retry_after):
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose                # default BT: compute once, execute
        self.goal_pending = True
        self.mode = mode
        self.reorient_started = self.get_clock().now().nanoseconds / 1e9
        future = self.navigator.send_goal_async(goal)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future):
        self.goal_pending = False
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warning('maneuver goal rejected')
            self.mode = 'chase'
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self._result)

    def _result(self, future):
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
        if self.mode == 'search':
            if status == GoalStatus.STATUS_ABORTED:
                # Planner failed (e.g. smeared costmap): allow another try
                # after the backoff while the sighting is still fresh.
                self.get_logger().info('search aborted; will retry')
            else:
                self.search_done = True
                self.get_logger().info(
                    'search finished; holding for the person')
        if self.mode in ('search', 'avoid'):
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
