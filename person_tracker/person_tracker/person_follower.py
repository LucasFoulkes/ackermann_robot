#!/usr/bin/env python3
"""Follow the confirmed person — hybrid (v3).

Two modes, each using the subsystem proven at that job:

CHASE (person inside the front cone): direct velocity streaming using the
Park–Kuipers graceful-motion control law — the same law Nav2's Following/
Docking servers use. Curvature from the egocentric polar error, speed
automatically collapsing with curvature demand. Reaction latency is one
10 Hz tick. Commands flow through the normal safety chain
(/cmd_vel_nav_raw -> adaptive limits -> collision monitor).

REORIENT (person far off-axis or behind): an Ackermann with a ~1.2 m
turning radius cannot swing its nose in place — forward-only chasing
orbits the room and hand-rolled K-turns are a poor rewrite of what Smac
Reeds-Shepp already does well. So: stream stops, ONE NavigateToPose goal
is sent to the standoff point (default compute-once BT, cusp dispatcher
executes the multi-point turn exactly like a normal goal), and the moment
the person is back in the front cone the goal is canceled and CHASE
resumes. No goal updates mid-maneuver -> no replan churn.
"""

import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry  # person topic carries velocity too
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')
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
            'max_speed_mps': 0.60,
            'speed_gain': 0.8,           # m/s per metre of distance error
            'max_curvature_1pm': 1.15,
            # Park-Kuipers law: kappa = (k2*bearing + sin(bearing)) / r,
            # v = vmax / (1 + beta*|kappa|^lambda) — speed collapses as
            # curvature demand rises (graceful approach, no orbiting).
            'k2': 2.0,
            'beta': 0.4,
            'lambda': 2.0,
            # Single bidirectional law: person in the front hemisphere ->
            # nose-first pursuit; rear hemisphere -> TAIL-FIRST pursuit
            # (back toward them like a car to a hitch). Hysteresis on the
            # hemisphere choice only; no modes, no frozen maneuver goals.
            'hemisphere_to_reverse_rad': 1.92,   # |bearing| beyond -> back up
            'hemisphere_to_forward_rad': 1.45,   # |bearing| within -> nose-first
            'max_reverse_speed_mps': 0.45,
            'reorient_timeout_s': 25.0,
            'retry_backoff_s': 2.5,
            'standoff_m': 0.5,
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
        self.drive_direction = 1          # +1 nose-first, -1 tail-first
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
        self.command_pub = self.create_publisher(
            Twist, self.p['command_topic'], 10)
        self.navigator = ActionClient(self, NavigateToPose,
                                      'navigate_to_pose')
        self.create_timer(1.0 / self.p['control_rate_hz'], self._tick)
        self.get_logger().info(
            'person_follower v3 (hybrid): graceful-law chase in the front '
            'cone, one-shot Smac maneuver to reorient')

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
                    < self.p['search_max_age_s']):
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
                command.linear.x = (-self.drive_direction
                                    * self.p['escape_speed_mps'])
                self.command_pub.publish(command)
                self.was_commanding = True
                return
            self.escape_until = None
            self.stalled_since = None
        # Hemisphere with hysteresis: nose-first vs tail-first.
        if self.drive_direction > 0:
            if abs(bearing) > self.p['hemisphere_to_reverse_rad']:
                self.drive_direction = -1
                self.get_logger().info('person behind: following tail-first')
        else:
            if abs(bearing) < self.p['hemisphere_to_forward_rad']:
                self.drive_direction = 1
                self.get_logger().info('person ahead: following nose-first')
        # Park-Kuipers graceful law (theta=0), applied to whichever end
        # leads: tail-first uses the person's bearing FROM THE TAIL and the
        # mirrored curvature sign (omega = v * kappa with v < 0).
        if self.drive_direction > 0:
            effective_bearing = bearing
        else:
            effective_bearing = wrap(bearing - math.pi)
        curvature = (self.p['k2'] * effective_bearing
                     + math.sin(effective_bearing)) / max(distance, 0.3)
        if self.drive_direction < 0:
            curvature = -curvature
        curvature = max(-self.p['max_curvature_1pm'],
                        min(self.p['max_curvature_1pm'], curvature))
        top = (self.p['max_speed_mps'] if self.drive_direction > 0
               else self.p['max_reverse_speed_mps'])
        speed = top / (
            1.0 + self.p['beta'] * abs(curvature) ** self.p['lambda'])
        # Speed matching: move at the person's pace plus a catch-up term.
        speed = min(speed,
                    self.person_speed + self.p['speed_gain'] * error)
        command = Twist()
        command.linear.x = float(self.drive_direction * max(0.0, speed))
        command.angular.z = float(command.linear.x * curvature)
        self.command_pub.publish(command)
        self.was_commanding = True
        if abs(command.linear.x) > 0.05 and self.robot_speed < 0.03:
            if self.stalled_since is None:
                self.stalled_since = seconds
            elif seconds - self.stalled_since > self.p['stall_escape_after_s']:
                self.stalled_since = None
                # Pursuit is blocked: hand it to Smac IMMEDIATELY — the
                # planner sees the costmap and routes around, while blind
                # pursuit can only bump the obstacle gate (looked like
                # 'avoidance not working' in live testing).
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
        scale = max(0.0, (distance - self.p['standoff_m']) / distance)
        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.pose[0] + dx * scale
        pose.pose.position.y = self.pose[1] + dy * scale
        yaw = math.atan2(dy, dx)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
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
