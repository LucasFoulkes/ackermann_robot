#!/usr/bin/env python3
"""Follow the confirmed person with a direct pursuit law (v2).

v1 used the Nav2 BT approach (GoalUpdater + TruncatePath + replanning).
The docs recommend a dedicated Following Server for MOVING targets — a
direct smooth control law with no global planning — but opennav_following
is not released for Jazzy, so this node implements the same pattern:

  person pose (10 Hz) -> pure-pursuit arc toward the person
                      -> speed proportional to (distance - desired)
                      -> Twist into the EXISTING safety chain

Publishing on /cmd_vel_nav_raw means every layer that guards Nav2 driving
also guards following: the adaptive controller clamps speed/curvature and
applies steering-transition slowdown, Collision Monitor gates the result,
and the launch ladder / obstacle stops behave exactly as in normal goals.
No Smac, no cusp dispatcher, no behavior tree: reaction latency is one
control tick instead of a replanning cycle.

Ackermann reality: the robot cannot turn in place. A person far off-axis
(or behind) is approached with a maximum-curvature forward arc at reduced
speed; if there is no room to arc, the collision monitor holds it and the
robot waits for the person to come around.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
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
            'max_speed_mps': 0.60,
            'speed_gain': 0.8,          # m/s per metre of distance error
            'max_curvature_1pm': 1.1,
            # |bearing| above this: K-turn (alternating fwd/rev arcs with
            # opposite steering — a 3-point turn) instead of orbiting the
            # room on a forward-only max-curvature circle.
            'hard_turn_bearing_rad': 0.9,
            # ...and below this the K-turn hands back to normal pursuit.
            'kturn_exit_bearing_rad': 0.5,
            'kturn_phase_s': 1.5,
            'crawl_speed_mps': 0.18,
            'lost_timeout_s': 2.0,
            'control_rate_hz': 10.0,
            # Commanding forward but not moving this long (arc jammed into
            # an obstacle / gate hold) -> back straight up briefly to open
            # the front, then retry. Ackermann has no other escape.
            'stall_escape_after_s': 3.0,
            'escape_duration_s': 2.0,
            'escape_speed_mps': 0.14,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}

        self.person = None
        self.person_stamp = None
        self.pose = None
        self.robot_speed = 0.0
        self.was_commanding = False
        self.stalled_since = None
        self.escape_until = None
        self.kturn_phase = None       # None | 'fwd' | 'rev'
        self.kturn_phase_until = 0.0

        self.create_subscription(PoseStamped, self.p['person_topic'],
                                 self._person, 10)
        self.create_subscription(Odometry, '/odom', self._odom, 20)
        self.command_pub = self.create_publisher(
            Twist, self.p['command_topic'], 10)
        self.create_timer(1.0 / self.p['control_rate_hz'], self._tick)
        self.get_logger().info(
            'person_follower v2 (direct pursuit): desired distance '
            f"{self.p['desired_distance_m']} m, reacting at "
            f"{self.p['control_rate_hz']:.0f} Hz through the normal "
            'safety chain')

    def _person(self, msg):
        self.person = msg
        self.person_stamp = self.get_clock().now()

    def _odom(self, msg):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     yaw_from_quaternion(msg.pose.pose.orientation))
        self.robot_speed = abs(msg.twist.twist.linear.x)

    def _stop(self):
        if self.was_commanding:
            self.command_pub.publish(Twist())
            self.was_commanding = False
            self.get_logger().info('holding (person lost or at distance)')

    def _tick(self):
        now = self.get_clock().now()
        fresh = (self.person_stamp is not None and
                 (now - self.person_stamp).nanoseconds / 1e9
                 < self.p['lost_timeout_s'])
        if not fresh or self.pose is None:
            self._stop()
            return
        seconds = now.nanoseconds / 1e9
        if self.escape_until is not None:
            if seconds < self.escape_until:
                command = Twist()
                command.linear.x = -self.p['escape_speed_mps']
                self.command_pub.publish(command)
                self.was_commanding = True
                return
            self.escape_until = None
            self.stalled_since = None
        dx = self.person.pose.position.x - self.pose[0]
        dy = self.person.pose.position.y - self.pose[1]
        distance = math.hypot(dx, dy)
        error = distance - self.p['desired_distance_m']
        if error < self.p['arrive_tolerance_m']:
            self._stop()
            return
        bearing = wrap(math.atan2(dy, dx) - self.pose[2])
        # --- K-turn (3-point turn) when the person is far off-axis ---
        # Forward arc steered toward them, then reverse arc steered away:
        # both phases rotate the nose the SAME direction, turning around in
        # the robot's own footprint instead of orbiting the room.
        if self.kturn_phase is None:
            if abs(bearing) > self.p['hard_turn_bearing_rad']:
                self.kturn_phase = 'fwd'
                self.kturn_phase_until = seconds + self.p['kturn_phase_s']
                self.get_logger().info(
                    f'K-turn: person at {math.degrees(bearing):+.0f} deg')
        elif abs(bearing) < self.p['kturn_exit_bearing_rad']:
            self.kturn_phase = None
            self.get_logger().info('K-turn done; resuming pursuit')
        if self.kturn_phase is not None:
            if seconds >= self.kturn_phase_until:
                self.kturn_phase = ('rev' if self.kturn_phase == 'fwd'
                                    else 'fwd')
                self.kturn_phase_until = seconds + self.p['kturn_phase_s']
            sign = 1.0 if bearing > 0 else -1.0
            command = Twist()
            if self.kturn_phase == 'fwd':
                command.linear.x = self.p['crawl_speed_mps']
                kappa = sign * self.p['max_curvature_1pm']
            else:
                command.linear.x = -self.p['crawl_speed_mps']
                kappa = -sign * self.p['max_curvature_1pm']
            command.angular.z = command.linear.x * kappa
            self.command_pub.publish(command)
            self.was_commanding = True
            self.stalled_since = None
            return
        # --- normal pursuit: arc whose chord ends at the person ---
        curvature = 2.0 * math.sin(bearing) / max(distance, 0.3)
        curvature = max(-self.p['max_curvature_1pm'],
                        min(self.p['max_curvature_1pm'], curvature))
        speed = min(self.p['max_speed_mps'], self.p['speed_gain'] * error)
        # Slow down while pointing away; full speed when aligned.
        speed *= max(0.3, math.cos(bearing))
        command = Twist()
        command.linear.x = float(max(0.0, speed))
        command.angular.z = float(command.linear.x * curvature)
        self.command_pub.publish(command)
        self.was_commanding = True
        # Jam detection: forward commanded, robot pinned (obstacle gate or
        # blocked arc) -> reverse briefly to reopen the front.
        if command.linear.x > 0.05 and self.robot_speed < 0.03:
            if self.stalled_since is None:
                self.stalled_since = seconds
            elif seconds - self.stalled_since > self.p['stall_escape_after_s']:
                self.escape_until = seconds + self.p['escape_duration_s']
                self.get_logger().info(
                    'front jammed: reversing to open space, then retrying')
        else:
            self.stalled_since = None


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Leave a zero command; the adaptive controller's command watchdog
        # holds neutral once we stop publishing.
        try:
            node.command_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
