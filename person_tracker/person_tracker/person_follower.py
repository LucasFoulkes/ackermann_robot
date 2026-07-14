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
            'max_speed_mps': 0.40,
            'speed_gain': 0.8,          # m/s per metre of distance error
            'max_curvature_1pm': 1.1,
            # |bearing| above this: crawl on a max-curvature arc instead of
            # driving fast while pointing the wrong way.
            'hard_turn_bearing_rad': 0.9,
            'crawl_speed_mps': 0.14,
            'lost_timeout_s': 2.0,
            'control_rate_hz': 10.0,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}

        self.person = None
        self.person_stamp = None
        self.pose = None
        self.was_commanding = False

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
        dx = self.person.pose.position.x - self.pose[0]
        dy = self.person.pose.position.y - self.pose[1]
        distance = math.hypot(dx, dy)
        error = distance - self.p['desired_distance_m']
        if error < self.p['arrive_tolerance_m']:
            self._stop()
            return
        bearing = wrap(math.atan2(dy, dx) - self.pose[2])
        # Pure pursuit on the person: arc whose chord ends at them.
        curvature = 2.0 * math.sin(bearing) / max(distance, 0.3)
        curvature = max(-self.p['max_curvature_1pm'],
                        min(self.p['max_curvature_1pm'], curvature))
        if abs(bearing) > self.p['hard_turn_bearing_rad']:
            speed = self.p['crawl_speed_mps']
            curvature = math.copysign(self.p['max_curvature_1pm'], bearing)
        else:
            speed = min(self.p['max_speed_mps'],
                        self.p['speed_gain'] * error)
            # Slow down while pointing away; full speed when aligned.
            speed *= max(0.3, math.cos(bearing))
        command = Twist()
        command.linear.x = float(max(0.0, speed))
        command.angular.z = float(command.linear.x * curvature)
        self.command_pub.publish(command)
        self.was_commanding = True


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
