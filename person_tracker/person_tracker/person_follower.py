#!/usr/bin/env python3
"""Close the loop: follow the (single) confirmed person.

Consumes the tracker's best-person pose, runs one NavigateToPose action
with the dynamic-following behavior tree (GoalUpdater + TruncatePath 0.9 m
standoff), and streams fresh person poses onto /goal_update. Person lost
longer than lost_timeout_s -> cancel the action, robot stops; person
re-confirmed -> re-engage. All Nav2 safety layers (collision monitor,
speed limit, adaptive controller gates) stay in the loop unchanged.
"""

import math
import os

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')
        default_bt = os.path.join(
            get_package_share_directory('person_tracker'),
            'config', 'follow_person.xml')
        defaults = {
            'follow_bt_xml': default_bt,
            'person_topic': '/person_tracker/person',
            'goal_update_topic': '/goal_update',
            'lost_timeout_s': 3.0,
            'update_period_s': 0.5,
            # Goal is planted this far SHORT of the person along the
            # robot->person line: the person's own legs are lethal obstacles
            # in the costmap, so a goal ON them can never be planned to
            # (Smac burned its whole iteration budget trying).
            'standoff_m': 0.5,
            # Person closer than this: do not chase, just hold position.
            'engage_min_distance_m': 1.4,
            # After an aborted follow, wait this long before re-engaging so
            # a persistent planning failure cannot loop at 2 Hz.
            'retry_backoff_s': 2.5,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}

        self.person = None
        self.person_stamp = None
        self.robot = None
        self.goal_handle = None
        self.goal_pending = False
        self.retry_after = None

        self.create_subscription(PoseStamped, self.p['person_topic'],
                                 self._person, 10)
        self.create_subscription(Odometry, '/odom', self._odom, 20)
        self.goal_pub = self.create_publisher(
            PoseStamped, self.p['goal_update_topic'], 10)
        self.navigator = ActionClient(self, NavigateToPose,
                                      'navigate_to_pose')
        self.create_timer(self.p['update_period_s'], self._tick)
        self.get_logger().info(
            f"person_follower up: standoff via BT truncate, "
            f"lost timeout {self.p['lost_timeout_s']} s")

    def _person(self, msg):
        self.person = msg
        self.person_stamp = self.get_clock().now()

    def _odom(self, msg):
        self.robot = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _goal_pose(self):
        """Free-space goal standoff_m short of the person, facing them.
        None when the person is already within the standoff."""
        if self.robot is None:
            return None
        px = self.person.pose.position.x
        py = self.person.pose.position.y
        dx, dy = px - self.robot[0], py - self.robot[1]
        distance = math.hypot(dx, dy)
        if distance < self.p['standoff_m'] + 0.15:
            return None
        scale = (distance - self.p['standoff_m']) / distance
        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.robot[0] + dx * scale
        pose.pose.position.y = self.robot[1] + dy * scale
        yaw = math.atan2(dy, dx)
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = (
            yaw_to_quaternion(yaw))
        return pose

    def _tick(self):
        now = self.get_clock().now()
        fresh = (self.person_stamp is not None and
                 (now - self.person_stamp).nanoseconds / 1e9
                 < self.p['lost_timeout_s'])
        active = self.goal_handle is not None or self.goal_pending
        if not fresh:
            if self.goal_handle is not None:
                self.get_logger().info('person lost -> stopping follow')
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None
            return
        if self.robot is not None:
            distance = math.hypot(
                self.person.pose.position.x - self.robot[0],
                self.person.pose.position.y - self.robot[1])
            if not active and distance < self.p['engage_min_distance_m']:
                return                       # already at standoff; wait
        goal = self._goal_pose()
        if goal is None:
            return          # inside standoff: let the running goal finish
        if not active:
            if (self.retry_after is None
                    or now >= self.retry_after):
                self._engage(goal)
        else:
            self.goal_pub.publish(goal)

    def _engage(self, pose):
        if not self.navigator.server_is_ready():
            self.get_logger().warning('navigate_to_pose not ready',
                                      throttle_duration_sec=5.0)
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.behavior_tree = os.path.expanduser(self.p['follow_bt_xml'])
        self.goal_pending = True
        self.get_logger().info('engaging: following person')
        future = self.navigator.send_goal_async(goal)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future):
        self.goal_pending = False
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warning('follow goal rejected')
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
            self.retry_after = self.get_clock().now() + rclpy.duration.Duration(
                seconds=self.p['retry_backoff_s'])
            self.get_logger().warning(
                'follow aborted; backing off '
                f"{self.p['retry_backoff_s']} s before re-engaging")


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
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
