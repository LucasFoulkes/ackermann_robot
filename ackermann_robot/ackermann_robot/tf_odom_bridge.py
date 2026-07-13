#!/usr/bin/env python3
"""Bridge MOLA's odom->base_link TF into a nav_msgs/Odometry topic.

MOLA lidar odometry publishes TF only; the rest of the stack (adaptive
controller, dispatcher, Nav2) consumes a plain /odom topic. This node is the
vehicle-specific glue that used to live inside the adaptive controller —
extracted so the controller is a pure Odometry consumer and any pose source
(wheel encoders, VIO, another SLAM) can replace this bridge unchanged.

Twist is derived from consecutive stamped poses with the same median filter
the controller historically published, so downstream consumers keep the
exact /odom contract they had before the split.
"""

import math
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_difference(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class TfOdomBridge(Node):
    def __init__(self):
        super().__init__('tf_odom_bridge')
        defaults = {
            'parent_frame': 'odom', 'child_frame': 'base_link',
            'odom_topic': '/odom', 'velocity_median_samples': 3,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        samples = int(self.p['velocity_median_samples'])
        self.speed_history = deque(maxlen=samples)
        self.yaw_rate_history = deque(maxlen=samples)
        self.pose = None
        self.last_stamp = None
        self.odom_pub = self.create_publisher(Odometry, self.p['odom_topic'],
                                              10)
        self.create_subscription(TFMessage, '/tf', self._tf, 50)

    def _tf(self, message):
        for transform in message.transforms:
            if (transform.header.frame_id.lstrip('/')
                    != self.p['parent_frame']
                    or transform.child_frame_id.lstrip('/')
                    != self.p['child_frame']):
                continue
            stamp = (transform.header.stamp.sec
                     + transform.header.stamp.nanosec * 1e-9)
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            yaw = yaw_from_quaternion(transform.transform.rotation)
            speed, yaw_rate = 0.0, 0.0
            if self.pose is not None and stamp > self.last_stamp:
                dt = stamp - self.last_stamp
                dx, dy = x - self.pose[0], y - self.pose[1]
                forward = (math.cos(self.pose[2]) * dx
                           + math.sin(self.pose[2]) * dy)
                self.speed_history.append(forward / dt)
                self.yaw_rate_history.append(
                    angle_difference(yaw, self.pose[2]) / dt)
            if self.speed_history:
                speed = sorted(self.speed_history)[
                    len(self.speed_history) // 2]
                yaw_rate = sorted(self.yaw_rate_history)[
                    len(self.yaw_rate_history) // 2]
            if self.pose is None or stamp > self.last_stamp:
                self.pose = (x, y, yaw)
                self.last_stamp = stamp
            odom = Odometry()
            odom.header = transform.header
            odom.child_frame_id = transform.child_frame_id
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.position.z = transform.transform.translation.z
            odom.pose.pose.orientation = transform.transform.rotation
            odom.twist.twist.linear.x = speed
            odom.twist.twist.angular.z = yaw_rate
            self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = TfOdomBridge()
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
