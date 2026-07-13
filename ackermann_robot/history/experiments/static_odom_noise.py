#!/usr/bin/env python3
import argparse
import math
import statistics
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def get_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def angle_difference(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def describe(values):
    if not values:
        return 'no samples'
    mean = statistics.fmean(values)
    std = statistics.stdev(values) if len(values) > 1 else 0.0
    rms = math.sqrt(statistics.fmean(v * v for v in values))
    peak = max(abs(v) for v in values)
    return f'mean={mean:+.6f} std={std:.6f} rms={rms:.6f} peak={peak:.6f}'


class Monitor(Node):
    def __init__(self, args):
        super().__init__('static_odom_noise')
        self.args = args
        self.start_time = time.monotonic()
        self.source = None
        self.poses = []
        self.vx, self.vy, self.wz = [], [], []
        self.last_tf = None
        self.create_subscription(Odometry, args.odom_topic, self.odom_cb, 50)
        self.create_subscription(TFMessage, '/tf', self.tf_cb, 100)

    def odom_cb(self, msg):
        self.source = 'Odometry topic'
        p, t = msg.pose.pose, msg.twist.twist
        self.poses.append((p.position.x, p.position.y, get_yaw(p.orientation)))
        self.vx.append(t.linear.x)
        self.vy.append(t.linear.y)
        self.wz.append(t.angular.z)

    def tf_cb(self, msg):
        if self.source == 'Odometry topic':
            return
        for tf in msg.transforms:
            if (tf.header.frame_id.lstrip('/') != self.args.parent_frame or
                    tf.child_frame_id.lstrip('/') != self.args.child_frame):
                continue
            self.source = 'TF (velocity derived by finite differences)'
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            p = tf.transform.translation
            heading = get_yaw(tf.transform.rotation)
            self.poses.append((p.x, p.y, heading))
            if self.last_tf:
                old_stamp, old_x, old_y, old_heading = self.last_tf
                dt = stamp - old_stamp
                if dt > 0:
                    self.vx.append((p.x - old_x) / dt)
                    self.vy.append((p.y - old_y) / dt)
                    self.wz.append(angle_difference(heading, old_heading) / dt)
            self.last_tf = stamp, p.x, p.y, heading

    def report(self):
        print(f'\nStatic noise measurement: {self.args.duration:.1f} seconds')
        print(f'Source: {self.source or "none"}; pose samples: {len(self.poses)}')
        if not self.poses:
            print(f'No {self.args.odom_topic} or {self.args.parent_frame} -> '
                  f'{self.args.child_frame} data received.')
            return
        x0, y0, h0 = self.poses[0]
        offsets = [(x - x0, y - y0, angle_difference(h, h0))
                   for x, y, h in self.poses]
        dx, dy, dh = offsets[-1]
        max_position = max(math.hypot(x, y) for x, y, _ in offsets)
        print(f'Final drift: {math.hypot(dx, dy):.6f} m, '
              f'{math.degrees(dh):+.4f} deg')
        print(f'Maximum position excursion: {max_position:.6f} m')
        print(f'vx [m/s]:   {describe(self.vx)}')
        print(f'vy [m/s]:   {describe(self.vy)}')
        print(f'wz [rad/s]: {describe(self.wz)}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--parent-frame', default='odom')
    parser.add_argument('--child-frame', default='base_link')
    args, ros_args = parser.parse_known_args()
    args.parent_frame = args.parent_frame.lstrip('/')
    args.child_frame = args.child_frame.lstrip('/')
    rclpy.init(args=ros_args)
    node = Monitor(args)
    try:
        while rclpy.ok() and time.monotonic() - node.start_time < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.report()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
