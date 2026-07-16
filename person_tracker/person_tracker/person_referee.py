#!/usr/bin/env python3
"""Neural person referee: low-rate DR-SPAAM detections for the tracker.

Runs the pretrained leg classifier over every Nth scan and publishes
odom-frame person detections as JSON on /person_referee/detections.
The tracker uses them to RANK tracks (person-score), which is the one
judgment geometry cannot make: a static human vs static furniture.
Benchmarked offline 2026-07-16: ranks the human 4-10x above ghosts at
this lidar height; ~0.5 s/scan on the Pi at stride 2, hence the skip.

NOT a colcon entry point: torch lives in the drspaam venv, so the
bringup launches this script with that interpreter:
  ~/venvs/drspaam/bin/python person_referee.py [ckpt] [scan_skip]
"""

import json
import math
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

DEFAULT_CKPT = os.path.expanduser(
    '~/opt/drspaam_ckpts/self_supervised_person_detection/'
    'ckpt_jrdb_ann_ft_dr_spaam_e20.pth')
LASER_X = 0.10
CONF_MIN = 0.30


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PersonReferee(Node):
    def __init__(self, ckpt, scan_skip):
        super().__init__('person_referee')
        import torch
        _orig = torch.load
        torch.load = lambda *a, **k: _orig(
            *a, **{**k, 'map_location': 'cpu'})
        from dr_spaam.detector import Detector
        self.detector = Detector(ckpt, model='DR-SPAAM', gpu=False,
                                 stride=2, panoramic_scan=True)
        self.scan_skip = scan_skip
        self.fov_set = False
        self.pose = None
        self.count = 0
        qos = QoSProfile(depth=2)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, '/scan', self._scan, qos)
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.pub = self.create_publisher(
            String, '/person_referee/detections', 2)
        self.get_logger().info(
            f'person referee up: DR-SPAAM every {scan_skip}th scan, '
            f'conf >= {CONF_MIN} (ranking signal, not ground truth)')

    def _odom(self, msg):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     yaw_from_quaternion(msg.pose.pose.orientation))

    def _scan(self, msg):
        self.count += 1
        if self.count % self.scan_skip or self.pose is None:
            return
        if not self.fov_set:
            self.detector.set_laser_fov(
                math.degrees(msg.angle_max - msg.angle_min))
            self.fov_set = True
        ranges = np.nan_to_num(
            np.asarray(msg.ranges, dtype=np.float32),
            nan=msg.range_max, posinf=msg.range_max)
        dets_xy, dets_cls, _ = self.detector(ranges)
        px, py, pyaw = self.pose
        sx = px + LASER_X * math.cos(pyaw)
        sy = py + LASER_X * math.sin(pyaw)
        points = []
        for (dx, dy), conf in zip(dets_xy, dets_cls):
            if conf < CONF_MIN:
                continue
            points.append([
                round(sx + dx * math.cos(pyaw) - dy * math.sin(pyaw), 3),
                round(sy + dx * math.sin(pyaw) + dy * math.cos(pyaw), 3),
                round(float(conf), 3)])
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.pub.publish(String(data=json.dumps(
            {'stamp': stamp, 'points': points})))


def main():
    ckpt = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CKPT
    skip = int(sys.argv[2]) if len(sys.argv) > 2 else 10
    rclpy.init()
    node = PersonReferee(ckpt, skip)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
