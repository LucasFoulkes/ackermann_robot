#!/usr/bin/env python3
"""Offline person-tracker replay: run recorded sensor data through the
tracker without a robot or a room.

Feeds a bag's /scan + /odom chronologically into a fresh PersonTracker
instance (optionally with parameter overrides) and reports every birth,
confirmation, follow switch, and the final track table. Iterate on
tracker parameters against the EXACT failure that hurt you, from a desk.

Usage:
  tracker_replay.py <bag_dir> [param=value ...]
  tracker_replay.py ~/.robot/bags/adaptive_drive_20260715_201500 \
      min_travel_confirm_m=0.55 ego_calm_speed_mps=0.3
"""

import sys

import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from person_tracker.person_tracker_node import PersonTracker


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    bag = sys.argv[1]
    overrides = {}
    for item in sys.argv[2:]:
        key, value = item.split('=', 1)
        overrides[key] = float(value)
    rclpy.init()
    node = PersonTracker()
    for key, value in overrides.items():
        if key not in node.p:
            raise SystemExit(f'unknown tracker parameter: {key}')
        node.p[key] = value
        print(f'override: {key} = {value}')
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id=''),
                ConverterOptions('', ''))
    scans = 0
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic == '/odom':
            node._odom(deserialize_message(raw, Odometry))
        elif topic == '/scan':
            node._scan(deserialize_message(raw, LaserScan))
            scans += 1
    print(f'\n=== replay complete: {scans} scans ===')
    print('final tracks:')
    for t in node.tracks:
        print(f'  id {t.id}: confirmed={t.confirmed} travel={t.travel:.2f} '
              f'speed={t.speed:.2f} missed={t.missed}')
    print(f'followed_id: {node.followed_id}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
