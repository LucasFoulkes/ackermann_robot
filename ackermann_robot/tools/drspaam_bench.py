#!/usr/bin/env python3
"""DR-SPAAM offline benchmark: score the stage-2 person classifier
against recorded failure bags BEFORE it costs the robot anything.

For every /scan in a bag, run the DR-SPAAM detector and measure wall
time (the Pi budget answer). Cross-reference detections with the
tracker debug stream: each tracked id gets a person-score = fraction
of its lifetime a DR-SPAAM detection sat within `match_radius` of it.
Confirmed tracks with low person-scores are the ghosts stage-1 could
not tell apart; the followed id's score is the headline.

Run inside the drspaam venv:
  ~/venvs/drspaam/bin/python drspaam_bench.py <bag_dir> <ckpt.pth>
"""

import json
import math
import sys
import time

import numpy as np
import rclpy  # noqa: F401  (message deserialization context)
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

MATCH_RADIUS = 0.5
LASER_X = 0.10  # base_link -> laser, from the birth certificate


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return
    bag, ckpt = sys.argv[1], sys.argv[2]
    from dr_spaam.detector import Detector
    detector = Detector(ckpt, model='DR-SPAAM', gpu=False, stride=1,
                        panoramic_scan=True)
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id=''),
                ConverterOptions('', ''))
    pose = None
    fov_set = False
    times = []
    detections = []            # (stamp, [(odom_x, odom_y), ...])
    debug_frames = []
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic == '/odom':
            m = deserialize_message(raw, Odometry)
            pose = (m.pose.pose.position.x, m.pose.pose.position.y,
                    yaw_from_quaternion(m.pose.pose.orientation))
        elif topic == '/person_tracker/debug':
            debug_frames.append(json.loads(
                deserialize_message(raw, String).data))
        elif topic == '/scan' and pose is not None:
            m = deserialize_message(raw, LaserScan)
            if not fov_set:
                detector.set_laser_fov(
                    math.degrees(m.angle_max - m.angle_min))
                fov_set = True
            ranges = np.nan_to_num(
                np.asarray(m.ranges, dtype=np.float32),
                nan=m.range_max, posinf=m.range_max)
            t0 = time.perf_counter()
            dets_xy, dets_cls, _ = detector(ranges)
            times.append(time.perf_counter() - t0)
            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            px, py, pyaw = pose
            sx = px + LASER_X * math.cos(pyaw)
            sy = py + LASER_X * math.sin(pyaw)
            world = []
            for (dx, dy), conf in zip(dets_xy, dets_cls):
                if conf < 0.5:
                    continue
                # detector frame: x forward, y left of the laser
                wx = sx + dx * math.cos(pyaw) - dy * math.sin(pyaw)
                wy = sy + dx * math.sin(pyaw) + dy * math.cos(pyaw)
                world.append((wx, wy))
            detections.append((stamp, world))
    print(f'\n=== timing on this machine ===')
    if times:
        times_ms = sorted(t * 1e3 for t in times)
        print(f'{len(times)} scans: median {times_ms[len(times_ms)//2]:.0f} '
              f'ms, p90 {times_ms[int(0.9 * len(times_ms))]:.0f} ms '
              f'(scan period is 100 ms)')
    # person-score per tracked id
    det_index = 0
    scores = {}
    for frame in debug_frames:
        while (det_index + 1 < len(detections) and
               detections[det_index + 1][0] <= frame['stamp']):
            det_index += 1
        world = detections[det_index][1] if detections else []
        for track in frame['tracks']:
            entry = scores.setdefault(
                track['id'],
                {'frames': 0, 'matched': 0,
                 'confirmed': track['confirmed'], 'followed': 0})
            entry['frames'] += 1
            entry['confirmed'] |= track['confirmed']
            if track['id'] == frame['followed_id']:
                entry['followed'] += 1
            if any(math.hypot(track['x'] - wx, track['y'] - wy)
                   < MATCH_RADIUS for wx, wy in world):
                entry['matched'] += 1
    print('\n=== person-score per track (confirmed or followed) ===')
    rows = [(tid, e) for tid, e in scores.items()
            if e['confirmed'] or e['followed']]
    rows.sort(key=lambda item: -item[1]['followed'])
    for tid, e in rows[:20]:
        score = e['matched'] / max(e['frames'], 1)
        print(f"  id {tid}: person-score {score:.2f} over {e['frames']} "
              f"frames, followed {e['followed']} frames"
              f"{' CONFIRMED' if e['confirmed'] else ''}")
    print('\nverdict guide: followed ids with score < 0.3 are the ghosts '
          'stage-1 could not distinguish; ids > 0.7 are the human.')


if __name__ == '__main__':
    main()
