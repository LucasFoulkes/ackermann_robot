#!/usr/bin/env python3
"""Causally evaluate learned path preview against future Nav2 commands in a bag."""

import argparse
import bisect
import math
import os
import random
import statistics

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from ackermann_robot.adaptive_model import PathGeometry, compose_preview_curvature
from ackermann_robot.adaptive_ackermann_controller import yaw_from_quaternion


def median(values):
    return statistics.median(values) if values else math.nan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('--runtime-model',
                        default='~/.robot/adaptive_ackermann_runtime.yaml')
    parser.add_argument('--wheelbase', type=float, default=.2775)
    parser.add_argument('--maximum-curvature', type=float, default=1.15)
    parser.add_argument('--lookahead-time', type=float, default=1.5)
    parser.add_argument('--minimum-lookahead', type=float, default=.30)
    parser.add_argument('--maximum-lookahead', type=float, default=.75)
    args = parser.parse_args()
    model_path = os.path.expanduser(args.runtime_model)
    delay = 0.0
    if os.path.exists(model_path):
        with open(model_path) as stream:
            model = yaml.safe_load(stream) or {}
        delay = float(model.get('estimated_steering_delay_s', 0.0))
    if delay <= 0.0:
        raise SystemExit('runtime model has no positive learned delay')

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    topic_types = {item.name: item.type
                   for item in reader.get_all_topics_and_types()}
    classes = {topic: get_message(topic_types[topic])
               for topic in ('/plan', '/odom', '/cmd_vel_nav')}
    plan = None
    pose = None
    commands = []
    observations = []
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic not in classes:
            continue
        msg = deserialize_message(data, classes[topic])
        stamp = stamp_ns * 1e-9
        if topic == '/plan':
            if msg.header.frame_id.lstrip('/') != 'odom' or len(msg.poses) < 3:
                plan = None
                continue
            plan = PathGeometry([
                (item.pose.position.x, item.pose.position.y,
                 yaw_from_quaternion(item.pose.orientation))
                for item in msg.poses])
        elif topic == '/odom':
            pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                    yaw_from_quaternion(msg.pose.pose.orientation))
        else:
            v, w = msg.linear.x, msg.angular.z
            commands.append((stamp, v, w))
            if plan is None or pose is None or abs(v) < .01:
                continue
            direction = 1 if v > 0.0 else -1
            index = plan.nearest_index(pose[0], pose[1])
            if plan.direction_at(index) != direction:
                continue
            lookahead = max(args.minimum_lookahead, min(
                args.maximum_lookahead, abs(v) * args.lookahead_time))
            current = plan.pure_pursuit_curvature(
                index, direction, lookahead)
            preview_index, _, _ = plan.advance(
                index, abs(v) * delay, direction)
            future = plan.pure_pursuit_curvature(
                preview_index, direction, lookahead)
            rpp = max(-args.maximum_curvature,
                      min(args.maximum_curvature, w / v))
            preview = compose_preview_curvature(
                rpp, current, future, 1.0, args.maximum_curvature)
            observations.append((stamp, direction, rpp, preview))

    command_times = [item[0] for item in commands]
    paired = []
    for stamp, direction, current, preview in observations:
        target = stamp + delay
        index = bisect.bisect_left(command_times, target)
        if index <= 0 or index >= len(commands):
            continue
        candidate = min((commands[index - 1], commands[index]),
                        key=lambda item: abs(item[0] - target))
        _, v, w = candidate
        if abs(v) < .01 or (1 if v > 0.0 else -1) != direction:
            continue
        future_command = max(-args.maximum_curvature,
                             min(args.maximum_curvature, w / v))
        paired.append((abs(current - future_command),
                       abs(preview - future_command)))
    if len(paired) < 20:
        raise SystemExit(f'insufficient paired commands: {len(paired)}')
    baseline = [item[0] for item in paired]
    previewed = [item[1] for item in paired]
    differences = [new - old for old, new in paired]
    rng = random.Random(0)
    bootstrap = []
    for _ in range(2000):
        sample = [differences[rng.randrange(len(differences))]
                  for _ in differences]
        bootstrap.append(median(sample))
    bootstrap.sort()
    lower = bootstrap[int(.025 * len(bootstrap))]
    upper = bootstrap[int(.975 * len(bootstrap))]
    base_p75 = statistics.quantiles(baseline, n=4)[2]
    preview_p75 = statistics.quantiles(previewed, n=4)[2]
    print(f'learned delay: {delay:.3f} s')
    print(f'paired commands: {len(paired)}')
    print(f'baseline future-command median error: {median(baseline):.4f} 1/m')
    print(f'preview future-command median error:  {median(previewed):.4f} 1/m')
    print(f'paired median change 95% CI: [{lower:.4f}, {upper:.4f}] 1/m')
    print(f'baseline/preview p75: {base_p75:.4f} / {preview_p75:.4f} 1/m')
    accepted = upper < 0.0 and preview_p75 <= base_p75
    print(f'preview command-prediction acceptance: {accepted}')


if __name__ == '__main__':
    main()
