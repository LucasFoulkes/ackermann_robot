#!/usr/bin/env python3
"""Identify steering pulse-to-curvature using short forward/reverse shuttles."""

import argparse
import csv
import math
import os
import statistics
import time

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from ackermann_robot.experiments.esc_forward_calibrate import (
    NEUTRAL,
    US_PER_TICK,
    neutral,
    open_bus,
    write_tick,
)
from ackermann_robot.experiments.esc_speed_control_test import interpolate, local_gain
from ackermann_robot.experiments.steering_sanity import (
    DEFAULT_STEERING_CHANNEL,
    release_channel,
    write_steering_us,
)


WHEELBASE_M = 0.2775


def angle_difference(new, old):
    return math.atan2(math.sin(new - old), math.cos(new - old))


class SteeringExperiment(Node):
    def __init__(self, args):
        super().__init__('steering_system_id')
        self.args = args
        self.pose = None
        self.previous_pose = None
        self.odom_time = None
        self.odom_seq = 0
        self.signed_speed = 0.0
        self.velocity_history = []
        self.last_ds = 0.0
        self.last_dyaw = 0.0
        self.last_dt = 0.0
        self.front = math.inf
        self.rear = math.inf
        self.scan_time = None
        self.direction = 'forward'
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 20)

    def on_tf(self, msg):
        for transform in msg.transforms:
            if (transform.header.frame_id.lstrip('/') != 'odom' or
                    transform.child_frame_id.lstrip('/') != 'base_link'):
                continue
            stamp = (transform.header.stamp.sec +
                     transform.header.stamp.nanosec * 1e-9)
            p = transform.transform.translation
            q = transform.transform.rotation
            yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y * q.y + q.z * q.z),
            )
            current = (stamp, p.x, p.y, yaw)
            self.last_ds = self.last_dyaw = 0.0
            self.last_dt = 0.0
            if self.previous_pose and stamp > self.previous_pose[0]:
                old = self.previous_pose
                dt = stamp - old[0]
                dx, dy = p.x - old[1], p.y - old[2]
                dyaw = angle_difference(yaw, old[3])
                mid_yaw = old[3] + 0.5 * dyaw
                ds = dx * math.cos(mid_yaw) + dy * math.sin(mid_yaw)
                instantaneous_speed = ds / dt
                self.signed_speed = (
                    0.75 * self.signed_speed + 0.25 * instantaneous_speed
                )
                self.last_ds = ds
                self.last_dyaw = dyaw
                self.last_dt = dt
                self.velocity_history.append(self.signed_speed)
                del self.velocity_history[:-500]
            self.previous_pose = self.pose = current
            self.odom_seq += 1
            self.odom_time = time.monotonic()

    def on_scan(self, msg):
        front, rear = [], []
        for index, distance in enumerate(msg.ranges):
            if (not math.isfinite(distance) or
                    not msg.range_min <= distance <= msg.range_max):
                continue
            angle = msg.angle_min + index * msg.angle_increment
            if abs(angle) <= math.radians(35):
                front.append(distance)
            if abs(abs(angle) - math.pi) <= math.radians(35):
                rear.append(distance)
        self.front = min(front, default=math.inf)
        self.rear = min(rear, default=math.inf)
        self.scan_time = time.monotonic()

    def safe(self, leg_motion):
        if self.odom_time is None or time.monotonic() - self.odom_time > 0.5:
            return False, 'odometry missing/stale'
        if self.scan_time is None or time.monotonic() - self.scan_time > 0.5:
            return False, 'lidar scan missing/stale'
        clearance = self.front if self.direction == 'forward' else self.rear
        if clearance < self.args.stop_range:
            return False, f'{self.direction} obstacle at {clearance:.2f} m'
        if leg_motion >= self.args.max_leg_distance:
            return False, f'leg distance limit {leg_motion:.2f} m'
        if abs(self.signed_speed) >= self.args.max_speed:
            return False, f'overspeed {self.signed_speed:+.3f} m/s'
        return True, ''

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.03)


def load_throttle_model(path):
    path = os.path.expanduser(path)
    if not os.path.exists(path):
        return {}, path
    with open(path) as model_file:
        return yaml.safe_load(model_file) or {}, path


def fitted_center(summaries):
    points = [(row['pulse_us'], row['curvature_1pm']) for row in summaries
              if row['valid']]
    if len(points) < 3:
        return None, None
    x_mean = statistics.fmean(x for x, _ in points)
    y_mean = statistics.fmean(y for _, y in points)
    denominator = sum((x - x_mean) ** 2 for x, _ in points)
    if denominator <= 0:
        return None, None
    slope = sum((x - x_mean) * (y - y_mean) for x, y in points) / denominator
    if abs(slope) < 1e-6:
        return None, slope
    return x_mean - y_mean / slope, slope


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm', action='store_true')
    parser.add_argument('--channel', type=int, default=DEFAULT_STEERING_CHANNEL)
    parser.add_argument('--center-us', type=float, default=1500.0)
    parser.add_argument('--test-offset-us', type=float, default=20.0)
    parser.add_argument('--approach-offset-us', type=float, default=30.0)
    parser.add_argument(
        '--interior-approach-only', action='store_true',
        help='at extreme targets, approach only from toward center',
    )
    parser.add_argument('--target-speed', type=float, default=0.20)
    parser.add_argument('--leg-distance', type=float, default=0.45)
    parser.add_argument('--max-leg-distance', type=float, default=0.65)
    parser.add_argument('--leg-timeout', type=float, default=8.0)
    parser.add_argument('--max-speed', type=float, default=0.40)
    parser.add_argument('--stop-range', type=float, default=0.20)
    parser.add_argument('--kp', type=float, default=0.0,
                        help='override derived throttle P gain; 0=auto')
    parser.add_argument(
        '--ki', type=float, default=30.0,
        help='fresh-odometry integral trim gain in us/(m/s*s)',
    )
    parser.add_argument('--model', default='~/.robot/esc_model.yaml')
    parser.add_argument('--output', default='steering_system_id.csv')
    parser.add_argument('--summary-output',
                        default='steering_system_id_summary.csv')
    args, ros_args = parser.parse_known_args()

    if not args.arm:
        parser.error('refusing hardware output without --arm')
    if not 0 <= args.channel <= 15:
        parser.error('--channel must be 0..15')
    if not 1400.0 <= args.center_us <= 1600.0:
        parser.error('--center-us must be within 1400..1600')
    if not 10.0 <= args.test_offset_us <= 270.0:
        parser.error('--test-offset-us must be 10..270')
    if args.test_offset_us > 220.0 and not args.interior_approach_only:
        parser.error('offsets above 220 require --interior-approach-only')
    if not 15.0 <= args.approach_offset_us <= 50.0:
        parser.error('--approach-offset-us must be 15..50')
    if not 0.15 <= args.target_speed <= 0.25:
        parser.error('--target-speed must be 0.15..0.25 m/s')
    if not 0.0 <= args.ki <= 60.0:
        parser.error('--ki must be 0..60 us/(m/s*s)')
    if not 0.25 <= args.leg_distance <= 0.60:
        parser.error('--leg-distance must be 0.25..0.60 m')
    if args.max_leg_distance < args.leg_distance:
        parser.error('--max-leg-distance must be at least --leg-distance')

    targets = [
        args.center_us,
        args.center_us - args.test_offset_us,
        args.center_us + args.test_offset_us,
    ]
    # Earlier hardware operation reached roughly 1137..1860 us. Keep the
    # commanded targets inside 1180..1720. A later 1740 us test caused an
    # unrecoverable drivetrain stall and is therefore outside the valid range.
    # widest range, approach each extreme from the interior so hysteresis
    # setup never commands closer to a mechanical endpoint than the target.
    if min(targets) < 1180.0 or max(targets) > 1720.0:
        parser.error('steering target exceeds the validated 1180..1720 envelope')
    if not args.interior_approach_only:
        if min(targets) - args.approach_offset_us < 1200.0:
            parser.error('lower steering anchor exceeds conservative envelope')
        if max(targets) + args.approach_offset_us > 1700.0:
            parser.error('higher steering anchor exceeds conservative envelope')

    throttle_maps = {
        'forward': [(.20, 1419.0), (.24, 1417.4), (.25, 1417.0)],
        'reverse': [(.20, 1592.0), (.25, 1593.5)],
    }
    model, model_path = load_throttle_model(args.model)
    persisted = {float(k): float(v)
                 for k, v in model.get('feedforward_us', {}).items()}
    rclpy.init(args=ros_args)
    node = SteeringExperiment(args)
    bus = None
    summaries = []
    started = time.monotonic()
    raw_fields = [
        'condition', 'leg', 'elapsed_s', 'phase', 'target_speed_mps',
        'vehicle_direction', 'steering_pulse_us', 'steering_tick',
        'approach_direction', 'prior_steering_pulse_us', 'esc_pulse_us',
        'esc_tick', 'adaptive_feedforward_us', 'integral_trim_us',
        'controller_state', 'fresh_odom', 'tf_stamp', 'x_m', 'y_m', 'yaw_rad',
        'measured_speed_mps', 'increment_ds_m', 'increment_dyaw_rad',
        'instant_curvature_1pm', 'effective_angle_rad', 'leg_motion_m',
        'settled_signed_distance_m', 'settled_heading_change_rad',
        'front_clearance_m', 'rear_clearance_m'
    ]

    with open(args.output, 'w', newline='') as raw_file:
        writer = csv.DictWriter(raw_file, fieldnames=raw_fields)
        writer.writeheader()
        try:
            bus = open_bus()
            neutral(bus)
            write_steering_us(bus, args.channel, args.center_us)
            node.spin_for(3.0)
            neutral_samples = node.velocity_history[-30:]
            if len(neutral_samples) < 10:
                raise RuntimeError('insufficient fresh odometry for noise calibration')
            noise_center = statistics.median(neutral_samples)
            noise_mad = statistics.median(
                abs(value - noise_center) for value in neutral_samples
            )
            noise_sigma = max(0.001, 1.4826 * noise_mad)
            movement_threshold = max(0.020, min(0.060, 4.0 * noise_sigma))
            breakaway_threshold = max(
                movement_threshold, min(0.080, 0.25 * args.target_speed)
            )
            settled_speed_threshold = max(
                movement_threshold, 0.50 * args.target_speed
            )
            print(f'Loaded throttle model: {model_path}')
            print(f'ODOM NOISE: sigma={noise_sigma:.4f} m/s, '
                  f'movement={movement_threshold:.4f} m/s, '
                  f'breakaway={breakaway_threshold:.4f} m/s')
            print('ACKERMANN ID: kappa=delta_yaw/signed_distance, '
                  'delta=atan(wheelbase*kappa)')

            conditions = []
            session_trim = {'forward': 0.0, 'reverse': 0.0}
            for target in targets:
                approaches = ('increasing', 'decreasing')
                if args.interior_approach_only and target < args.center_us:
                    approaches = ('decreasing',)
                elif args.interior_approach_only and target > args.center_us:
                    approaches = ('increasing',)
                for approach in approaches:
                    prior = (target - args.approach_offset_us
                             if approach == 'increasing'
                             else target + args.approach_offset_us)
                    conditions.append((target, approach, prior))

            leg_number = 0
            for condition, (steering_us, approach, prior_us) in enumerate(
                    conditions, 1):
                neutral(bus)
                write_steering_us(bus, args.channel, prior_us)
                node.spin_for(0.5)
                steering_tick = write_steering_us(
                    bus, args.channel, steering_us
                )
                node.spin_for(0.8)
                print(f'condition {condition}/{len(conditions)}: '
                      f'{steering_us:.1f} us approached by {approach}')

                for direction in ('forward', 'reverse'):
                    leg_number += 1
                    sign = 1.0 if direction == 'forward' else -1.0
                    target_speed = sign * args.target_speed
                    node.direction = direction
                    neutral(bus)
                    # Wait for the preceding leg to stop before reversing.
                    stop_deadline = time.monotonic() + 2.0
                    while (rclpy.ok() and time.monotonic() < stop_deadline and
                           abs(node.signed_speed) > movement_threshold):
                        rclpy.spin_once(node, timeout_sec=0.03)

                    map_points = throttle_maps[direction]
                    fallback_ff = interpolate(map_points, args.target_speed)
                    feedforward = persisted.get(target_speed, fallback_ff)
                    gain = (args.kp if args.kp > 0
                            else local_gain(map_points, args.target_speed))
                    integral_trim = session_trim[direction]
                    adaptive_feedforward = feedforward + integral_trim
                    # Begin every leg at a deliberately weak rolling command
                    # and search for breakaway from current conditions. A
                    # historical kick is never applied blindly.
                    command = (min(1425.0, adaptive_feedforward + 4.0)
                               if sign > 0 else
                               max(1575.0, adaptive_feedforward - 4.0))
                    startup = True
                    moving_samples = 0
                    low_samples = 0
                    recovery_samples = 0
                    recovery_count = 0
                    recovery_active = False
                    fresh_count = 0
                    last_odom_seq = node.odom_seq
                    leg_motion = 0.0
                    settled_ds = 0.0
                    settled_dyaw = 0.0
                    settled_samples = 0
                    leg_started = time.monotonic()
                    completed = False

                    while rclpy.ok():
                        now = time.monotonic()
                        if now - leg_started >= args.leg_timeout:
                            raise RuntimeError(f'leg {leg_number} timed out')
                        rclpy.spin_once(node, timeout_sec=0.02)
                        fresh = node.odom_seq != last_odom_seq
                        if fresh:
                            last_odom_seq = node.odom_seq
                            fresh_count += 1
                            leg_motion += abs(node.last_ds)
                            speed_in_direction = sign * node.signed_speed
                            if startup:
                                if speed_in_direction >= breakaway_threshold:
                                    moving_samples += 1
                                else:
                                    moving_samples = 0
                                if moving_samples >= 1:
                                    startup = False
                                    kick_command = command
                                    # Static friction has been defeated. Return
                                    # immediately to the learned rolling effort;
                                    # holding or slowly shedding the kick causes
                                    # the delayed lidar feedback to overshoot.
                                    command = adaptive_feedforward
                                    print(f'  leg {leg_number} {direction}: '
                                          f'breakaway {kick_command:.1f} us -> '
                                          f'sustain {command:.1f} us')
                                else:
                                    ramp = (US_PER_TICK if fresh_count <= 4
                                            else 2.5 * US_PER_TICK)
                                    command += -ramp if sign > 0 else ramp
                                    command = (max(1380.0, command) if sign > 0
                                               else min(1620.0, command))
                            else:
                                error = target_speed - node.signed_speed
                                if recovery_active:
                                    recovery_samples += 1
                                    if speed_in_direction >= breakaway_threshold:
                                        adaptive_feedforward = (
                                            feedforward + integral_trim
                                        )
                                        print(f'  leg {leg_number} {direction}: '
                                              'recovered -> adaptive sustain '
                                              f'{adaptive_feedforward:.1f} us')
                                        recovery_active = False
                                        low_samples = recovery_samples = 0
                                        command = adaptive_feedforward
                                    elif recovery_samples >= 18:
                                        raise RuntimeError(
                                            f'leg {leg_number} traction recovery timed out'
                                        )
                                    else:
                                        command += (-2.5 * US_PER_TICK if sign > 0
                                                    else 2.5 * US_PER_TICK)
                                        command = (max(1380.0, command) if sign > 0
                                                   else min(1620.0, command))
                                else:
                                    integral_trim -= (
                                        args.ki * error * max(0.0, node.last_dt)
                                    )
                                    integral_trim = max(
                                        -12.0, min(12.0, integral_trim)
                                    )
                                    session_trim[direction] = integral_trim
                                    adaptive_feedforward = (
                                        feedforward + integral_trim
                                    )
                                    desired = adaptive_feedforward - gain * error
                                    desired = (max(1404.0, min(1425.0, desired))
                                               if sign > 0 else
                                               max(1575.0, min(1608.0, desired)))
                                    delta = desired - command
                                    command += max(-6.0, min(6.0, delta))
                                    if speed_in_direction < movement_threshold:
                                        low_samples += 1
                                    else:
                                        low_samples = 0
                                    if low_samples >= 3:
                                        if recovery_count >= 2:
                                            raise RuntimeError(
                                                f'leg {leg_number} stalled after two recoveries'
                                            )
                                        recovery_active = True
                                        recovery_count += 1
                                        recovery_samples = 0
                                        print(f'  leg {leg_number} {direction}: '
                                              'adaptive traction recovery')
                                if (not recovery_active and
                                        speed_in_direction >= settled_speed_threshold and
                                        abs(error) <= 0.08):
                                    settled_ds += node.last_ds
                                    settled_dyaw += node.last_dyaw
                                    settled_samples += 1

                        esc_tick = round(command / US_PER_TICK)
                        # PCA9685 channels retain their values in hardware,
                        # but explicitly refresh steering alongside throttle
                        # so the sole experiment writer continuously asserts
                        # the complete actuator state.
                        write_steering_us(bus, args.channel, steering_us)
                        write_tick(bus, esc_tick)
                        okay, reason = node.safe(leg_motion)
                        if not okay:
                            raise RuntimeError(f'leg {leg_number}: {reason}')
                        pose = node.pose or (
                            math.nan, math.nan, math.nan, math.nan
                        )
                        instant_curvature = (
                            node.last_dyaw / node.last_ds
                            if fresh and abs(node.last_ds) > 1e-5 else math.nan
                        )
                        effective_angle = (
                            math.atan(WHEELBASE_M * instant_curvature)
                            if math.isfinite(instant_curvature) else math.nan
                        )
                        writer.writerow({
                            'condition': condition,
                            'leg': leg_number,
                            'elapsed_s': f'{now - started:.4f}',
                            'phase': 'startup' if startup else 'rolling',
                            'target_speed_mps': f'{target_speed:.4f}',
                            'vehicle_direction': direction,
                            'steering_pulse_us': f'{steering_us:.3f}',
                            'steering_tick': steering_tick,
                            'approach_direction': approach,
                            'prior_steering_pulse_us': f'{prior_us:.3f}',
                            'esc_pulse_us': f'{command:.3f}',
                            'esc_tick': esc_tick,
                            'adaptive_feedforward_us': (
                                f'{adaptive_feedforward:.3f}'
                            ),
                            'integral_trim_us': f'{integral_trim:.3f}',
                            'controller_state': (
                                'startup' if startup else
                                'recovery' if recovery_active else 'rolling'
                            ),
                            'fresh_odom': int(fresh),
                            'tf_stamp': f'{pose[0]:.9f}',
                            'x_m': f'{pose[1]:.6f}',
                            'y_m': f'{pose[2]:.6f}',
                            'yaw_rad': f'{pose[3]:.6f}',
                            'measured_speed_mps': f'{node.signed_speed:.6f}',
                            'increment_ds_m': f'{node.last_ds:.7f}' if fresh else '',
                            'increment_dyaw_rad': f'{node.last_dyaw:.7f}' if fresh else '',
                            'instant_curvature_1pm': (
                                f'{instant_curvature:.7f}'
                                if math.isfinite(instant_curvature) else ''
                            ),
                            'effective_angle_rad': (
                                f'{effective_angle:.7f}'
                                if math.isfinite(effective_angle) else ''
                            ),
                            'leg_motion_m': f'{leg_motion:.6f}',
                            'settled_signed_distance_m': f'{settled_ds:.6f}',
                            'settled_heading_change_rad': f'{settled_dyaw:.6f}',
                            'front_clearance_m': f'{node.front:.3f}',
                            'rear_clearance_m': f'{node.rear:.3f}',
                        })
                        if leg_motion >= args.leg_distance:
                            completed = True
                            break

                    neutral(bus)
                    raw_file.flush()
                    if not completed or settled_samples < 3 or abs(settled_ds) < 0.05:
                        raise RuntimeError(
                            f'leg {leg_number} had insufficient settled odometry'
                        )
                    curvature = settled_dyaw / settled_ds
                    effective_angle = math.atan(WHEELBASE_M * curvature)
                    summary = {
                        'condition': condition,
                        'leg': leg_number,
                        'pulse_us': steering_us,
                        'approach': approach,
                        'prior_us': prior_us,
                        'direction': direction,
                        'target_speed_mps': target_speed,
                        'signed_distance_m': settled_ds,
                        'heading_change_rad': settled_dyaw,
                        'curvature_1pm': curvature,
                        'effective_angle_rad': effective_angle,
                        'samples': settled_samples,
                        'integral_trim_us': integral_trim,
                        'valid': True,
                    }
                    summaries.append(summary)
                    print(f'  leg {leg_number} {direction}: '
                          f'kappa={curvature:+.4f} 1/m, '
                          f'delta={math.degrees(effective_angle):+.2f} deg, '
                          f'n={settled_samples}')
                    node.spin_for(0.8)

            center, slope = fitted_center(summaries)
            if center is not None:
                print(f'PRELIMINARY rolling center: {center:.2f} us '
                      f'(slope {slope:+.6f} 1/m/us)')
                if not min(targets) <= center <= max(targets):
                    print('WARNING: zero crossing is outside tested pulses; '
                          'do not adopt it yet')
            else:
                print('No reliable preliminary center fit')
        except (KeyboardInterrupt, RuntimeError) as error:
            print('STOP:', error)
        finally:
            if bus is not None:
                try:
                    for _ in range(5):
                        neutral(bus)
                        time.sleep(0.02)
                    write_steering_us(bus, args.channel, args.center_us)
                    time.sleep(0.3)
                    release_channel(bus, args.channel)
                finally:
                    bus.close()
            node.destroy_node()
            rclpy.shutdown()
            print(f'ESC neutral; steering centered/released. Log: {args.output}')

    with open(args.summary_output, 'w', newline='') as summary_file:
        fields = [
            'condition', 'leg', 'pulse_us', 'approach', 'prior_us',
            'direction', 'target_speed_mps', 'signed_distance_m',
            'heading_change_rad', 'curvature_1pm', 'effective_angle_rad',
            'samples', 'integral_trim_us', 'valid'
        ]
        summary_writer = csv.DictWriter(summary_file, fieldnames=fields)
        summary_writer.writeheader()
        summary_writer.writerows(summaries)
    print('Summary:', args.summary_output)


if __name__ == '__main__':
    main()
