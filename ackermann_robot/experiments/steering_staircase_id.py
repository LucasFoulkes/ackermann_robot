#!/usr/bin/env python3
"""Identify dynamic steering response during a balanced forward/reverse path."""

import argparse
import csv
import math
import os
import statistics
import time

import rclpy
import yaml

from ackermann_robot.experiments.esc_forward_calibrate import (
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
from ackermann_robot.experiments.steering_system_id import (
    WHEELBASE_M,
    SteeringExperiment,
    angle_difference,
)


THROTTLE_MAPS = {
    'forward': [(.20, 1419.0), (.24, 1417.4), (.25, 1417.0)],
    'reverse': [(.20, 1592.0), (.25, 1593.5)],
}


class AdaptiveThrottle:
    """Fresh-odometry PI trim with adaptive breakaway and stall recovery."""

    def __init__(self, args, direction, model, session_trim,
                 movement_threshold, breakaway_threshold):
        self.args = args
        self.direction = direction
        self.sign = 1.0 if direction == 'forward' else -1.0
        self.target = self.sign * args.target_speed
        self.movement_threshold = movement_threshold
        self.breakaway_threshold = breakaway_threshold
        points = THROTTLE_MAPS[direction]
        fallback = interpolate(points, args.target_speed)
        persisted = {
            float(key): float(value)
            for key, value in model.get('feedforward_us', {}).items()
        }
        self.base_feedforward = persisted.get(self.target, fallback)
        self.gain = (args.kp if args.kp > 0
                     else local_gain(points, args.target_speed))
        self.integral_trim = session_trim[direction]
        self.session_trim = session_trim
        self.adaptive_feedforward = (
            self.base_feedforward + self.integral_trim
        )
        self.command = (
            min(1425.0, self.adaptive_feedforward + 4.0)
            if self.sign > 0 else
            max(1575.0, self.adaptive_feedforward - 4.0)
        )
        self.state = 'startup'
        self.fresh_count = 0
        self.low_samples = 0
        self.recovery_samples = 0
        self.recovery_limit_dwell = 0
        self.recovery_count = 0
        self.new_recovery = False

    def update(self, speed, dt):
        self.new_recovery = False
        self.fresh_count += 1
        speed_in_direction = self.sign * speed

        if self.state == 'startup':
            if speed_in_direction >= self.breakaway_threshold:
                kick = self.command
                self.state = 'rolling'
                self.recovery_limit_dwell = 0
                self.command = self.adaptive_feedforward
                print(f'  {self.direction}: breakaway {kick:.1f} us -> '
                      f'adaptive sustain {self.command:.1f} us')
            else:
                ramp = (US_PER_TICK if self.fresh_count <= 4
                        else 2.5 * US_PER_TICK)
                self.command += -ramp if self.sign > 0 else ramp
                forward_limit = getattr(
                    self.args, 'forward_recovery_limit_us', 1380.0
                )
                reverse_limit = getattr(
                    self.args, 'reverse_recovery_limit_us', 1620.0
                )
                self.command = (max(forward_limit, self.command)
                                if self.sign > 0 else
                                min(reverse_limit, self.command))
                active_limit = (forward_limit if self.sign > 0
                                else reverse_limit)
                at_limit = (self.command <= active_limit + 0.5
                            if self.sign > 0 else
                            self.command >= active_limit - 0.5)
                self.recovery_limit_dwell = (
                    self.recovery_limit_dwell + 1 if at_limit else 0
                )
                dwell_required = getattr(
                    self.args, 'recovery_limit_dwell_samples', None
                )
                if (dwell_required is not None and
                        self.recovery_limit_dwell >= dwell_required):
                    raise RuntimeError(
                        f'{self.direction} startup exhausted authority at '
                        f'{active_limit:.1f} us'
                    )
            return

        error = self.target - speed
        if self.state == 'recovery':
            self.recovery_samples += 1
            if speed_in_direction >= self.breakaway_threshold:
                self.state = 'rolling'
                self.low_samples = self.recovery_samples = 0
                self.recovery_limit_dwell = 0
                self.command = self.adaptive_feedforward
                print(f'  {self.direction}: recovered -> adaptive sustain '
                      f'{self.command:.1f} us')
            else:
                forward_limit = getattr(
                    self.args, 'forward_recovery_limit_us', 1380.0
                )
                reverse_limit = getattr(
                    self.args, 'reverse_recovery_limit_us', 1620.0
                )
                active_limit = (forward_limit if self.sign > 0
                                else reverse_limit)
                at_limit = (self.command <= active_limit + 0.5
                            if self.sign > 0 else
                            self.command >= active_limit - 0.5)
                self.recovery_limit_dwell = (
                    self.recovery_limit_dwell + 1 if at_limit else 0
                )
                dwell_required = getattr(
                    self.args, 'recovery_limit_dwell_samples', None
                )
                if (dwell_required is not None and
                        self.recovery_limit_dwell >= dwell_required):
                    raise RuntimeError(
                        f'{self.direction} recovery exhausted authority at '
                        f'{active_limit:.1f} us'
                    )
                if (dwell_required is None and
                        self.recovery_samples >= self.args.recovery_samples):
                    raise RuntimeError(
                        f'{self.direction} traction recovery timed out'
                    )
                self.command += (-2.5 * US_PER_TICK if self.sign > 0
                                 else 2.5 * US_PER_TICK)
                self.command = (max(forward_limit, self.command)
                                if self.sign > 0 else
                                min(reverse_limit, self.command))
            return

        self.integral_trim -= self.args.ki * error * max(0.0, dt)
        trim_limit = getattr(self.args, 'max_integral_trim_us', 12.0)
        self.integral_trim = max(
            -trim_limit, min(trim_limit, self.integral_trim)
        )
        self.session_trim[self.direction] = self.integral_trim
        self.adaptive_feedforward = (
            self.base_feedforward + self.integral_trim
        )
        desired = self.adaptive_feedforward - self.gain * error
        forward_rolling_limit = getattr(
            self.args, 'forward_rolling_limit_us', 1404.0
        )
        reverse_rolling_limit = getattr(
            self.args, 'reverse_rolling_limit_us', 1608.0
        )
        desired = (max(forward_rolling_limit, min(1425.0, desired))
                   if self.sign > 0 else
                   max(1575.0, min(reverse_rolling_limit, desired)))
        self.command += max(-6.0, min(6.0, desired - self.command))

        if speed_in_direction < self.movement_threshold:
            self.low_samples += 1
        else:
            self.low_samples = 0
        if self.low_samples >= 3:
            if self.recovery_count >= self.args.max_recoveries:
                raise RuntimeError(
                    f'{self.direction} exceeded recovery limit'
                )
            self.state = 'recovery'
            self.recovery_count += 1
            self.recovery_samples = 0
            self.recovery_limit_dwell = 0
            self.new_recovery = True
            print(f'  {self.direction}: adaptive traction recovery '
                  f'{self.recovery_count}/{self.args.max_recoveries}')


def load_model(path):
    expanded = os.path.expanduser(path)
    if not os.path.exists(expanded):
        return {}, expanded
    with open(expanded) as model_file:
        return yaml.safe_load(model_file) or {}, expanded


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm', action='store_true')
    parser.add_argument('--channel', type=int, default=DEFAULT_STEERING_CHANNEL)
    parser.add_argument('--center-us', type=float, default=1450.0)
    parser.add_argument('--curvature-gain', type=float, default=0.00325,
                        help='measured curvature gain in 1/m/us')
    parser.add_argument(
        '--validation-profile', action='store_true',
        help='test unseen curvatures with direction-specific fitted maps',
    )
    parser.add_argument('--target-speed', type=float, default=0.20)
    parser.add_argument('--block-distance', type=float, default=0.45)
    parser.add_argument('--settle-samples', type=int, default=7)
    parser.add_argument('--max-direction-distance', type=float, default=3.00)
    parser.add_argument('--direction-timeout', type=float, default=30.0)
    parser.add_argument('--stop-range', type=float, default=0.20)
    parser.add_argument('--max-speed', type=float, default=0.40)
    parser.add_argument('--kp', type=float, default=0.0)
    parser.add_argument('--ki', type=float, default=30.0)
    parser.add_argument('--max-recoveries', type=int, default=4)
    parser.add_argument('--recovery-samples', type=int, default=18)
    parser.add_argument('--model', default='~/.robot/esc_model.yaml')
    parser.add_argument('--output', default='steering_staircase.csv')
    parser.add_argument('--summary-output',
                        default='steering_staircase_summary.csv')
    args, ros_args = parser.parse_known_args()

    if not args.arm:
        parser.error('refusing hardware output without --arm')
    if not 0 <= args.channel <= 15:
        parser.error('--channel must be 0..15')
    if not 1420.0 <= args.center_us <= 1480.0:
        parser.error('--center-us must be within 1420..1480')
    if not 0.0025 <= args.curvature_gain <= 0.0040:
        parser.error('--curvature-gain must be 0.0025..0.0040')
    if not 0.15 <= args.target_speed <= 0.25:
        parser.error('--target-speed must be 0.15..0.25')
    if not 0.25 <= args.block_distance <= 0.55:
        parser.error('--block-distance must be 0.25..0.55')
    if not 3 <= args.settle_samples <= 8:
        parser.error('--settle-samples must be 3..8')
    if not 0.0 <= args.ki <= 60.0:
        parser.error('--ki must be 0..60')
    expected_distance = 6 * args.block_distance
    if args.max_direction_distance < expected_distance + 0.15:
        parser.error('--max-direction-distance leaves insufficient safety margin')

    forward_profile = ([0.0, 0.2, -0.2, 0.75, -0.75, 0.0]
                       if args.validation_profile else
                       [0.0, 0.3, -0.3, 0.6, -0.6, 0.0])
    profiles = {
        'forward': forward_profile,
        'reverse': list(reversed(forward_profile)),
    }
    if args.validation_profile:
        fitted = {
            'forward': {'center': 1442.6354, 'gain': 0.00318643},
            'reverse': {'center': 1463.3872, 'gain': 0.00295056},
        }
        pulses = {
            direction: {
                curvature: values['center'] + curvature / values['gain']
                for curvature in set(forward_profile)
            }
            for direction, values in fitted.items()
        }
        all_pulses = [pulse for mapping in pulses.values()
                      for pulse in mapping.values()]
        if min(all_pulses) < 1180.0 or max(all_pulses) > 1720.0:
            parser.error('directional validation pulse exceeds 1180..1720 us')
    else:
        shared = {
            curvature: args.center_us + curvature / args.curvature_gain
            for curvature in set(forward_profile)
        }
        pulses = {'forward': shared, 'reverse': shared}
        if min(shared.values()) < 1260.0 or max(shared.values()) > 1640.0:
            parser.error('derived steering pulse exceeds staircase range')

    # SteeringExperiment uses this name for its directional distance safety.
    args.max_leg_distance = args.max_direction_distance
    model, model_path = load_model(args.model)
    session_trim = {'forward': 0.0, 'reverse': 0.0}
    summaries = []
    raw_fields = [
        'direction', 'block', 'elapsed_s', 'block_elapsed_s',
        'target_speed_mps', 'target_curvature_1pm', 'steering_pulse_us',
        'steering_tick', 'approach_direction', 'controller_state',
        'esc_pulse_us', 'esc_tick', 'adaptive_feedforward_us',
        'integral_trim_us', 'fresh_odom', 'transition_sample', 'tf_stamp',
        'x_m', 'y_m', 'yaw_rad', 'measured_speed_mps', 'increment_ds_m',
        'increment_dyaw_rad', 'instant_curvature_1pm', 'block_motion_m',
        'direction_motion_m', 'settled_signed_distance_m',
        'settled_heading_change_rad', 'front_clearance_m',
        'rear_clearance_m'
    ]

    rclpy.init(args=ros_args)
    node = SteeringExperiment(args)
    bus = None
    started = time.monotonic()
    start_pose = None

    with open(args.output, 'w', newline='') as raw_file:
        writer = csv.DictWriter(raw_file, fieldnames=raw_fields)
        writer.writeheader()
        try:
            bus = open_bus()
            neutral(bus)
            write_steering_us(bus, args.channel, args.center_us)
            node.spin_for(3.0)
            start_pose = node.pose
            neutral_samples = node.velocity_history[-30:]
            if len(neutral_samples) < 10:
                raise RuntimeError('insufficient fresh odometry')
            center = statistics.median(neutral_samples)
            mad = statistics.median(abs(value - center)
                                    for value in neutral_samples)
            sigma = max(0.001, 1.4826 * mad)
            movement_threshold = max(0.020, min(0.060, 4.0 * sigma))
            breakaway_threshold = max(
                movement_threshold, min(0.080, 0.25 * args.target_speed)
            )
            settled_speed = max(
                movement_threshold, 0.50 * args.target_speed
            )
            print(f'Loaded throttle model: {model_path}')
            print(f'ODOM NOISE: sigma={sigma:.4f}, '
                  f'movement={movement_threshold:.4f} m/s')
            for direction in ('forward', 'reverse'):
                print(f'{direction.upper()} pulses:', ', '.join(
                    f'{curvature:+.2f}->{pulses[direction][curvature]:.1f} us'
                    for curvature in sorted(pulses[direction])
                ))

            previous_pulse = args.center_us
            global_block = 0
            for direction in ('forward', 'reverse'):
                sign = 1.0 if direction == 'forward' else -1.0
                node.direction = direction
                neutral(bus)
                stop_deadline = time.monotonic() + 3.0
                while (rclpy.ok() and time.monotonic() < stop_deadline and
                       abs(node.signed_speed) > movement_threshold):
                    rclpy.spin_once(node, timeout_sec=0.03)

                controller = AdaptiveThrottle(
                    args, direction, model, session_trim,
                    movement_threshold, breakaway_threshold,
                )
                direction_motion = 0.0
                direction_started = time.monotonic()
                last_odom_seq = node.odom_seq

                for block_index, target_curvature in enumerate(
                        profiles[direction], 1):
                    global_block += 1
                    steering_us = pulses[direction][target_curvature]
                    approach = ('increasing' if steering_us > previous_pulse else
                                'decreasing' if steering_us < previous_pulse else
                                'same')
                    steering_tick = write_steering_us(
                        bus, args.channel, steering_us
                    )
                    block_started = time.monotonic()
                    block_motion = 0.0
                    fresh_in_block = 0
                    settled_ds = settled_dyaw = 0.0
                    settled_speeds = []
                    block_recoveries = 0
                    print(f'{direction} block {block_index}/6: '
                          f'kappa={target_curvature:+.2f}, '
                          f'pulse={steering_us:.1f} us ({approach})')

                    while rclpy.ok() and block_motion < args.block_distance:
                        now = time.monotonic()
                        if now - direction_started >= args.direction_timeout:
                            raise RuntimeError(f'{direction} direction timed out')
                        rclpy.spin_once(node, timeout_sec=0.02)
                        fresh = node.odom_seq != last_odom_seq
                        transition_sample = 0
                        if fresh:
                            last_odom_seq = node.odom_seq
                            fresh_in_block += 1
                            transition_sample = fresh_in_block
                            increment = abs(node.last_ds)
                            block_motion += increment
                            direction_motion += increment
                            controller.update(node.signed_speed, node.last_dt)
                            if controller.new_recovery:
                                block_recoveries += 1
                            speed_in_direction = sign * node.signed_speed
                            if (fresh_in_block > args.settle_samples and
                                    controller.state == 'rolling' and
                                    speed_in_direction >= settled_speed):
                                settled_ds += node.last_ds
                                settled_dyaw += node.last_dyaw
                                settled_speeds.append(speed_in_direction)

                        write_steering_us(bus, args.channel, steering_us)
                        esc_tick = round(controller.command / US_PER_TICK)
                        write_tick(bus, esc_tick)
                        okay, reason = node.safe(direction_motion)
                        if not okay:
                            raise RuntimeError(f'{direction} block '
                                               f'{block_index}: {reason}')
                        pose = node.pose or (
                            math.nan, math.nan, math.nan, math.nan
                        )
                        instant_curvature = (
                            node.last_dyaw / node.last_ds
                            if fresh and abs(node.last_ds) > 1e-5 else math.nan
                        )
                        writer.writerow({
                            'direction': direction,
                            'block': global_block,
                            'elapsed_s': f'{now - started:.4f}',
                            'block_elapsed_s': f'{now - block_started:.4f}',
                            'target_speed_mps': f'{controller.target:.4f}',
                            'target_curvature_1pm': f'{target_curvature:.5f}',
                            'steering_pulse_us': f'{steering_us:.3f}',
                            'steering_tick': steering_tick,
                            'approach_direction': approach,
                            'controller_state': controller.state,
                            'esc_pulse_us': f'{controller.command:.3f}',
                            'esc_tick': esc_tick,
                            'adaptive_feedforward_us': (
                                f'{controller.adaptive_feedforward:.3f}'
                            ),
                            'integral_trim_us': f'{controller.integral_trim:.3f}',
                            'fresh_odom': int(fresh),
                            'transition_sample': transition_sample,
                            'tf_stamp': f'{pose[0]:.9f}',
                            'x_m': f'{pose[1]:.6f}',
                            'y_m': f'{pose[2]:.6f}',
                            'yaw_rad': f'{pose[3]:.6f}',
                            'measured_speed_mps': f'{node.signed_speed:.6f}',
                            'increment_ds_m': f'{node.last_ds:.7f}' if fresh else '',
                            'increment_dyaw_rad': (
                                f'{node.last_dyaw:.7f}' if fresh else ''
                            ),
                            'instant_curvature_1pm': (
                                f'{instant_curvature:.7f}'
                                if math.isfinite(instant_curvature) else ''
                            ),
                            'block_motion_m': f'{block_motion:.6f}',
                            'direction_motion_m': f'{direction_motion:.6f}',
                            'settled_signed_distance_m': f'{settled_ds:.6f}',
                            'settled_heading_change_rad': f'{settled_dyaw:.6f}',
                            'front_clearance_m': f'{node.front:.3f}',
                            'rear_clearance_m': f'{node.rear:.3f}',
                        })

                    raw_file.flush()
                    valid = (len(settled_speeds) >= 5 and
                             abs(settled_ds) >= 0.08 and
                             block_recoveries == 0)
                    measured_curvature = (
                        settled_dyaw / settled_ds
                        if abs(settled_ds) >= 0.02 else math.nan
                    )
                    summaries.append({
                        'direction': direction,
                        'block': global_block,
                        'target_curvature_1pm': target_curvature,
                        'steering_pulse_us': steering_us,
                        'approach_direction': approach,
                        'settled_signed_distance_m': settled_ds,
                        'settled_heading_change_rad': settled_dyaw,
                        'measured_curvature_1pm': measured_curvature,
                        'curvature_error_1pm': (
                            measured_curvature - target_curvature
                            if math.isfinite(measured_curvature) else math.nan
                        ),
                        'effective_angle_rad': (
                            math.atan(WHEELBASE_M * measured_curvature)
                            if math.isfinite(measured_curvature) else math.nan
                        ),
                        'mean_speed_mps': (
                            statistics.fmean(settled_speeds)
                            if settled_speeds else math.nan
                        ),
                        'speed_std_mps': (
                            statistics.pstdev(settled_speeds)
                            if len(settled_speeds) > 1 else math.nan
                        ),
                        'settled_samples': len(settled_speeds),
                        'recovery_events': block_recoveries,
                        'integral_trim_us': controller.integral_trim,
                        'valid': valid,
                    })
                    print(f'  measured kappa={measured_curvature:+.3f} 1/m, '
                          f'n={len(settled_speeds)}, recoveries={block_recoveries}')
                    previous_pulse = steering_us

                neutral(bus)
                node.spin_for(1.0)

            if start_pose and node.pose:
                retrace_distance = math.hypot(
                    node.pose[1] - start_pose[1],
                    node.pose[2] - start_pose[2],
                )
                retrace_yaw = angle_difference(node.pose[3], start_pose[3])
                print(f'RETRACE: position={retrace_distance:.3f} m, '
                      f'yaw={math.degrees(retrace_yaw):+.2f} deg')
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

    summary_fields = [
        'direction', 'block', 'target_curvature_1pm', 'steering_pulse_us',
        'approach_direction', 'settled_signed_distance_m',
        'settled_heading_change_rad', 'measured_curvature_1pm',
        'curvature_error_1pm', 'effective_angle_rad', 'mean_speed_mps',
        'speed_std_mps', 'settled_samples', 'recovery_events',
        'integral_trim_us', 'valid'
    ]
    with open(args.summary_output, 'w', newline='') as summary_file:
        summary_writer = csv.DictWriter(summary_file, fieldnames=summary_fields)
        summary_writer.writeheader()
        summary_writer.writerows(summaries)
    print('Summary:', args.summary_output)


if __name__ == '__main__':
    main()
