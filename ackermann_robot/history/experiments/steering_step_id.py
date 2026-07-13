#!/usr/bin/env python3
"""Two-speed steering step response: separate time-domain from distance-domain lag.

Protocol: at each target speed, drive straight (run-in), then apply instant
steering steps center -> +K -> center -> -K -> center, holding each for a fixed
distance. The reverse leg retraces with the mirrored profile. Offline analysis
fits t_settle(v) = tau_fixed + d0 / v across the two speeds to decompose the
lag into a speed-independent part (servo, ESC, odometry pipeline) and a
rolled-distance part (tire relaxation, standstill windup release).
"""

import argparse
import csv
import math
import statistics
import time

import rclpy

from esc_forward_calibrate import (
    US_PER_TICK,
    neutral,
    open_bus,
    write_tick,
)
from steering_sanity import (
    DEFAULT_STEERING_CHANNEL,
    release_channel,
    write_steering_us,
)
from steering_staircase_id import (
    AdaptiveThrottle,
    load_model,
)
from steering_system_id import (
    SteeringExperiment,
    angle_difference,
)


# Current controller calibration (adaptive_ackermann_controller defaults).
STEERING_MAPS = {
    'forward': [(-1.15, 1077.0), (0.0, 1451.5), (1.15, 1764.0)],
    'reverse': [(-1.15, 1070.0), (0.0, 1475.5), (1.15, 1792.0)],
}

RAW_FIELDS = [
    'target_speed_mps', 'pass_index', 'direction', 'block', 'global_block',
    'from_kappa_1pm', 'to_kappa_1pm', 'steering_pulse_us', 'steering_tick',
    'controller_state', 'esc_pulse_us', 'adaptive_feedforward_us',
    'integral_trim_us', 'elapsed_s', 'time_since_step_s',
    'motion_since_step_m', 'fresh_odom', 'tf_stamp', 'x_m', 'y_m', 'yaw_rad',
    'measured_speed_mps', 'increment_ds_m', 'increment_dyaw_rad',
    'instant_curvature_1pm', 'leg_motion_m', 'front_clearance_m',
    'rear_clearance_m',
]

SUMMARY_FIELDS = [
    'target_speed_mps', 'pass_index', 'direction', 'block', 'global_block',
    'from_kappa_1pm', 'to_kappa_1pm', 'steering_pulse_us',
    'settled_signed_distance_m', 'settled_heading_change_rad',
    'measured_curvature_1pm', 'curvature_error_1pm', 'mean_speed_mps',
    'settled_samples', 'recovery_events', 'valid',
]


def kappa_to_pulse(direction, kappa):
    points = STEERING_MAPS[direction]
    if kappa <= points[0][0]:
        return points[0][1]
    if kappa >= points[-1][0]:
        return points[-1][1]
    for (k0, p0), (k1, p1) in zip(points, points[1:]):
        if k0 <= kappa <= k1:
            return p0 + (p1 - p0) * (kappa - k0) / (k1 - k0)
    raise ValueError(f'curvature {kappa} outside steering map')


def wait_for_stop(node, movement_threshold, seconds=4.0):
    deadline = time.monotonic() + seconds
    while (rclpy.ok() and time.monotonic() < deadline and
           abs(node.signed_speed) > movement_threshold):
        rclpy.spin_once(node, timeout_sec=0.03)


def measure_noise(node):
    samples = node.velocity_history[-30:]
    if len(samples) < 10:
        raise RuntimeError('insufficient fresh odometry (is MOLA running?)')
    center = statistics.median(samples)
    mad = statistics.median(abs(value - center) for value in samples)
    sigma = max(0.001, 1.4826 * mad)
    return sigma, max(0.020, min(0.060, 4.0 * sigma))


def run_leg(node, bus, args, writer, controller, direction, speed,
            pass_index, profile, counters):
    sign = 1.0 if direction == 'forward' else -1.0
    node.direction = direction
    leg_motion = 0.0
    leg_started = time.monotonic()
    leg_timeout = 20.0 + 3.0 * args.max_leg_distance / speed
    last_odom_seq = node.odom_seq
    previous_kappa = 0.0
    settled_speed = max(controller.movement_threshold, 0.50 * speed)
    summaries = []

    for block_index, target_kappa in enumerate(profile, 1):
        counters['global_block'] += 1
        required = (args.runin_distance if block_index == 1
                    else args.block_distance)
        steering_us = kappa_to_pulse(direction, target_kappa)
        steering_tick = write_steering_us(bus, args.channel, steering_us)
        block_started = time.monotonic()
        block_motion = 0.0
        fresh_in_block = 0
        settled_ds = settled_dyaw = 0.0
        settled_speeds = []
        block_recoveries = 0
        print(f'{direction} v={speed:.2f} pass {pass_index} '
              f'block {block_index}/{len(profile)}: '
              f'kappa {previous_kappa:+.2f} -> {target_kappa:+.2f} '
              f'({steering_us:.1f} us)')

        while rclpy.ok() and block_motion < required:
            now = time.monotonic()
            if now - leg_started >= leg_timeout:
                raise RuntimeError(f'{direction} leg timed out')
            rclpy.spin_once(node, timeout_sec=0.02)
            fresh = node.odom_seq != last_odom_seq
            if fresh:
                last_odom_seq = node.odom_seq
                fresh_in_block += 1
                increment = abs(node.last_ds)
                block_motion += increment
                leg_motion += increment
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
            write_tick(bus, round(controller.command / US_PER_TICK))
            okay, reason = node.safe(leg_motion)
            if not okay:
                raise RuntimeError(
                    f'{direction} block {block_index}: {reason}')
            pose = node.pose or (math.nan,) * 4
            instant = (node.last_dyaw / node.last_ds
                       if fresh and abs(node.last_ds) > 1e-5 else math.nan)
            writer.writerow({
                'target_speed_mps': f'{speed:.3f}',
                'pass_index': pass_index,
                'direction': direction,
                'block': block_index,
                'global_block': counters['global_block'],
                'from_kappa_1pm': f'{previous_kappa:.3f}',
                'to_kappa_1pm': f'{target_kappa:.3f}',
                'steering_pulse_us': f'{steering_us:.3f}',
                'steering_tick': steering_tick,
                'controller_state': controller.state,
                'esc_pulse_us': f'{controller.command:.3f}',
                'adaptive_feedforward_us': (
                    f'{controller.adaptive_feedforward:.3f}'),
                'integral_trim_us': f'{controller.integral_trim:.3f}',
                'elapsed_s': f'{now - counters["started"]:.4f}',
                'time_since_step_s': f'{now - block_started:.4f}',
                'motion_since_step_m': f'{block_motion:.6f}',
                'fresh_odom': int(fresh),
                'tf_stamp': f'{pose[0]:.9f}',
                'x_m': f'{pose[1]:.6f}',
                'y_m': f'{pose[2]:.6f}',
                'yaw_rad': f'{pose[3]:.6f}',
                'measured_speed_mps': f'{node.signed_speed:.6f}',
                'increment_ds_m': f'{node.last_ds:.7f}' if fresh else '',
                'increment_dyaw_rad': (
                    f'{node.last_dyaw:.7f}' if fresh else ''),
                'instant_curvature_1pm': (
                    f'{instant:.7f}' if math.isfinite(instant) else ''),
                'leg_motion_m': f'{leg_motion:.6f}',
                'front_clearance_m': f'{node.front:.3f}',
                'rear_clearance_m': f'{node.rear:.3f}',
            })

        measured = (settled_dyaw / settled_ds
                    if abs(settled_ds) >= 0.02 else math.nan)
        valid = (len(settled_speeds) >= 5 and abs(settled_ds) >= 0.08 and
                 block_recoveries == 0)
        summaries.append({
            'target_speed_mps': speed,
            'pass_index': pass_index,
            'direction': direction,
            'block': block_index,
            'global_block': counters['global_block'],
            'from_kappa_1pm': previous_kappa,
            'to_kappa_1pm': target_kappa,
            'steering_pulse_us': steering_us,
            'settled_signed_distance_m': settled_ds,
            'settled_heading_change_rad': settled_dyaw,
            'measured_curvature_1pm': measured,
            'curvature_error_1pm': (measured - target_kappa
                                    if math.isfinite(measured) else math.nan),
            'mean_speed_mps': (statistics.fmean(settled_speeds)
                               if settled_speeds else math.nan),
            'settled_samples': len(settled_speeds),
            'recovery_events': block_recoveries,
            'valid': valid,
        })
        print(f'  settled kappa={measured:+.3f} 1/m, '
              f'n={len(settled_speeds)}, recoveries={block_recoveries}')
        previous_kappa = target_kappa
    return summaries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm', action='store_true')
    parser.add_argument('--channel', type=int, default=DEFAULT_STEERING_CHANNEL)
    parser.add_argument('--speeds', type=float, nargs='+',
                        default=[0.30, 0.15],
                        help='target speeds; two distinct values decompose the lag')
    parser.add_argument('--step-curvature', type=float, default=0.80)
    parser.add_argument('--runin-distance', type=float, default=0.80)
    parser.add_argument('--block-distance', type=float, default=0.55)
    parser.add_argument('--passes', type=int, default=3)
    parser.add_argument('--settle-samples', type=int, default=7)
    parser.add_argument('--stop-range', type=float, default=0.25)
    parser.add_argument('--max-speed', type=float, default=0.55)
    parser.add_argument('--kp', type=float, default=0.0)
    parser.add_argument('--ki', type=float, default=30.0)
    parser.add_argument('--max-recoveries', type=int, default=4)
    parser.add_argument('--recovery-samples', type=int, default=18)
    parser.add_argument('--model', default='~/.robot/esc_model.yaml')
    parser.add_argument('--output', default='steering_step_id.csv')
    parser.add_argument('--summary-output', default='steering_step_summary.csv')
    args, ros_args = parser.parse_known_args()

    if not args.arm:
        parser.error('refusing hardware output without --arm')
    if not 0 <= args.channel <= 15:
        parser.error('--channel must be 0..15')
    if not 1 <= len(args.speeds) <= 3:
        parser.error('--speeds takes 1..3 values')
    for speed in args.speeds:
        if not 0.10 <= speed <= 0.36:
            parser.error('each speed must be 0.10..0.36')
    if not 0.30 <= args.step_curvature <= 1.00:
        parser.error('--step-curvature must be 0.30..1.00')
    if not 0.50 <= args.runin_distance <= 1.20:
        parser.error('--runin-distance must be 0.50..1.20')
    if not 0.35 <= args.block_distance <= 0.80:
        parser.error('--block-distance must be 0.35..0.80')
    if not 1 <= args.passes <= 5:
        parser.error('--passes must be 1..5')
    for direction in ('forward', 'reverse'):
        for kappa in (args.step_curvature, -args.step_curvature, 0.0):
            pulse = kappa_to_pulse(direction, kappa)
            if not 1150.0 <= pulse <= 1850.0:
                parser.error(f'derived pulse {pulse:.0f} us out of bounds')

    kappa = args.step_curvature
    profile = [0.0, kappa, 0.0, -kappa, 0.0]
    leg_distance = args.runin_distance + (len(profile) - 1) * args.block_distance
    args.max_leg_distance = leg_distance + 0.40

    model, model_path = load_model(args.model)
    session_trim = {'forward': 0.0, 'reverse': 0.0}
    all_summaries = []

    rclpy.init(args=ros_args)
    node = SteeringExperiment(args)
    bus = None
    counters = {'started': time.monotonic(), 'global_block': 0}

    with open(args.output, 'w', newline='') as raw_file:
        writer = csv.DictWriter(raw_file, fieldnames=RAW_FIELDS)
        writer.writeheader()
        try:
            bus = open_bus()
            neutral(bus)
            write_steering_us(bus, args.channel,
                              kappa_to_pulse('forward', 0.0))
            node.spin_for(3.0)
            sigma, movement_threshold = measure_noise(node)
            print(f'Throttle model: {model_path}')
            print(f'ODOM NOISE: sigma={sigma:.4f} m/s, '
                  f'movement={movement_threshold:.4f} m/s')
            print(f'Leg length {leg_distance:.2f} m, profile '
                  f'{profile}, {args.passes} passes x '
                  f'{len(args.speeds)} speeds')

            start_pose = node.pose
            for speed in args.speeds:
                args.target_speed = speed
                breakaway_threshold = max(movement_threshold,
                                          min(0.080, 0.25 * speed))
                for pass_index in range(1, args.passes + 1):
                    for direction in ('forward', 'reverse'):
                        leg_profile = (profile if direction == 'forward'
                                       else list(reversed(profile)))
                        neutral(bus)
                        wait_for_stop(node, movement_threshold)
                        controller = AdaptiveThrottle(
                            args, direction, model, session_trim,
                            movement_threshold, breakaway_threshold)
                        all_summaries.extend(run_leg(
                            node, bus, args, writer, controller, direction,
                            speed, pass_index, leg_profile, counters))
                        raw_file.flush()
                    if start_pose and node.pose:
                        drift = math.hypot(node.pose[1] - start_pose[1],
                                           node.pose[2] - start_pose[2])
                        yaw_drift = angle_difference(node.pose[3],
                                                     start_pose[3])
                        print(f'RETRACE after v={speed:.2f} pass '
                              f'{pass_index}: {drift:.3f} m, '
                              f'{math.degrees(yaw_drift):+.1f} deg')
        except (KeyboardInterrupt, RuntimeError) as error:
            print('STOP:', error)
        finally:
            if bus is not None:
                try:
                    for _ in range(5):
                        neutral(bus)
                        time.sleep(0.02)
                    write_steering_us(bus, args.channel,
                                      kappa_to_pulse('forward', 0.0))
                    time.sleep(0.3)
                    release_channel(bus, args.channel)
                finally:
                    bus.close()
            node.destroy_node()
            rclpy.shutdown()
            print(f'ESC neutral; steering centered/released. '
                  f'Log: {args.output}')

    with open(args.summary_output, 'w', newline='') as summary_file:
        summary_writer = csv.DictWriter(summary_file,
                                        fieldnames=SUMMARY_FIELDS)
        summary_writer.writeheader()
        summary_writer.writerows(all_summaries)
    print('Summary:', args.summary_output)


if __name__ == '__main__':
    main()
