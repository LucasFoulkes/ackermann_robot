#!/usr/bin/env python3
"""Adaptively learn maximum useful left/right steering from odometry."""

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
from ackermann_robot.experiments.steering_sanity import (
    DEFAULT_STEERING_CHANNEL,
    release_channel,
    write_steering_us,
)
from ackermann_robot.experiments.steering_staircase_id import AdaptiveThrottle
from ackermann_robot.experiments.steering_system_id import (
    WHEELBASE_M,
    SteeringExperiment,
)


TRACK_WIDTH_M = 0.247


class MobilityFailure(RuntimeError):
    pass


def load_model(path):
    expanded = os.path.expanduser(path)
    if not os.path.exists(expanded):
        return {}, expanded
    with open(expanded) as model_file:
        return yaml.safe_load(model_file) or {}, expanded


def wheel_angles(curvature):
    if abs(curvature) < 1e-6:
        return 0.0, 0.0, 0.0
    sign = 1.0 if curvature > 0 else -1.0
    radius = 1.0 / abs(curvature)
    effective = math.atan(WHEELBASE_M * curvature)
    inner_radius = max(0.01, radius - TRACK_WIDTH_M / 2.0)
    outer_radius = radius + TRACK_WIDTH_M / 2.0
    inner = sign * math.atan(WHEELBASE_M / inner_radius)
    outer = sign * math.atan(WHEELBASE_M / outer_radius)
    return effective, inner, outer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm', action='store_true')
    parser.add_argument('--channel', type=int, default=DEFAULT_STEERING_CHANNEL)
    parser.add_argument('--initial-center-us', type=float, default=1450.0,
                        help='starting hypothesis only; center is re-estimated')
    parser.add_argument('--probe-offset-us', type=float, default=50.0)
    parser.add_argument('--absolute-min-us', type=float, default=1000.0,
                        help='immutable electrical guard, not a learned endpoint')
    parser.add_argument('--absolute-max-us', type=float, default=2000.0,
                        help='immutable electrical guard, not a learned endpoint')
    parser.add_argument('--search-side', choices=('both', 'lower', 'upper'),
                        default='both')
    parser.add_argument(
        '--resume-pulse', type=float,
        help='freshly revalidate this learned pulse, then continue outward',
    )
    parser.add_argument('--leg-distance', type=float, default=0.76)
    parser.add_argument('--warmup-distance', type=float, default=0.35,
                        help='short straight trim-learning distance')
    parser.add_argument('--settle-samples', type=int, default=6)
    parser.add_argument('--target-speed', type=float, default=0.20)
    parser.add_argument('--leg-timeout', type=float, default=14.0)
    parser.add_argument('--stop-range', type=float, default=0.20)
    parser.add_argument('--max-speed', type=float, default=0.40)
    parser.add_argument('--kp', type=float, default=0.0)
    parser.add_argument('--ki', type=float, default=30.0)
    parser.add_argument('--max-recoveries', type=int, default=5)
    parser.add_argument('--recovery-samples', type=int, default=32)
    parser.add_argument('--forward-recovery-limit-us', type=float, default=1180.0)
    parser.add_argument('--reverse-recovery-limit-us', type=float, default=1820.0)
    parser.add_argument('--recovery-limit-dwell-samples', type=int, default=5)
    parser.add_argument('--forward-rolling-limit-us', type=float, default=1380.0)
    parser.add_argument('--reverse-rolling-limit-us', type=float, default=1640.0)
    parser.add_argument('--max-integral-trim-us', type=float, default=40.0)
    parser.add_argument('--model', default='~/.robot/esc_model.yaml')
    parser.add_argument('--output', default='steering_limit_search.csv')
    parser.add_argument('--summary-output',
                        default='steering_limit_search_summary.csv')
    args, ros_args = parser.parse_known_args()

    if not args.arm:
        parser.error('refusing hardware output without --arm')
    if not 0 <= args.channel <= 15:
        parser.error('--channel must be 0..15')
    if not 1200.0 <= args.initial_center_us <= 1800.0:
        parser.error('--initial-center-us must be 1200..1800')
    if not 25.0 <= args.probe_offset_us <= 80.0:
        parser.error('--probe-offset-us must be 25..80')
    if not 900.0 <= args.absolute_min_us < args.absolute_max_us <= 2100.0:
        parser.error('invalid immutable electrical pulse guards')
    if args.resume_pulse is not None and args.search_side == 'both':
        parser.error('--resume-pulse requires --search-side lower or upper')
    if (args.resume_pulse is not None and
            not args.absolute_min_us <= args.resume_pulse <= args.absolute_max_us):
        parser.error('--resume-pulse must be inside the electrical guard')
    if not 0.25 <= args.leg_distance <= 0.80:
        parser.error('--leg-distance must be 0.25..0.80')
    if not 0.20 <= args.warmup_distance <= 0.50:
        parser.error('--warmup-distance must be 0.20..0.50')
    if not 0.15 <= args.target_speed <= 0.25:
        parser.error('--target-speed must be 0.15..0.25')
    if not 1000.0 <= args.forward_recovery_limit_us <= 1380.0:
        parser.error('--forward-recovery-limit-us must be 1000..1380')
    if not 1620.0 <= args.reverse_recovery_limit_us <= 2000.0:
        parser.error('--reverse-recovery-limit-us must be 1620..2000')
    if not args.forward_recovery_limit_us <= args.forward_rolling_limit_us <= 1410.0:
        parser.error('invalid forward rolling authority')
    if not 1590.0 <= args.reverse_rolling_limit_us <= args.reverse_recovery_limit_us:
        parser.error('invalid reverse rolling authority')
    if not 12.0 <= args.max_integral_trim_us <= 50.0:
        parser.error('--max-integral-trim-us must be 12..50')
    if not 3 <= args.recovery_limit_dwell_samples <= 15:
        parser.error('--recovery-limit-dwell-samples must be 3..15')

    args.max_leg_distance = args.leg_distance + 0.15
    model, model_path = load_model(args.model)
    session_trim = {'forward': 0.0, 'reverse': 0.0}
    raw_fields = [
        'stage', 'side', 'candidate', 'direction', 'elapsed_s',
        'steering_pulse_us', 'steering_tick', 'controller_state',
        'esc_pulse_us', 'esc_tick', 'adaptive_feedforward_us',
        'integral_trim_us', 'fresh_odom', 'fresh_in_leg', 'tf_stamp',
        'x_m', 'y_m', 'yaw_rad', 'measured_speed_mps', 'increment_ds_m',
        'increment_dyaw_rad', 'leg_motion_m', 'settled_signed_distance_m',
        'settled_heading_change_rad', 'front_clearance_m',
        'rear_clearance_m'
    ]
    result_fields = [
        'stage', 'side', 'candidate', 'pulse_us', 'forward_curvature_1pm',
        'forward_curvature_se_1pm', 'reverse_curvature_1pm',
        'reverse_curvature_se_1pm', 'mean_curvature_1pm',
        'direction_disagreement_1pm', 'effective_angle_deg',
        'inner_wheel_angle_deg', 'outer_wheel_angle_deg',
        'incremental_gain_1pm_per_us', 'recovery_events', 'clean_mobile',
        'stop_reason'
    ]
    results = []

    rclpy.init(args=ros_args)
    node = SteeringExperiment(args)
    bus = None
    started = time.monotonic()

    with open(args.output, 'w', newline='') as raw_file:
        raw_writer = csv.DictWriter(raw_file, fieldnames=raw_fields)
        raw_writer.writeheader()
        try:
            bus = open_bus()
            neutral(bus)
            write_steering_us(bus, args.channel, args.initial_center_us)
            node.spin_for(3.0)
            neutral_samples = node.velocity_history[-30:]
            if len(neutral_samples) < 10:
                raise RuntimeError('insufficient fresh odometry')
            velocity_center = statistics.median(neutral_samples)
            velocity_mad = statistics.median(
                abs(value - velocity_center) for value in neutral_samples
            )
            velocity_sigma = max(0.001, 1.4826 * velocity_mad)
            movement_threshold = max(
                0.020, min(0.060, 4.0 * velocity_sigma)
            )
            breakaway_threshold = max(
                movement_threshold, min(0.080, 0.25 * args.target_speed)
            )
            settled_speed = max(
                movement_threshold, 0.50 * args.target_speed
            )
            print(f'Loaded throttle model: {model_path}')
            print(f'ODOM NOISE: sigma={velocity_sigma:.4f}, '
                  f'movement={movement_threshold:.4f} m/s')
            print('LIMIT SEARCH: no historical steering endpoints are used')
            print(f'Electrical guard: {args.absolute_min_us:.0f}..'
                  f'{args.absolute_max_us:.0f} us')
            print(f'Throttle recovery authority: forward to '
                  f'{args.forward_recovery_limit_us:.0f} us, reverse to '
                  f'{args.reverse_recovery_limit_us:.0f} us')

            leg_counter = 0

            def stop_and_center(center_pulse):
                neutral(bus)
                deadline = time.monotonic() + 3.0
                while (rclpy.ok() and time.monotonic() < deadline and
                       abs(node.signed_speed) > movement_threshold):
                    rclpy.spin_once(node, timeout_sec=0.03)
                write_steering_us(bus, args.channel, center_pulse)
                node.spin_for(0.35)

            def run_leg(stage, side, candidate, pulse, direction,
                        target_distance=None):
                nonlocal leg_counter
                leg_counter += 1
                target_distance = (args.leg_distance if target_distance is None
                                   else target_distance)
                sign = 1.0 if direction == 'forward' else -1.0
                node.direction = direction
                stop_and_center(pulse)
                controller = AdaptiveThrottle(
                    args, direction, model, session_trim,
                    movement_threshold, breakaway_threshold,
                )
                last_odom_seq = node.odom_seq
                leg_started = time.monotonic()
                leg_motion = settled_ds = settled_dyaw = 0.0
                fresh_in_leg = recoveries = 0
                wrong_direction_samples = 0
                settled_samples = 0
                settled_increments = []
                expected_pulse_side = ('below 1500 us' if direction == 'forward'
                                       else 'above 1500 us')
                print(f'  LEG {leg_counter}: command {direction}; '
                      f'ESC expected {expected_pulse_side}')
                while rclpy.ok() and leg_motion < target_distance:
                    now = time.monotonic()
                    if now - leg_started >= args.leg_timeout:
                        raise MobilityFailure(f'{direction} leg timed out')
                    rclpy.spin_once(node, timeout_sec=0.02)
                    fresh = node.odom_seq != last_odom_seq
                    if fresh:
                        last_odom_seq = node.odom_seq
                        fresh_in_leg += 1
                        leg_motion += abs(node.last_ds)
                        directional_speed = sign * node.signed_speed
                        if directional_speed < -movement_threshold:
                            wrong_direction_samples += 1
                        else:
                            wrong_direction_samples = 0
                        if wrong_direction_samples >= 2:
                            raise RuntimeError(
                                f'DIRECTION MISMATCH: commanded {direction}, '
                                f'odometry={node.signed_speed:+.3f} m/s'
                            )
                        try:
                            controller.update(node.signed_speed, node.last_dt)
                        except RuntimeError as error:
                            raise MobilityFailure(str(error)) from error
                        if controller.new_recovery:
                            recoveries += 1
                        if (fresh_in_leg > args.settle_samples and
                                controller.state == 'rolling' and
                                sign * node.signed_speed >= settled_speed):
                            settled_ds += node.last_ds
                            settled_dyaw += node.last_dyaw
                            settled_samples += 1
                            settled_increments.append(
                                (node.last_ds, node.last_dyaw)
                            )

                    steering_tick = write_steering_us(
                        bus, args.channel, pulse
                    )
                    esc_tick = round(controller.command / US_PER_TICK)
                    write_tick(bus, esc_tick)
                    okay, reason = node.safe(leg_motion)
                    if not okay:
                        raise RuntimeError(reason)
                    pose = node.pose or (
                        math.nan, math.nan, math.nan, math.nan
                    )
                    raw_writer.writerow({
                        'stage': stage,
                        'side': side,
                        'candidate': candidate,
                        'direction': direction,
                        'elapsed_s': f'{now - started:.4f}',
                        'steering_pulse_us': f'{pulse:.3f}',
                        'steering_tick': steering_tick,
                        'controller_state': controller.state,
                        'esc_pulse_us': f'{controller.command:.3f}',
                        'esc_tick': esc_tick,
                        'adaptive_feedforward_us': (
                            f'{controller.adaptive_feedforward:.3f}'
                        ),
                        'integral_trim_us': f'{controller.integral_trim:.3f}',
                        'fresh_odom': int(fresh),
                        'fresh_in_leg': fresh_in_leg if fresh else 0,
                        'tf_stamp': f'{pose[0]:.9f}',
                        'x_m': f'{pose[1]:.6f}',
                        'y_m': f'{pose[2]:.6f}',
                        'yaw_rad': f'{pose[3]:.6f}',
                        'measured_speed_mps': f'{node.signed_speed:.6f}',
                        'increment_ds_m': f'{node.last_ds:.7f}' if fresh else '',
                        'increment_dyaw_rad': (
                            f'{node.last_dyaw:.7f}' if fresh else ''
                        ),
                        'leg_motion_m': f'{leg_motion:.6f}',
                        'settled_signed_distance_m': f'{settled_ds:.6f}',
                        'settled_heading_change_rad': f'{settled_dyaw:.6f}',
                        'front_clearance_m': f'{node.front:.3f}',
                        'rear_clearance_m': f'{node.rear:.3f}',
                    })
                neutral(bus)
                raw_file.flush()
                if settled_samples < 5 or abs(settled_ds) < 0.08:
                    raise MobilityFailure(
                        f'{direction} insufficient settled odometry'
                    )
                chunks = []
                chunk_count = min(4, max(1, len(settled_increments) // 5))
                for index in range(chunk_count):
                    start = index * len(settled_increments) // chunk_count
                    end = (index + 1) * len(settled_increments) // chunk_count
                    distance = sum(value[0]
                                   for value in settled_increments[start:end])
                    heading = sum(value[1]
                                  for value in settled_increments[start:end])
                    if abs(distance) >= 0.02:
                        chunks.append(heading / distance)
                curvature_se = (
                    statistics.stdev(chunks) / math.sqrt(len(chunks))
                    if len(chunks) >= 2 else math.inf
                )
                return {
                    'curvature': settled_dyaw / settled_ds,
                    'curvature_se': curvature_se,
                    'recoveries': recoveries,
                    'samples': settled_samples,
                }

            def run_pair(stage, side, candidate, pulse, center_pulse):
                if not args.absolute_min_us <= pulse <= args.absolute_max_us:
                    return {
                        'success': False,
                        'reason': 'immutable electrical guard',
                        'pulse': pulse,
                    }
                stop_and_center(center_pulse)
                write_steering_us(bus, args.channel, pulse)
                node.spin_for(0.65)
                legs = {}
                try:
                    for direction in ('forward', 'reverse'):
                        legs[direction] = run_leg(
                            stage, side, candidate, pulse, direction
                        )
                except MobilityFailure as error:
                    neutral(bus)
                    stop_and_center(center_pulse)
                    results.append({
                        'stage': stage,
                        'side': side,
                        'candidate': candidate,
                        'pulse_us': pulse,
                        'clean_mobile': False,
                        'stop_reason': str(error),
                    })
                    return {
                        'success': False,
                        'reason': str(error),
                        'pulse': pulse,
                        'legs': legs,
                    }
                forward_k = legs['forward']['curvature']
                reverse_k = legs['reverse']['curvature']
                mean_k = statistics.fmean((forward_k, reverse_k))
                disagreement = abs(forward_k - reverse_k) / 2.0
                recoveries = sum(value['recoveries'] for value in legs.values())
                effective, inner, outer = wheel_angles(mean_k)
                clean = recoveries == 0
                result = {
                    'stage': stage,
                    'side': side,
                    'candidate': candidate,
                    'pulse_us': pulse,
                    'forward_curvature_1pm': forward_k,
                    'forward_curvature_se_1pm': legs['forward']['curvature_se'],
                    'reverse_curvature_1pm': reverse_k,
                    'reverse_curvature_se_1pm': legs['reverse']['curvature_se'],
                    'mean_curvature_1pm': mean_k,
                    'direction_disagreement_1pm': disagreement,
                    'effective_angle_deg': math.degrees(effective),
                    'inner_wheel_angle_deg': math.degrees(inner),
                    'outer_wheel_angle_deg': math.degrees(outer),
                    'incremental_gain_1pm_per_us': math.nan,
                    'recovery_events': recoveries,
                    'clean_mobile': clean,
                    'stop_reason': '' if clean else 'traction recovery required',
                }
                results.append(result)
                print(f'{side} candidate {candidate}: {pulse:.1f} us -> '
                      f'kappa={mean_k:+.3f} 1/m, '
                      f'effective={math.degrees(effective):+.1f} deg, '
                      f'F/R disagreement={disagreement:.3f}, '
                      f'recoveries={recoveries}')
                return {'success': True, 'clean': clean, 'result': result}

            # Freshly learn throttle trim at the center hypothesis. Recovery is
            # allowed here and is not interpreted as a steering limit.
            print('WARMUP: learning current rolling throttle effort')
            for direction in ('forward', 'reverse'):
                try:
                    run_leg('warmup', 'center', 0,
                            args.initial_center_us, direction,
                            target_distance=args.warmup_distance)
                except MobilityFailure as error:
                    raise RuntimeError(f'warmup failed: {error}') from error

            lower_probe = args.initial_center_us - args.probe_offset_us
            upper_probe = args.initial_center_us + args.probe_offset_us
            lower = run_pair('probe', 'lower', 1, lower_probe,
                             args.initial_center_us)
            upper = run_pair('probe', 'upper', 1, upper_probe,
                             args.initial_center_us)
            if not lower.get('success') or not upper.get('success'):
                raise RuntimeError('initial steering probes failed')
            lower_result, upper_result = lower['result'], upper['result']
            delta_pulse = upper_probe - lower_probe
            baseline_gain = (
                upper_result['mean_curvature_1pm'] -
                lower_result['mean_curvature_1pm']
            ) / delta_pulse
            if baseline_gain <= 0:
                raise RuntimeError(
                    f'non-positive steering response gain {baseline_gain:.6f}'
                )
            learned_center = (
                lower_probe - lower_result['mean_curvature_1pm'] /
                baseline_gain
            )
            if not lower_probe <= learned_center <= upper_probe:
                print('WARNING: fitted center is outside probe bracket')
            print(f'LEARNED CENTER: {learned_center:.2f} us, '
                  f'initial gain={baseline_gain:.6f} 1/m/us')
            baseline_directional_gain = {
                'forward': (
                    upper_result['forward_curvature_1pm'] -
                    lower_result['forward_curvature_1pm']
                ) / delta_pulse,
                'reverse': (
                    upper_result['reverse_curvature_1pm'] -
                    lower_result['reverse_curvature_1pm']
                ) / delta_pulse,
            }
            if any(value <= 0 for value in baseline_directional_gain.values()):
                raise RuntimeError(
                    'non-positive forward or reverse steering probe gain'
                )
            information_increment = max(
                abs(upper_result['mean_curvature_1pm'] -
                    lower_result['mean_curvature_1pm']) / 2.0,
                3.0 * upper_result['forward_curvature_se_1pm'],
                3.0 * upper_result['reverse_curvature_se_1pm'],
                3.0 * lower_result['forward_curvature_se_1pm'],
                3.0 * lower_result['reverse_curvature_se_1pm'],
            )

            last_success = {
                'lower': lower_result,
                'upper': upper_result,
            }
            pulse_direction = {'lower': -1.0, 'upper': 1.0}
            search_sides = (('lower', 'upper') if args.search_side == 'both'
                            else (args.search_side,))
            for side in search_sides:
                previous = last_success[side]
                previous_step = abs(previous['pulse_us'] - learned_center)
                plateau_count = 0
                side_status = 'search ended unexpectedly'
                first_candidate = 2
                if args.resume_pulse is not None:
                    print(f'{side.upper()} RESUME: revalidating learned pulse '
                          f'{args.resume_pulse:.1f} us')
                    resumed = run_pair(
                        'resume', side, 2, args.resume_pulse, learned_center
                    )
                    if not resumed.get('success'):
                        reason = resumed.get(
                            'reason', 'resume candidate could not complete'
                        )
                        print(f'{side.upper()} STOP: resume validation failed: '
                              f'{reason}')
                        last_success[side]['stop_reason'] = (
                            f'resume validation failed: {reason}'
                        )
                        continue
                    previous = resumed['result']
                    if not resumed.get('clean'):
                        print(f'{side.upper()} RESUME: completed with adaptive '
                              'recovery; continuing with learned trim')
                    last_success[side] = previous
                    previous_step = abs(
                        previous['pulse_us'] -
                        (lower_probe if side == 'lower' else upper_probe)
                    )
                    first_candidate = 3
                for candidate in range(first_candidate, 33):
                    uncertainty = max(
                        previous['forward_curvature_se_1pm'],
                        previous['reverse_curvature_se_1pm'],
                        0.005,
                    )
                    desired_increment = max(
                        information_increment,
                        3.0 * uncertainty,
                    )
                    local_gain = previous.get(
                        'incremental_gain_1pm_per_us', math.nan
                    )
                    if not math.isfinite(local_gain) or local_gain <= 0:
                        local_gain = baseline_gain
                    step = desired_increment / local_gain
                    step = max(4.0 * US_PER_TICK,
                               min(1.5 * previous_step, step))
                    proposed_pulse = (previous['pulse_us'] +
                                      pulse_direction[side] * step)
                    guard = (args.absolute_min_us if side == 'lower'
                             else args.absolute_max_us)
                    already_at_guard = abs(previous['pulse_us'] - guard) < 0.5
                    if already_at_guard:
                        print(f'{side.upper()} STOP: reached immutable '
                              f'electrical guard at {guard:.1f} us without '
                              'a curvature plateau')
                        side_status = 'immutable electrical guard'
                        break
                    pulse = (max(guard, proposed_pulse) if side == 'lower'
                             else min(guard, proposed_pulse))
                    outcome = run_pair(
                        'search', side, candidate, pulse, learned_center
                    )
                    if not outcome.get('success'):
                        print(f'{side.upper()} STOP: mobility failure at '
                              f'{pulse:.1f} us: {outcome.get("reason")}')
                        side_status = f'mobility failure: {outcome.get("reason")}'
                        break
                    current = outcome['result']
                    if not current['clean_mobile']:
                        print(f'{side.upper()} CONFIRM: candidate {pulse:.1f} us '
                              'needed recovery; repeating with learned trim')
                        confirmation = run_pair(
                            'confirm', side, candidate, pulse, learned_center
                        )
                        if not confirmation.get('success'):
                            reason = confirmation.get(
                                'reason', 'confirmation could not complete'
                            )
                            print(f'{side.upper()} STOP: confirmation failed '
                                  f'at {pulse:.1f} us: {reason}')
                            side_status = f'confirmation failed: {reason}'
                            break
                        current = confirmation['result']
                        if confirmation.get('clean'):
                            print(f'{side.upper()} ACCEPT: confirmation was clean')
                        else:
                            print(f'{side.upper()} ACCEPT: confirmation completed '
                                  'with adaptive recovery; continuing')
                    delta_kappa = (
                        abs(current['mean_curvature_1pm']) -
                        abs(previous['mean_curvature_1pm'])
                    )
                    actual_step = abs(pulse - previous['pulse_us'])
                    incremental_gain = delta_kappa / actual_step
                    current['incremental_gain_1pm_per_us'] = incremental_gain
                    forward_increment = (
                        abs(current['forward_curvature_1pm']) -
                        abs(previous['forward_curvature_1pm'])
                    )
                    reverse_increment = (
                        abs(current['reverse_curvature_1pm']) -
                        abs(previous['reverse_curvature_1pm'])
                    )
                    forward_gain = forward_increment / actual_step
                    reverse_gain = reverse_increment / actual_step
                    forward_noise = 3.0 * math.hypot(
                        previous['forward_curvature_se_1pm'],
                        current['forward_curvature_se_1pm'],
                    )
                    reverse_noise = 3.0 * math.hypot(
                        previous['reverse_curvature_se_1pm'],
                        current['reverse_curvature_se_1pm'],
                    )
                    plateau = (
                        (forward_increment <= forward_noise and
                         reverse_increment <= reverse_noise) or
                        (forward_gain <= 0.20 *
                         baseline_directional_gain['forward'] and
                         reverse_gain <= 0.20 *
                         baseline_directional_gain['reverse'])
                    )
                    plateau_count = plateau_count + 1 if plateau else 0
                    if plateau_count >= 2:
                        print(f'{side.upper()} STOP: curvature response plateau')
                        side_status = 'curvature response plateau'
                        break
                    last_success[side] = current
                    previous_step = actual_step
                    previous = current
                else:
                    side_status = 'internal 32-iteration algorithm guard'
                last_success[side]['stop_reason'] = side_status

            print('FARTHEST COMPLETED, REPEATABLE CANDIDATES:')
            for side in search_sides:
                result = last_success[side]
                print(f'  {side}: pulse={result["pulse_us"]:.1f} us, '
                      f'kappa={result["mean_curvature_1pm"]:+.3f} 1/m, '
                      f'effective={result["effective_angle_deg"]:+.1f} deg, '
                      f'inner={result["inner_wheel_angle_deg"]:+.1f} deg, '
                      f'outer={result["outer_wheel_angle_deg"]:+.1f} deg, '
                      f'search_stop={result.get("stop_reason", "")})')
        except (KeyboardInterrupt, RuntimeError) as error:
            print('STOP:', error)
        finally:
            if bus is not None:
                try:
                    for _ in range(5):
                        neutral(bus)
                        time.sleep(0.02)
                    write_steering_us(
                        bus, args.channel, args.initial_center_us
                    )
                    time.sleep(0.3)
                    release_channel(bus, args.channel)
                finally:
                    bus.close()
            node.destroy_node()
            rclpy.shutdown()
            print(f'ESC neutral; steering released. Log: {args.output}')

    with open(args.summary_output, 'w', newline='') as summary_file:
        writer = csv.DictWriter(summary_file, fieldnames=result_fields)
        writer.writeheader()
        writer.writerows(results)
    print('Summary:', args.summary_output)


if __name__ == '__main__':
    main()
