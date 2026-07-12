#!/usr/bin/env python3
"""Measure the low-speed sustain curve and kinetic stall floor per direction.

Three leg types, shuttled forward/reverse in a ~3 m straight corridor:
  decay  - settle at 0.20 m/s, then weaken the pulse one PWM tick at a time
           (quasi-static) until rolling stalls: traces the full low-speed
           pulse/speed curve and the kinetic floor in a single leg.
  crawl  - hold 0.10 m/s closed-loop for the whole leg: does slow crawl work?
  fast   - hold 0.35 m/s: upper sustain map point.
"""

import argparse
import csv
import math
import statistics
import time

import rclpy

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
from ackermann_robot.experiments.steering_staircase_id import (
    AdaptiveThrottle,
    load_model,
)
from ackermann_robot.experiments.steering_step_id import (
    measure_noise,
    wait_for_stop,
)
from ackermann_robot.experiments.steering_system_id import SteeringExperiment

# Measured straight-line pulses (2026-07-11 step experiment: forward center
# relabeled +0.033, so kappa=0 sits at ~1441.6 us, not the old 1451.5).
STRAIGHT_PULSE = {'forward': 1441.6, 'reverse': 1475.5}

FIELDS = ['mode', 'pass_index', 'direction', 'phase', 'elapsed_s',
          'esc_pulse_us', 'esc_tick', 'signed_speed_mps', 'raw_ds_m',
          'raw_dyaw_rad', 'fresh_odom', 'leg_motion_m', 'clearance_m',
          'controller_state', 'recovery_count']


def run_leg(node, bus, args, writer, mode, direction, pass_index, started):
    sign = 1.0 if direction == 'forward' else -1.0
    node.direction = direction
    write_steering_us(bus, args.channel, STRAIGHT_PULSE[direction])
    targets = {'decay': 0.20, 'crawl': args.crawl_speed, 'fast': args.fast_speed}
    args.target_speed = targets[mode]
    controller = AdaptiveThrottle(
        args, direction, node.model_data, node.session_trim,
        node.movement_threshold, node.breakaway_threshold)
    leg_motion = 0.0
    leg_started = time.monotonic()
    last_odom = node.odom_seq
    phase = 'closed_loop'
    decay_pulse = None
    last_decay_step = 0.0
    floor_report = None
    while rclpy.ok():
        now = time.monotonic()
        if now - leg_started > args.leg_timeout:
            raise RuntimeError(f'{mode} {direction} leg timed out')
        rclpy.spin_once(node, timeout_sec=0.02)
        fresh = node.odom_seq != last_odom
        if fresh:
            last_odom = node.odom_seq
            leg_motion += abs(node.last_ds)
            if phase == 'closed_loop':
                controller.update(node.signed_speed, node.last_dt)
        if phase == 'closed_loop':
            pulse = controller.command
            if (mode == 'decay' and controller.state == 'rolling' and
                    leg_motion >= args.settle_distance):
                phase = 'decay'
                decay_pulse = controller.command
                last_decay_step = now
                print(f'  decay starts at {decay_pulse:.1f} us, '
                      f'v={node.signed_speed:+.3f}')
        else:
            if now - last_decay_step >= args.decay_dwell:
                decay_pulse += US_PER_TICK if sign > 0 else -US_PER_TICK
                last_decay_step = now
            pulse = decay_pulse
            if abs(node.signed_speed) < args.stall_speed:
                floor_report = (decay_pulse, node.signed_speed)
                print(f'  STALL at {decay_pulse:.1f} us')
                break
        tick = round(pulse / US_PER_TICK)
        write_tick(bus, tick)
        write_steering_us(bus, args.channel, STRAIGHT_PULSE[direction])
        okay, reason = node.safe(leg_motion)
        if not okay:
            raise RuntimeError(f'{mode} {direction}: {reason}')
        writer.writerow({
            'mode': mode, 'pass_index': pass_index, 'direction': direction,
            'phase': phase, 'elapsed_s': f'{now - started:.4f}',
            'esc_pulse_us': f'{pulse:.3f}', 'esc_tick': tick,
            'signed_speed_mps': f'{node.signed_speed:.6f}',
            'raw_ds_m': f'{node.last_ds:.7f}' if fresh else '',
            'raw_dyaw_rad': f'{node.last_dyaw:.7f}' if fresh else '',
            'fresh_odom': int(fresh), 'leg_motion_m': f'{leg_motion:.6f}',
            'clearance_m': f'{node.front if sign > 0 else node.rear:.3f}',
            'controller_state': controller.state,
            'recovery_count': controller.recovery_count,
        })
        if phase == 'closed_loop' and leg_motion >= args.leg_distance:
            break
    neutral(bus)
    return floor_report, controller.recovery_count, leg_motion


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm', action='store_true')
    parser.add_argument('--channel', type=int, default=DEFAULT_STEERING_CHANNEL)
    parser.add_argument('--passes', type=int, default=2)
    parser.add_argument('--leg-distance', type=float, default=2.2)
    parser.add_argument('--settle-distance', type=float, default=0.8)
    parser.add_argument('--decay-dwell', type=float, default=0.8)
    parser.add_argument('--stall-speed', type=float, default=0.03)
    parser.add_argument('--crawl-speed', type=float, default=0.10)
    parser.add_argument('--fast-speed', type=float, default=0.35)
    parser.add_argument('--leg-timeout', type=float, default=60.0)
    parser.add_argument('--stop-range', type=float, default=0.25)
    parser.add_argument('--max-speed', type=float, default=0.55)
    parser.add_argument('--kp', type=float, default=0.0)
    parser.add_argument('--ki', type=float, default=30.0)
    parser.add_argument('--max-recoveries', type=int, default=6)
    parser.add_argument('--recovery-samples', type=int, default=18)
    parser.add_argument('--model', default='~/.robot/esc_model.yaml')
    parser.add_argument('--output', default='esc_sustain_floor.csv')
    args, ros_args = parser.parse_known_args()

    if not args.arm:
        parser.error('refusing hardware output without --arm')
    if not 1 <= args.passes <= 5:
        parser.error('--passes must be 1..5')
    if not 1.5 <= args.leg_distance <= 4.0:
        parser.error('--leg-distance must be 1.5..4.0')
    if not 0.06 <= args.crawl_speed <= 0.15:
        parser.error('--crawl-speed must be 0.06..0.15')
    if not 0.30 <= args.fast_speed <= 0.36:
        parser.error('--fast-speed must be 0.30..0.36')
    if not 0.4 <= args.decay_dwell <= 2.0:
        parser.error('--decay-dwell must be 0.4..2.0')
    args.max_leg_distance = args.leg_distance + 0.5

    rclpy.init(args=ros_args)
    node = SteeringExperiment(args)
    node.model_data, model_path = load_model(args.model)
    node.session_trim = {'forward': 0.0, 'reverse': 0.0}
    bus = None
    started = time.monotonic()
    floors = []

    with open(args.output, 'w', newline='') as out:
        writer = csv.DictWriter(out, fieldnames=FIELDS)
        writer.writeheader()
        try:
            bus = open_bus()
            neutral(bus)
            write_steering_us(bus, args.channel, STRAIGHT_PULSE['forward'])
            node.spin_for(3.0)
            sigma, node.movement_threshold = measure_noise(node)
            node.breakaway_threshold = max(node.movement_threshold, 0.045)
            print(f'Throttle model: {model_path}')
            print(f'ODOM NOISE: sigma={sigma:.4f}, '
                  f'movement={node.movement_threshold:.4f} m/s')
            for pass_index in range(1, args.passes + 1):
                for mode in ('decay', 'crawl', 'fast'):
                    for direction in ('forward', 'reverse'):
                        neutral(bus)
                        wait_for_stop(node, node.movement_threshold)
                        print(f'pass {pass_index} {mode} {direction}')
                        floor, recoveries, motion = run_leg(
                            node, bus, args, writer, mode, direction,
                            pass_index, started)
                        out.flush()
                        print(f'  done: {motion:.2f} m, '
                              f'{recoveries} recoveries'
                              + (f', floor {floor[0]:.1f} us' if floor else ''))
                        if floor:
                            floors.append((pass_index, direction) + floor)
        except (KeyboardInterrupt, RuntimeError) as error:
            print('STOP:', error)
        finally:
            if bus is not None:
                try:
                    for _ in range(5):
                        neutral(bus)
                        time.sleep(0.02)
                    write_steering_us(bus, args.channel,
                                      STRAIGHT_PULSE['forward'])
                    time.sleep(0.3)
                    release_channel(bus, args.channel)
                finally:
                    bus.close()
            node.destroy_node()
            rclpy.shutdown()
            print(f'ESC neutral; steering released. Log: {args.output}')
    if floors:
        print('\nstall floors (pass, direction, pulse_us, speed_at_stall):')
        for row in floors:
            print(f'  {row[0]} {row[1]:>7} {row[2]:.1f} us {row[3]:+.3f} m/s')


if __name__ == '__main__':
    main()
