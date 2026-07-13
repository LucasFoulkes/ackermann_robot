#!/usr/bin/env python3
"""Characterize ESC direction-flip behavior (brake phase, rock-back, latency).

Per leg: launch in one direction, roll ~0.7 m at ~0.25 m/s, then execute a
flip variant and record what physically happens for up to 2.5 s:
  immediate   - opposite-direction sustain pulse while still rolling
  dwell_02    - neutral for 0.2 s, then opposite sustain pulse
  dwell_05    - neutral for 0.5 s, then opposite sustain pulse
Measured per flip: deceleration profile, backward-rock magnitude, and the
latency from flip command to sustained opposite motion. Feeds the cusp-exit
sequence and the direction-mismatch fault window.
"""

import argparse
import csv
import math
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
from steering_step_id import (
    measure_noise,
    wait_for_stop,
)
from steering_system_id import SteeringExperiment

STRAIGHT_PULSE = {'forward': 1441.6, 'reverse': 1475.5}
SUSTAIN_PULSE = {'forward': 1417.0, 'reverse': 1593.0}
VARIANTS = (('immediate', 0.0), ('dwell_02', 0.2), ('dwell_05', 0.5))

FIELDS = ['variant', 'pass_index', 'from_direction', 'phase', 'elapsed_s',
          'time_since_flip_s', 'esc_pulse_us', 'signed_speed_mps',
          'raw_ds_m', 'fresh_odom', 'leg_motion_m', 'front_clearance_m',
          'rear_clearance_m']


def run_leg(node, bus, args, writer, variant, dwell, from_direction,
            pass_index, started):
    sign = 1.0 if from_direction == 'forward' else -1.0
    to_direction = 'reverse' if from_direction == 'forward' else 'forward'
    node.direction = from_direction
    write_steering_us(bus, args.channel, STRAIGHT_PULSE[from_direction])
    args.target_speed = 0.25
    controller = AdaptiveThrottle(
        args, from_direction, node.model_data, node.session_trim,
        node.movement_threshold, node.breakaway_threshold)
    leg_motion = 0.0
    leg_started = time.monotonic()
    last_odom = node.odom_seq
    phase = 'cruise'
    flip_time = None
    while rclpy.ok():
        now = time.monotonic()
        if now - leg_started > 30.0:
            raise RuntimeError(f'{variant} {from_direction} leg timed out')
        rclpy.spin_once(node, timeout_sec=0.02)
        fresh = node.odom_seq != last_odom
        if fresh:
            last_odom = node.odom_seq
            leg_motion += abs(node.last_ds)
            if phase == 'cruise':
                controller.update(node.signed_speed, node.last_dt)
        if phase == 'cruise':
            pulse = controller.command
            if controller.state == 'rolling' and leg_motion >= args.runin_distance:
                phase = 'flip'
                flip_time = now
                # Flip is judged against the direction we are LEAVING: the
                # clearance behind matters, ahead we already vetted.
                node.direction = to_direction
        else:
            since = now - flip_time
            if since < dwell:
                pulse = 1500.0
            else:
                pulse = SUSTAIN_PULSE[to_direction]
            if since > 2.5:
                break
        tick = round(pulse / US_PER_TICK)
        write_tick(bus, tick)
        okay, reason = node.safe(leg_motion)
        if not okay:
            raise RuntimeError(f'{variant} {from_direction}: {reason}')
        writer.writerow({
            'variant': variant, 'pass_index': pass_index,
            'from_direction': from_direction, 'phase': phase,
            'elapsed_s': f'{now - started:.4f}',
            'time_since_flip_s': (f'{now - flip_time:.4f}'
                                  if flip_time else ''),
            'esc_pulse_us': f'{pulse:.2f}',
            'signed_speed_mps': f'{node.signed_speed:.6f}',
            'raw_ds_m': f'{node.last_ds:.7f}' if fresh else '',
            'fresh_odom': int(fresh),
            'leg_motion_m': f'{leg_motion:.6f}',
            'front_clearance_m': f'{node.front:.3f}',
            'rear_clearance_m': f'{node.rear:.3f}',
        })
    neutral(bus)
    return leg_motion


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm', action='store_true')
    parser.add_argument('--channel', type=int, default=DEFAULT_STEERING_CHANNEL)
    parser.add_argument('--passes', type=int, default=2)
    parser.add_argument('--runin-distance', type=float, default=0.7)
    parser.add_argument('--stop-range', type=float, default=0.30)
    parser.add_argument('--max-speed', type=float, default=0.55)
    parser.add_argument('--kp', type=float, default=0.0)
    parser.add_argument('--ki', type=float, default=30.0)
    parser.add_argument('--max-recoveries', type=int, default=4)
    parser.add_argument('--recovery-samples', type=int, default=18)
    parser.add_argument('--model', default='~/.robot/esc_model.yaml')
    parser.add_argument('--output', default='esc_direction_flip.csv')
    args, ros_args = parser.parse_known_args()

    if not args.arm:
        parser.error('refusing hardware output without --arm')
    if not 1 <= args.passes <= 4:
        parser.error('--passes must be 1..4')
    if not 0.5 <= args.runin_distance <= 1.5:
        parser.error('--runin-distance must be 0.5..1.5')
    args.max_leg_distance = args.runin_distance + 1.2

    rclpy.init(args=ros_args)
    node = SteeringExperiment(args)
    node.model_data, model_path = load_model(args.model)
    node.session_trim = {'forward': 0.0, 'reverse': 0.0}
    bus = None
    started = time.monotonic()

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
            print(f'ODOM NOISE: sigma={sigma:.4f}')
            for pass_index in range(1, args.passes + 1):
                for variant, dwell in VARIANTS:
                    for from_direction in ('forward', 'reverse'):
                        neutral(bus)
                        wait_for_stop(node, node.movement_threshold)
                        print(f'pass {pass_index} {variant} from '
                              f'{from_direction}')
                        motion = run_leg(node, bus, args, writer, variant,
                                         dwell, from_direction, pass_index,
                                         started)
                        out.flush()
                        print(f'  done ({motion:.2f} m)')
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


if __name__ == '__main__':
    main()
