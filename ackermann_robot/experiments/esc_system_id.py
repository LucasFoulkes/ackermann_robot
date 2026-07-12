#!/usr/bin/env python3
"""Safe bidirectional kick/sustain/coast ESC system-identification test."""
import argparse, csv, math, random, time
import rclpy
from ackermann_robot.experiments.esc_forward_calibrate import (
    ADDR, CHANNEL, NEUTRAL, US_PER_TICK, Experiment, neutral, open_bus, write_tick)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--arm', action='store_true')
    p.add_argument('--sustain-seconds', type=float, default=1.5)
    p.add_argument('--max-speed', type=float, default=.35)
    p.add_argument('--max-distance', type=float, default=1.50)
    p.add_argument('--stop-range', type=float, default=.20)
    p.add_argument('--output', default='esc_system_id_fine.csv')
    args, ros_args = p.parse_known_args()
    if not args.arm: p.error('refusing hardware output without --arm')
    if not .4 <= args.sustain_seconds <= 2.0:
        p.error('--sustain-seconds must be 0.4..2.0')
    args.breakaway_speed = .05

    rclpy.init(args=ros_args); bus = None
    with open(args.output, 'w', newline='') as output:
        writer = csv.writer(output)
        writer.writerow(['trial','direction','phase','elapsed_s','pulse_us',
                         'tick','signed_speed_mps','acceleration_mps2',
                         'distance_m','clearance_m'])
        node = Experiment(args, writer=None)
        started = time.monotonic(); last_t = last_v = None

        def sample(trial, direction, phase, pulse, tick):
            nonlocal last_t, last_v
            now, velocity = time.monotonic(), node.signed_speed
            accel = ((velocity-last_v)/(now-last_t)
                     if last_t is not None and now > last_t else 0.0)
            last_t, last_v = now, velocity
            clearance = node.front if direction == 'forward' else node.rear
            writer.writerow([trial,direction,phase,f'{now-started:.4f}',
                             f'{pulse:.2f}',tick,f'{velocity:.6f}',
                             f'{accel:.6f}',f'{node.distance:.6f}',
                             f'{clearance:.3f}'])

        def phase(trial, direction, name, pulse, seconds, stop_on_motion=False):
            error = 0.0; end = time.monotonic()+seconds
            sign = 1.0 if direction == 'forward' else -1.0
            while rclpy.ok() and time.monotonic() < end:
                ideal = pulse/US_PER_TICK + error
                tick = round(ideal); error = ideal-tick
                write_tick(bus, tick)
                rclpy.spin_once(node, timeout_sec=.02)
                sample(trial, direction, name, pulse, tick)
                okay, reason = node.safe()
                if not okay: return reason
                if stop_on_motion and sign*node.signed_speed >= .05:
                    return 'motion'
            return 'complete'

        try:
            bus = open_bus(); neutral(bus); node.spin_for(3)
            # Fixed seed makes trial order reproducible while avoiding a
            # monotonic strength/time correlation.
            # First system-ID run showed all wider/near-neutral candidates
            # decelerated to a stop. Concentrate on the narrow region between
            # reliable breakaway and the strongest failed sustain value.
            forward = [1412.5, 1413., 1413.5, 1414., 1414.5]
            reverse = [1593.5, 1595., 1596.5, 1598., 1599.]
            random.Random(1060).shuffle(forward)
            random.Random(1061).shuffle(reverse)
            trials = []
            for i in range(5):
                pair = [('forward', forward[i]), ('reverse', reverse[i])]
                if i % 2: pair.reverse()
                trials.extend(pair)

            print('SYSTEM-ID: learned kicks are fixed at forward=1401 us, '
                  'reverse=1610 us; no breakaway ramps will run')
            print(f'SYSTEM-ID: {len(trials)} kick/sustain/coast trials, '
                  f'obstacle stop={args.stop_range:.2f} m')
            kick_level = {'forward': 1401.0, 'reverse': 1610.0}

            for trial, (direction, sustain) in enumerate(trials, 1):
                node.direction = direction; node.origin = node.pose
                node.signed_speed = node.speed = 0.0
                last_t = last_v = None
                neutral(bus); node.spin_for(1.5)
                # Reuse the learned kick. If surface/load variation prevents
                # breakaway, strengthen it in small bounded steps instead of
                # rerunning a full threshold sweep. Remember success for the
                # remaining trials in this direction.
                kick = kick_level[direction]
                for attempt in range(4):
                    result = phase(trial, direction, 'kick', kick, .7, True)
                    if result == 'motion':
                        kick_level[direction] = kick
                        break
                    if result != 'complete':
                        raise RuntimeError(f'trial {trial} kick safety: {result}')
                    kick += -2.0 if direction == 'forward' else 2.0
                    print(f'trial {trial} {direction}: no breakaway; '
                          f'retrying kick at {kick:.1f} us')
                if result != 'motion':
                    raise RuntimeError(f'trial {trial} kick failed after bounded retries')
                result = phase(trial, direction, 'sustain', sustain,
                               args.sustain_seconds)
                neutral(bus)
                if result != 'complete':
                    print(f'trial {trial} bounded stop: {result}; neutral, continuing')
                # Observe coast until almost stopped, bounded to 1.5 s.
                end = time.monotonic()+1.5
                while rclpy.ok() and time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=.02)
                    sample(trial, direction, 'coast', NEUTRAL,
                           round(NEUTRAL/US_PER_TICK))
                    if abs(node.signed_speed) < .025: break
                output.flush()
                print(f'trial {trial}/10 {direction}: sustain {sustain:.0f} us, '
                      f'end speed {node.signed_speed:+.3f} m/s')
        except (KeyboardInterrupt, RuntimeError) as error:
            print(f'STOP: {error}')
        finally:
            if bus:
                for _ in range(5): neutral(bus); time.sleep(.02)
                bus.close()
            node.destroy_node(); rclpy.shutdown()
            print(f'ESC neutral. Log: {args.output}')


if __name__ == '__main__': main()
