#!/usr/bin/env python3
"""Safely verify the PCA9685 steering channel, polarity, and rough center."""

import argparse
import csv
import time

from ackermann_robot.experiments.esc_forward_calibrate import (
    ADDR,
    NEUTRAL,
    US_PER_TICK,
    neutral,
    open_bus,
)


DEFAULT_STEERING_CHANNEL = 12


def write_channel_tick(bus, channel, tick):
    tick = max(0, min(4095, int(tick)))
    bus.write_i2c_block_data(
        ADDR, 0x06 + 4 * channel, [0, 0, tick & 0xFF, tick >> 8]
    )


def write_steering_us(bus, channel, pulse_us):
    tick = round(pulse_us / US_PER_TICK)
    write_channel_tick(bus, channel, tick)
    return tick


def release_channel(bus, channel):
    """Set PCA9685 FULL_OFF for one channel so the servo is not held."""
    bus.write_i2c_block_data(
        ADDR, 0x06 + 4 * channel, [0, 0, 0, 0x10]
    )


def hold_command(bus, channel, pulse_us, seconds):
    """Hold steering while repeatedly ensuring that the ESC remains neutral."""
    tick = write_steering_us(bus, channel, pulse_us)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        neutral(bus)
        time.sleep(0.05)
    return tick


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm', action='store_true',
                        help='allow PCA9685 hardware output')
    parser.add_argument('--wheels-lifted', action='store_true',
                        help='confirm that the front wheels are clear of the floor')
    parser.add_argument('--channel', type=int, default=DEFAULT_STEERING_CHANNEL)
    parser.add_argument('--center-us', type=float, default=1500.0)
    parser.add_argument('--offset-us', type=float, default=50.0)
    parser.add_argument('--hold-seconds', type=float, default=2.0)
    parser.add_argument('--output', default='steering_sanity.csv')
    args = parser.parse_args()

    if not args.arm:
        parser.error('refusing hardware output without --arm')
    if not args.wheels_lifted:
        parser.error('first steering test requires --wheels-lifted')
    if not 0 <= args.channel <= 15:
        parser.error('--channel must be 0..15')
    if not 1400.0 <= args.center_us <= 1600.0:
        parser.error('--center-us must remain within 1400..1600 for this test')
    if not 10.0 <= args.offset_us <= 75.0:
        parser.error('--offset-us must be 10..75 for this test')
    if not 1.0 <= args.hold_seconds <= 4.0:
        parser.error('--hold-seconds must be 1..4')

    lower = args.center_us - args.offset_us
    higher = args.center_us + args.offset_us
    if lower < 1350.0 or higher > 1650.0:
        parser.error('requested sequence exceeds the conservative 1350..1650 us envelope')

    sequence = [
        ('center_initial', args.center_us),
        ('lower_pulse', lower),
        ('center_from_lower', args.center_us),
        ('higher_pulse', higher),
        ('center_from_higher', args.center_us),
    ]

    bus = None
    started = time.monotonic()
    with open(args.output, 'w', newline='') as output:
        writer = csv.writer(output)
        writer.writerow([
            'step', 'label', 'elapsed_s', 'channel', 'pulse_us', 'tick',
            'esc_pulse_us', 'hold_seconds'
        ])
        try:
            print('STEERING SANITY: ESC will remain neutral.')
            print('Watch the front wheels and listen for binding or servo buzz.')
            bus = open_bus()
            neutral(bus)
            time.sleep(1.0)
            for step, (label, pulse_us) in enumerate(sequence, 1):
                print(f'{step}/{len(sequence)} {label}: {pulse_us:.1f} us')
                tick = hold_command(
                    bus, args.channel, pulse_us, args.hold_seconds
                )
                writer.writerow([
                    step, label, f'{time.monotonic() - started:.3f}',
                    args.channel, f'{pulse_us:.3f}', tick, f'{NEUTRAL:.1f}',
                    f'{args.hold_seconds:.3f}'
                ])
                output.flush()
            print('COMPLETE: report which direction lower and higher pulse moved,')
            print('whether both center returns match, and any buzz or binding.')
        except KeyboardInterrupt:
            print('STOP: interrupted')
        finally:
            if bus is not None:
                try:
                    for _ in range(5):
                        neutral(bus)
                        time.sleep(0.02)
                    release_channel(bus, args.channel)
                finally:
                    bus.close()
            print(f'ESC neutral; steering released. Log: {args.output}')


if __name__ == '__main__':
    main()
