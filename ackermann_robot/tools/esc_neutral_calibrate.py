#!/usr/bin/env python3
"""Find the buzz-free ESC neutral: steps candidate neutral pulses while you
listen. Run with the ROBOT STACK DOWN (this script owns the bus; the
esc-watchdog recognizes it as a legitimate owner). Wheels off the ground or
clear space — a wrong neutral may creep the motor slightly; press Ctrl-C
anytime (ends at FULL_OFF release).

Usage: python3 esc_neutral_calibrate.py [start_us end_us step]
Default sweep: 1480..1520 in 5 us steps, 5 s each.
"""
import sys
import time

from smbus2 import SMBus

BUS, ADDR, ESC_CH = 1, 0x40, 14
PRESCALE = 30
FREQ = 196.8876          # this board's measured output frequency


def pulse(bus, channel, us):
    tick = int(round(us / (1_000_000.0 / (FREQ * 4096.0))))
    base = 0x06 + 4 * channel
    bus.write_i2c_block_data(ADDR, base, [0, 0, tick & 0xff, tick >> 8])


def main():
    start, end, step = 1480.0, 1520.0, 5.0
    if len(sys.argv) == 4:
        start, end, step = map(float, sys.argv[1:4])
    with SMBus(BUS) as bus:
        bus.write_byte_data(ADDR, 0x00, 0x10)
        bus.write_byte_data(ADDR, 0xFE, PRESCALE)
        bus.write_byte_data(ADDR, 0x00, 0x20)
        time.sleep(0.01)
        print('arming at 1500 us (3 s)...')
        pulse(bus, ESC_CH, 1500.0)
        time.sleep(3.0)
        try:
            us = start
            while us <= end + 0.01:
                print(f'>>> neutral candidate {us:.0f} us — listen for 5 s')
                pulse(bus, ESC_CH, us)
                time.sleep(5.0)
                us += step
            print('sweep done. Note the QUIETEST value; if it is not 1500,')
            print('tell Claude and the birth certificate gets updated.')
        finally:
            base = 0x06 + 4 * ESC_CH
            bus.write_i2c_block_data(ADDR, base, [0, 0, 0, 0x10])
            print('channel released (no pulse).')


if __name__ == '__main__':
    main()
