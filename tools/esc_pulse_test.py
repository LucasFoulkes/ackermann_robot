#!/usr/bin/env python3
"""Ground-truth ESC pulse test — raw microseconds straight to the PCA9685.

Bypasses ROS, the driver, the dither, the maps, and esc_inverted: what you
type is the pulse the ESC gets. For settling the fwd/rev symmetry question
(2026-07-05): mirrored pulses (e.g. 1600 vs 1400) should give mirrored wheel
speeds if the ESC is symmetric.

WHEELS OFF THE GROUND. Stop the stack and kill zombies first:
    sudo systemctl stop ackermann-drive && pkill -f ackermann_driver

Usage:
    python3 esc_pulse_test.py            # interactive: type pulse in us
    python3 esc_pulse_test.py 1400       # stream one value until Ctrl-C

Type a number (1000-2000) to stream it; 'n' or empty = neutral; 'q' quits
(always leaves the ESC at neutral). NOTE with the motor wires as wired
before 2026-07-05, <1500 us drives the robot FORWARD (inverted wiring);
after the wire swap it follows the standard convention.
"""
import sys
import time

ADDR = 0x40
ESC_CH = 14
PRESCALE = 30                     # ~196.9 Hz actual
FREQ = 25_000_000 / 4096 / (PRESCALE + 1)   # exact frame rate
US_PER_TICK = 1e6 / FREQ / 4096
NEUTRAL_US = 1500.0


def make_bus():
    try:
        from smbus2 import SMBus
    except ImportError:
        from smbus import SMBus
    bus = SMBus(1)
    bus.write_byte_data(ADDR, 0x00, 0x10)      # MODE1 sleep
    bus.write_byte_data(ADDR, 0xFE, PRESCALE)
    bus.write_byte_data(ADDR, 0x00, 0x00)      # wake
    time.sleep(0.005)
    bus.write_byte_data(ADDR, 0x00, 0xA0)      # restart + auto-increment
    return bus


def set_pulse(bus, us):
    tick = max(0, min(4095, round(us / US_PER_TICK)))
    bus.write_i2c_block_data(ADDR, 0x06 + 4 * ESC_CH,
                             [0, 0, tick & 0xFF, tick >> 8])
    return tick


def main():
    bus = make_bus()
    print(f"frame rate {FREQ:.1f} Hz, {US_PER_TICK:.3f} us/tick")
    set_pulse(bus, NEUTRAL_US)
    print("ESC at neutral (1500 us). Arm it (power ESC on now if it wasn't).")
    try:
        if len(sys.argv) > 1:
            us = float(sys.argv[1])
            t = set_pulse(bus, us)
            print(f"streaming {us:.0f} us (tick {t}) — Ctrl-C for neutral+exit")
            while True:
                time.sleep(0.5)
        while True:
            raw = input("pulse us (n=neutral, q=quit)> ").strip().lower()
            if raw in ("q", "quit"):
                break
            if raw in ("", "n", "neutral"):
                set_pulse(bus, NEUTRAL_US)
                print("neutral")
                continue
            try:
                us = float(raw)
            except ValueError:
                print("number, 'n', or 'q'")
                continue
            if not 900 <= us <= 2100:
                print("refusing: outside 900-2100 us")
                continue
            t = set_pulse(bus, us)
            print(f"-> {us:.0f} us (tick {t})")
    except KeyboardInterrupt:
        pass
    finally:
        set_pulse(bus, NEUTRAL_US)
        print("\nESC returned to neutral.")


if __name__ == "__main__":
    sys.exit(main())
