#!/usr/bin/env python3
"""Failsafe: force the ESC channel to neutral, steering to no-pulse.

Runs OUTSIDE ROS (raw I2C) so it works when everything else is dead.
Wired as ExecStopPost in ackermann-drive.service: systemd runs it on EVERY
service stop — clean, crashed, or killed — so the PCA9685 (a free-running
PWM generator with no watchdog of its own) can never be left streaming a
stale throttle to the ESC. 2026-07-04, after two incidents of exactly that.
"""
import sys
import time

ADDR = 0x40
ESC_CH = 14
STEER_CH = 12
NEUTRAL_TICK = 1210     # 1.5 ms at actual 196.9 Hz (prescale 30)
PRESCALE_100HZ = 30     # (name legacy; value = ~200 Hz since 2026-07-04)


def main():
    try:
        try:
            from smbus2 import SMBus
        except ImportError:
            from smbus import SMBus
        bus = SMBus(1)
        # Ensure the chip is awake and at the expected frame rate. If the
        # driver died mid-configuration this also restores a sane state.
        bus.write_byte_data(ADDR, 0x00, 0x10)            # MODE1 sleep
        bus.write_byte_data(ADDR, 0xFE, PRESCALE_100HZ)  # 100 Hz
        bus.write_byte_data(ADDR, 0x00, 0x00)            # wake
        time.sleep(0.005)
        bus.write_byte_data(ADDR, 0x00, 0xA0)            # restart + auto-inc

        def set_ch(ch, tick):
            bus.write_i2c_block_data(ADDR, 0x06 + 4 * ch,
                                     [0, 0, tick & 0xFF, tick >> 8])

        set_ch(ESC_CH, NEUTRAL_TICK)   # ESC -> stopped
        set_ch(STEER_CH, 0)            # steering -> no pulse (servo limp)
        print("esc_neutral: ESC neutral written, steering released")
        return 0
    except Exception as exc:
        # A failed write here usually means the I2C bus is wedged — the one
        # case software cannot fix. Scream so it's visible in the journal.
        print(f"esc_neutral: FAILED to neutralize ESC: {exc}", file=sys.stderr)
        print("esc_neutral: CUT MOTOR POWER MANUALLY (bus may be wedged)",
              file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
