#!/usr/bin/env python3
"""Independent ESC neutral failsafe.

The PCA9685 free-runs its last commanded pulse forever. If the process that
commanded it dies hard (SIGKILL, OOM, crash, hung I2C transaction), the robot
keeps driving with nobody in control. This daemon enforces one invariant:

    no live owner process  =>  no drive pulse on the ESC channel

It never touches the bus while an owner is alive, so it cannot interfere with
the controller or any experiment. On violation it holds neutral briefly, then
sets FULL_OFF on both ESC and steering channels (no pulse -> ESC disarms).

Runs as a systemd user service; see esc-watchdog.service.
"""

import subprocess
import time

from smbus2 import SMBus

BUS = 1
ADDR = 0x40
ESC_CHANNEL = 14
STEERING_CHANNEL = 12
NEUTRAL_US = 1500.0
POLL_S = 0.5
# Any process matching one of these owns the PCA9685 legitimately.
OWNER_PATTERNS = (
    'adaptive_ackermann_controller',
    'esc_forward_calibrate', 'esc_system_id', 'esc_speed_control_test',
    'esc_sustain_floor_id',
    'steering_sanity', 'steering_system_id', 'steering_staircase_id',
    'steering_limit_search', 'steering_step_id',
)


def owner_alive():
    try:
        out = subprocess.run(
            ['pgrep', '-f', '|'.join(OWNER_PATTERNS)],
            capture_output=True, text=True, timeout=5)
        return bool(out.stdout.strip())
    except Exception:
        return True  # fail safe toward NOT touching the bus


def read_channel(bus, channel):
    base = 0x06 + 4 * channel
    return bus.read_i2c_block_data(ADDR, base, 4)


def channel_active(regs):
    on = regs[0] | (regs[1] << 8)
    off = regs[2] | (regs[3] << 8)
    if off & 0x1000:  # FULL_OFF bit
        return False
    return on != 0 or off != 0


def neutral_tick(bus):
    prescale = bus.read_byte_data(ADDR, 0xFE)
    frequency = 25_000_000.0 / (4096.0 * (prescale + 1))
    return max(1, min(4095, round(NEUTRAL_US * 1e-6 * frequency * 4096.0)))


def write_channel(bus, channel, on, off):
    base = 0x06 + 4 * channel
    bus.write_i2c_block_data(
        ADDR, base, [on & 0xFF, on >> 8, off & 0xFF, off >> 8])


def enforce(bus):
    tick = neutral_tick(bus)
    write_channel(bus, ESC_CHANNEL, 0, tick)
    time.sleep(0.5)
    write_channel(bus, ESC_CHANNEL, 0, 0x1000)      # FULL_OFF: no pulse
    write_channel(bus, STEERING_CHANNEL, 0, 0x1000)
    print('esc_watchdog: no owner process; ESC neutralized and released',
          flush=True)


def main():
    print('esc_watchdog: monitoring', flush=True)
    while True:
        time.sleep(POLL_S)
        if owner_alive():
            continue
        try:
            with SMBus(BUS) as bus:
                if channel_active(read_channel(bus, ESC_CHANNEL)):
                    # Re-check the owner right before acting to close the
                    # race with a process that started this instant.
                    if not owner_alive():
                        enforce(bus)
        except OSError:
            time.sleep(2.0)  # bus busy or HAT unpowered; retry later


if __name__ == '__main__':
    main()
