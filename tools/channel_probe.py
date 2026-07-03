#!/usr/bin/env python3
"""Interactive PCA9685 channel probe — identify which channel is the steering
servo vs the ESC after rewiring (HW-039 H-bridge -> single-channel ESC).

Servo/ESC share the same 50 Hz servo PWM:
    1000 us = full one way   1500 us = center / NEUTRAL   2000 us = full other way
Tell them apart:
  * steering servo -> arm/wheels move to an ANGLE and HOLD there
  * ESC            -> motor SPINS; 1500 us = stopped, >1500 forward, <1500 reverse
                     (most ESCs need ~2 s at 1500 us first to ARM, then beep)

SAFETY
  * Put the DRIVE WHEELS OFF THE GROUND before running — the ESC will spin them.
  * Stop the drive stack first (it also writes the PCA over I2C):
        pkill -f ackermann_driver   # or Ctrl-C the launch
Run:
        python3 channel_probe.py
Commands:
  c <ch> <us>   set channel <ch> to <us> microseconds   e.g.  c 0 1500
  sweep <ch>    gently sweep <ch> 1400<->1600 us a few times (watch it)
  n             set the two probed channels (0 and 15) to neutral 1500 us
  off           release ALL channels (duty 0)
  q             quit (releases all channels)
"""
import sys
import time

from adafruit_pca9685 import PCA9685

PWM_HZ = 50.0
PERIOD_US = 1_000_000.0 / PWM_HZ          # 20000 us at 50 Hz
PROBE_CHANNELS = (0, 15)                   # first and last


def open_i2c(bus: int = 1):
    try:
        from adafruit_extended_bus import ExtendedI2C
        return ExtendedI2C(bus)
    except ImportError:
        import board
        return board.I2C()


def us_to_tick(us: float) -> int:
    return max(0, min(4095, round(us * 4096.0 / PERIOD_US)))


class Probe:
    def __init__(self):
        self.pca = PCA9685(open_i2c(), address=0x40, reference_clock_speed=25_000_000)
        self.pca.frequency = PWM_HZ

    def set_us(self, ch: int, us: float):
        self.pca.channels[ch].duty_cycle = us_to_tick(us) << 4
        print(f"  ch{ch:>2} -> {us:.0f} us (tick {us_to_tick(us)})")

    def off(self, ch=None):
        chans = range(16) if ch is None else [ch]
        for c in chans:
            self.pca.channels[c].duty_cycle = 0

    def sweep(self, ch: int):
        print(f"  sweeping ch{ch} 1400<->1600 us — watch which actuator moves")
        for _ in range(3):
            for us in list(range(1400, 1601, 20)) + list(range(1600, 1399, -20)):
                self.pca.channels[ch].duty_cycle = us_to_tick(us) << 4
                time.sleep(0.02)
        self.set_us(ch, 1500)


def main():
    p = Probe()
    print(__doc__)
    print("Arming: both probe channels to neutral (1500 us)...")
    for ch in PROBE_CHANNELS:
        p.set_us(ch, 1500)
    time.sleep(2.0)
    print("Ready. Type a command (h for help).")
    try:
        for line in sys.stdin:
            t = line.split()
            if not t:
                continue
            cmd = t[0]
            if cmd == "q":
                break
            elif cmd == "h":
                print(__doc__)
            elif cmd == "c" and len(t) == 3:
                p.set_us(int(t[1]), float(t[2]))
            elif cmd == "sweep" and len(t) == 2:
                p.sweep(int(t[1]))
            elif cmd == "n":
                for ch in PROBE_CHANNELS:
                    p.set_us(ch, 1500)
            elif cmd == "off":
                p.off()
                print("  all channels released")
            else:
                print("  ?  c <ch> <us> | sweep <ch> | n | off | q")
    except KeyboardInterrupt:
        pass
    finally:
        p.off()
        print("\nreleased all channels, bye")


if __name__ == "__main__":
    main()
