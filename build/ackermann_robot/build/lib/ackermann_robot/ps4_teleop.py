#!/usr/bin/env python3
import subprocess
import time
import sys
import os
import glob
import select
import threading
from dataclasses import dataclass

try:
    import evdev
    from evdev import ecodes
except ImportError:
    sys.exit("Error: Please install evdev (pip install evdev)")

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


@dataclass
class Config:
    mac: str = "00:10:80:26:6B:1A"
    # Minimum deadzone in percent (even if kernel flat is 0)
    min_deadzone_pct: int = 4
    # Optional per-axis center overrides (set to int to force, else None)
    lx_center_override: int | None = None
    ly_center_override: int | None = None
    rx_center_override: int | None = None
    ry_center_override: int | None = None


class DualShock4:
    def __init__(self, config: Config):
        self.cfg = config
        self.device = None
        self.axes = {
            ecodes.ABS_X: 0,    # Left stick X
            ecodes.ABS_Y: 0,    # Left stick Y
            ecodes.ABS_RX: 0,   # Right stick X
            ecodes.ABS_RY: 0,   # Right stick Y
        }
        self._abs_cache = {}

    def connect_bluetooth(self, retries=12) -> bool:
        print(f"Connecting to {self.cfg.mac}...")
        script = "\n".join(
            [
                "power on",
                "agent on",
                "default-agent",
                "pairable on",
                f"trust {self.cfg.mac}",
                f"pair {self.cfg.mac}",
                f"connect {self.cfg.mac}",
                "quit",
            ]
        )

        for i in range(1, retries + 1):
            res = subprocess.run(["bluetoothctl"], input=script, text=True, capture_output=True).stdout
            info = subprocess.run(
                ["bluetoothctl", "info", self.cfg.mac], text=True, capture_output=True
            ).stdout

            if "Connection successful" in res or "Connected: yes" in info:
                print("Connection successful")
                time.sleep(2)
                return True

            print(f"Attempt {i}/{retries} failed. Put controller in pairing mode (Share+PS)...")
            time.sleep(2)

        return False

    def find_device(self, retries=30):
        print(f"Waiting for input device {self.cfg.mac}...")
        for _ in range(retries):
            for path in evdev.list_devices():
                try:
                    d = evdev.InputDevice(path)
                    if self._is_gamepad(d):
                        print(f"Found: {d.name} at {path}")
                        self.device = d
                        self._prime_abs_cache()
                        return True
                except (OSError, AttributeError):
                    pass
            time.sleep(1)
        return False

    def _is_gamepad(self, dev):
        if any(x in dev.name for x in ("Touchpad", "Motion Sensors")):
            return False
        if not (dev.uniq and dev.uniq.upper() == self.cfg.mac.upper()):
            return False
        caps = dev.capabilities()
        if ecodes.EV_ABS not in caps or ecodes.EV_KEY not in caps:
            return False
        abs_caps = dict(caps[ecodes.EV_ABS])
        keys = caps[ecodes.EV_KEY]
        # Basic check: has ABS_X and a face button
        return (ecodes.ABS_X in abs_caps) and (ecodes.BTN_SOUTH in keys or ecodes.BTN_A in keys)

    @property
    def battery(self):
        clean_mac = self.cfg.mac.replace(":", "").lower()
        for path in glob.glob("/sys/class/power_supply/*"):
            if clean_mac in path.lower() or self.cfg.mac.lower() in path.lower():
                try:
                    with open(os.path.join(path, "capacity"), "r") as f:
                        return int(f.read().strip())
                except (OSError, ValueError):
                    pass
        return None

    def _prime_abs_cache(self):
        """Cache absinfo for the four stick axes and print them once."""
        axis_list = [ecodes.ABS_X, ecodes.ABS_Y, ecodes.ABS_RX, ecodes.ABS_RY]
        names = {
            ecodes.ABS_X: "LX",
            ecodes.ABS_Y: "LY",
            ecodes.ABS_RX: "RX",
            ecodes.ABS_RY: "RY",
        }

        print("\nAxis absinfo (authoritative per-axis range/deadzone):")
        for code in axis_list:
            try:
                ai = self.device.absinfo(code)
                self._abs_cache[code] = ai
                print(
                    f"  {names[code]:>2} ({code:>3}) "
                    f"min={ai.min:<4} max={ai.max:<4} flat={ai.flat:<4} fuzz={ai.fuzz:<4} res={ai.resolution}"
                )
            except Exception as e:
                self._abs_cache[code] = None
                print(f"  {names[code]:>2} ({code:>3}) absinfo unavailable: {e}")

        print("")  # newline

    def _center_override_for(self, code):
        return {
            ecodes.ABS_X: self.cfg.lx_center_override,
            ecodes.ABS_Y: self.cfg.ly_center_override,
            ecodes.ABS_RX: self.cfg.rx_center_override,
            ecodes.ABS_RY: self.cfg.ry_center_override,
        }.get(code)

    def _center_for(self, code, ai):
        ov = self._center_override_for(code)
        if ov is not None:
            return int(ov)
        return (ai.min + ai.max) // 2

    def _deadzone_pct_for(self, ai):
        half = max(1.0, (ai.max - ai.min) / 2.0)
        dz = (ai.flat / half) * 100.0 if getattr(ai, "flat", 0) else 0.0
        return max(float(self.cfg.min_deadzone_pct), dz)

    def _scale_axis(self, code, invert=False):
        """Scale an EV_ABS axis to -100..100 using per-axis min/max/flat."""
        ai = self._abs_cache.get(code) or self.device.absinfo(code)
        if ai is None:
            return 0

        val = self.axes.get(code, 0)
        amin, amax = ai.min, ai.max
        center = self._center_for(code, ai)

        if val >= center:
            denom = amax - center
            pct = 0.0 if denom == 0 else (val - center) / denom * 100.0
        else:
            denom = center - amin
            pct = 0.0 if denom == 0 else -((center - val) / denom * 100.0)

        dz = self._deadzone_pct_for(ai)
        if abs(pct) < dz:
            pct = 0.0

        pct = max(-100.0, min(100.0, pct))
        if invert:
            pct = -pct

        return int(round(pct))

    def monitor(self, callback=None):
        print("Monitoring... (Ctrl+C to stop)")
        last_batt = None

        try:
            while True:
                batt = self.battery
                if batt is not None and batt != last_batt:
                    print(f"\nBattery Level: {batt}%")
                    last_batt = batt

                r, _, _ = select.select([self.device.fd], [], [], 1.0)
                if r:
                    for event in self.device.read():
                        if event.type == ecodes.EV_ABS and event.code in self.axes:
                            self.axes[event.code] = event.value

                            # Many controllers report "up" as smaller raw values on Y axes.
                            # Invert LY and RY so pushing up becomes +%.
                            lx = self._scale_axis(ecodes.ABS_X, invert=False)
                            ly = self._scale_axis(ecodes.ABS_Y, invert=True)
                            rx = self._scale_axis(ecodes.ABS_RX, invert=False)
                            ry = self._scale_axis(ecodes.ABS_RY, invert=True)

                            if callback:
                                callback(lx, ly, rx, ry)

                            print(
                                f"\rLX: {lx:>4}% | LY: {ly:>4}% | RX: {rx:>4}% | RY: {ry:>4}%",
                                end="",
                                flush=True,
                            )
        except KeyboardInterrupt:
            print("\nExiting.")
        except OSError:
            print("\nDevice disconnected.")


class PS4TeleopNode(Node):
    def __init__(self):
        super().__init__("ps4_teleop")
        self.declare_parameter("cmd_topic", "/ackermann/cmd")
        self._cmd_topic = self.get_parameter("cmd_topic").value
        
        self.pub = self.create_publisher(Float32MultiArray, self._cmd_topic, 10)
        self.get_logger().info(f"Publishing to {self._cmd_topic}")

        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def _publish_command(self, lx, ly, rx, ry):
        # User request: Left Joystick X (lx) -> Steering
        #               Right Joystick Y (ry) -> Motor
        msg = Float32MultiArray()
        msg.data = [float(lx), float(ry)]
        self.pub.publish(msg)

    def _worker(self):
        if not os.access('/dev/input/event0', os.R_OK):
            print("Warning: You may need 'sudo' to access input devices.")

        ds4 = DualShock4(Config())
        if ds4.connect_bluetooth() and ds4.find_device():
            ds4.monitor(callback=self._publish_command)
        else:
            self.get_logger().error("Failed to connect or find input device.")

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PS4TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
