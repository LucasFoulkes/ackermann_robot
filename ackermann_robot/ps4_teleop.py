#!/usr/bin/env python3
import math
import subprocess
import threading
import time
from pathlib import Path

try:
    import evdev
    from evdev import ecodes as e
except ImportError:
    raise SystemExit("Error: Please install evdev (pip install evdev)")

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray


class PS4TeleopNode(Node):
    def __init__(self):
        super().__init__("ps4_teleop")

        p = lambda name, default: self.declare_parameter(name, default).value
        self.mac = str(p("mac_address", "00:10:80:26:6B:1A")).upper()
        self.dz = float(p("min_deadzone_pct", 4.0)) / 100.0

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(Float32MultiArray, str(p("cmd_topic", "/ackermann/cmd_effort")), qos)

        # Tuples are implicitly thread-safe in Python (GIL atomic assignment)
        self.cmd = (0.0, 0.0)
        self.battery_path = None
        self.last_batt = None

        self.create_timer(1.0 / max(1.0, float(p("publish_hz", 50.0))), self._publish_cb)
        self.create_timer(10.0, self._battery_cb)

        threading.Thread(target=self._worker, daemon=True).start()

    # -- ROS Timers -----------------------------------------------------------

    def _publish_cb(self):
        self.pub.publish(Float32MultiArray(data=list(self.cmd)))

    def _battery_cb(self):
        if not self.battery_path:
            return
        try:
            if (batt := int(self.battery_path.read_text().strip())) != self.last_batt:
                self.get_logger().info(f"Controller Battery: {batt}%")
                self.last_batt = batt
        except (OSError, ValueError):
            pass

    # -- Initialization & Math ------------------------------------------------

    def _ensure_bluetooth(self) -> bool:
        def check():
            return "Connected: yes" in subprocess.run(
                ["bluetoothctl", "info", self.mac], capture_output=True, text=True
            ).stdout

        if check():
            return True

        self.get_logger().info(f"Connecting to {self.mac}...")
        subprocess.run(
            ["bluetoothctl"],
            input=f"power on\ntrust {self.mac}\nconnect {self.mac}\nquit\n",
            text=True, capture_output=True,
        )
        time.sleep(1.0)
        return check()

    def _find_device(self):
        for path in evdev.list_devices():
            try:
                dev = evdev.InputDevice(path)
                if (dev.uniq and dev.uniq.upper() == self.mac
                        and e.ABS_X in dev.capabilities(absinfo=False).get(e.EV_ABS, [])):
                    return dev
            except OSError:
                pass
        return None

    def _scale(self, val: float, ai) -> float:
        """Rescale raw axis value to -1..1 with deadzone and smooth stretch."""
        center = (ai.max + ai.min) / 2.0
        rng = max(1.0, (ai.max - ai.min) / 2.0)

        norm = (val - center) / rng
        dz = max(self.dz, ai.flat / rng)

        if abs(norm) < dz:
            return 0.0

        rescaled = (abs(norm) - dz) / (1.0 - dz)
        return float(max(-1.0, min(1.0, math.copysign(rescaled, norm))))

    # -- Hardware Threading ---------------------------------------------------

    def _worker(self):
        targets = {e.ABS_X: 0, e.ABS_RY: 0}

        while rclpy.ok():
            if not self._ensure_bluetooth() or not (dev := self._find_device()):
                self.cmd = (0.0, 0.0)
                time.sleep(3.0)
                continue

            self.get_logger().info(f"Connected: {dev.name} - Ready to Drive!")

            info = {code: dev.absinfo(code) for code in targets}
            axes = {code: info[code].value for code in targets}

            mac_cln = self.mac.replace(":", "").lower()
            mac_lwr = self.mac.lower()
            self.battery_path = next(
                (p / "capacity" for p in Path("/sys/class/power_supply").iterdir()
                 if mac_cln in p.name.lower() or mac_lwr in p.name.lower()),
                None,
            )

            try:
                dirty = False
                for ev in dev.read_loop():
                    if ev.type == e.EV_ABS and ev.code in axes:
                        axes[ev.code] = ev.value
                        dirty = True

                    elif ev.type == e.EV_SYN and dirty:
                        self.cmd = (
                            -self._scale(axes[e.ABS_X], info[e.ABS_X]),
                            -self._scale(axes[e.ABS_RY], info[e.ABS_RY]),
                        )
                        dirty = False

            except OSError:
                self.get_logger().error("Device disconnected. Reconnecting...")
            finally:
                self.cmd = (0.0, 0.0)
                self.battery_path = None


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
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
