#!/usr/bin/env python3
"""PCA9685 actuator driver: Float32MultiArray [steer, throttle] in -1..1 -> servo + motor PWM."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray
from adafruit_pca9685 import PCA9685


def open_i2c(bus: int):
    try:
        from adafruit_extended_bus import ExtendedI2C
        return ExtendedI2C(bus)
    except ImportError:
        import board
        return board.I2C()


def clamp(x) -> float:
    """Fast filter for NaNs, type-safety, and out-of-bounds inputs."""
    try:
        x = float(x)
        return max(-1.0, min(1.0, x)) if math.isfinite(x) else 0.0
    except (TypeError, ValueError):
        return 0.0


class ActuatorDriver(Node):
    MAX_PWM = 4095

    def __init__(self):
        super().__init__("actuator_driver")

        p = lambda n, d: self.declare_parameter(n, d).value

        # Timings pre-computed to nanoseconds for zero-cost loop comparisons
        self.timeout_ns = int(p("timeout_s", 10.0) * 1e9)
        self.hold_ns = int(max(0.0, p("neutral_hold_ms", 100.0)) * 1e6)
        self.eps = float(p("throttle_zero_epsilon", 0.02))
        self.min_delta = int(p("min_tick_delta", 0))

        # Hardware Setup
        self.pca = PCA9685(
            open_i2c(p("i2c_bus", 1)), address=p("i2c_address", 64),
            reference_clock_speed=p("osc_clock_hz", 25_000_000),
        )
        self.pca.frequency = p("pwm_hz", 50.0)

        # Steering servo
        self.st_ch = p("steering_channel", 12)
        self.st_cal = (p("steering_min_tick", 205), p("steering_center_tick", 307), p("steering_max_tick", 410))
        self.st_dir = -1.0 if p("invert_steering", False) else 1.0

        # Motor
        self.fw_ch = p("motor_forward_channel", 14)
        self.rv_ch = p("motor_backward_channel", 15)
        self.th_cal = (p("throttle_min_tick", 819), p("throttle_max_tick", 1638))
        self.th_dir = -1.0 if p("invert_throttle", False) else 1.0

        # Validate calibration
        s_min, s_mid, s_max = self.st_cal
        t_min, t_max = self.th_cal
        if not (0 <= s_min <= s_mid <= s_max <= self.MAX_PWM):
            raise ValueError("steering ticks out of range")
        if not (0 <= t_min <= t_max <= self.MAX_PWM):
            raise ValueError("throttle ticks out of range")
        if len({self.st_ch, self.fw_ch, self.rv_ch}) != 3:
            raise ValueError("PCA9685 channels must be distinct")

        # State
        now = self.get_clock().now().nanoseconds
        self.last_active_ns = {1: now - self.hold_ns, -1: now - self.hold_ns}
        self.last_cmd_ns = now
        self.cmd = (0.0, 0.0)
        self._prev = {}

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Float32MultiArray, str(p("cmd_topic", "/ackermann/cmd_effort")), self._on_cmd, qos)
        self.create_timer(1.0 / max(1.0, float(p("update_hz", 50.0))), self._tick)

    # -- ROS Callbacks --------------------------------------------------------

    def _on_cmd(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            self.cmd = (clamp(msg.data[0]), clamp(msg.data[1]))
            self.last_cmd_ns = self.get_clock().now().nanoseconds

    def _tick(self):
        now = self.get_clock().now().nanoseconds

        st, th = self.cmd if (now - self.last_cmd_ns) <= self.timeout_ns else (0.0, 0.0)

        st *= self.st_dir
        th *= self.th_dir

        # Direction vector: 1 (FW), -1 (RV), 0 (Neutral)
        ts = 1 if th > self.eps else -1 if th < -self.eps else 0

        if ts != 0:
            if now < self.last_active_ns[-ts] + self.hold_ns:
                th, ts = 0.0, 0
            else:
                self.last_active_ns[ts] = now

        # Steering interpolation (asymmetric around center, clamped to calibration range)
        s_min, s_mid, s_max = self.st_cal
        st_tick = max(s_min, min(s_max, int(round(
            s_mid + st * (s_max - s_mid if st >= 0 else s_mid - s_min)
        ))))

        # Throttle interpolation
        t_min, t_max = self.th_cal
        duty = int(round(t_min + abs(th) * (t_max - t_min))) if ts != 0 else 0
        fw, rv = (duty, 0) if ts == 1 else (0, duty) if ts == -1 else (0, 0)

        try:
            self._set_channel(self.st_ch, st_tick)
            self._set_channel(self.fw_ch, fw)
            self._set_channel(self.rv_ch, rv)
        except OSError as e:
            self.get_logger().error(f"I2C Write Failed: {e}", throttle_duration_sec=1.0)

    # -- Hardware Output ------------------------------------------------------

    def _set_channel(self, ch: int, tick: int):
        tick = max(0, min(self.MAX_PWM, tick))
        prev = self._prev.get(ch, -1)

        if prev == tick or (self.min_delta and prev > 0 and tick > 0 and abs(tick - prev) < self.min_delta):
            return

        self.pca.channels[ch].duty_cycle = 0xFFFF if tick == self.MAX_PWM else (tick << 4) if tick else 0
        self._prev[ch] = tick

    def destroy_node(self):
        try:
            for ch in (self.st_ch, self.fw_ch, self.rv_ch):
                self._set_channel(ch, 0)
            self.pca.deinit()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
