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
    """Filter NaNs, bad types, and out-of-bounds inputs to a safe -1..1."""
    try:
        x = float(x)
        return max(-1.0, min(1.0, x)) if math.isfinite(x) else 0.0
    except (TypeError, ValueError):
        return 0.0


class ActuatorDriver(Node):
    def __init__(self):
        super().__init__("actuator_driver")

        self.timeout_ns = int(self.param("timeout_s", 0.5) * 1e9)
        self.reverse_delay_ns = int(max(0.0, self.param("neutral_hold_ms", 100.0)) * 1e6)

        # Hardware
        self.pca = PCA9685(
            open_i2c(self.param("i2c_bus", 1)),
            address=self.param("i2c_address", 64),
            reference_clock_speed=self.param("osc_clock_hz", 25_000_000),
        )
        self.pca.frequency = self.param("pwm_hz", 50.0)

        # Steering servo: linear map -1..1 -> center +/- half
        self.steering_ch = self.param("steering_channel", 15)
        self.steering_inverted = bool(self.param("steering_inverted", False))
        steering_min = self.param("steering_min_tick", 205)
        steering_max = self.param("steering_max_tick", 410)
        self.steering_center = self.param("steering_center_tick", 307)
        self.steering_half_range = min(
            self.steering_center - steering_min,
            steering_max - self.steering_center,
        )

        # Drive motor: dual-channel H-bridge (forward / backward)
        self.forward_ch = self.param("motor_forward_channel", 1)
        self.reverse_ch = self.param("motor_backward_channel", 2)
        self.throttle_max = self.param("throttle_max_tick", 1638)

        self.cmd = (0.0, 0.0)
        now = self.get_clock().now().nanoseconds
        self.last_cmd_ns = now
        self.last_forward_ns = now - self.reverse_delay_ns
        self.last_reverse_ns = now - self.reverse_delay_ns

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(
            Float32MultiArray,
            str(self.param("cmd_topic", "/ackermann/cmd_effort")),
            self._on_cmd,
            qos,
        )
        update_hz = max(1.0, float(self.param("update_hz", 50.0)))
        self.create_timer(1.0 / update_hz, self._tick)

    def param(self, name: str, default):
        return self.declare_parameter(name, default).value

    def _on_cmd(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            self.cmd = (clamp(msg.data[0]), clamp(msg.data[1]))
            self.last_cmd_ns = self.get_clock().now().nanoseconds

    def _tick(self):
        now = self.get_clock().now().nanoseconds

        # Watchdog: zero everything if commands have gone stale.
        if now - self.last_cmd_ns > self.timeout_ns:
            steering, throttle = 0.0, 0.0
        else:
            steering, throttle = self.cmd
        if self.steering_inverted:
            steering = -steering

        steering_tick = round(self.steering_center + steering * self.steering_half_range)

        # Motor: block a direction flip until the opposite side has idled reverse_delay_ns.
        forward = reverse = 0
        if throttle > 0.0 and now >= self.last_reverse_ns + self.reverse_delay_ns:
            self.last_forward_ns = now
            forward = round(throttle * self.throttle_max)
        elif throttle < 0.0 and now >= self.last_forward_ns + self.reverse_delay_ns:
            self.last_reverse_ns = now
            reverse = round(-throttle * self.throttle_max)

        try:
            self._set(self.steering_ch, steering_tick)
            self._set(self.forward_ch, forward)
            self._set(self.reverse_ch, reverse)
        except OSError as e:
            self.get_logger().error(f"I2C write failed: {e}", throttle_duration_sec=1.0)

    def _set(self, ch: int, tick):
        tick = max(0, min(4095, int(tick)))
        self.pca.channels[ch].duty_cycle = tick << 4

    def destroy_node(self):
        try:
            for ch in (self.steering_ch, self.forward_ch, self.reverse_ch):
                self._set(ch, 0)
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
