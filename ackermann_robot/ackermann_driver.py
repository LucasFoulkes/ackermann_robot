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

        # Drive motor: single servo-style ESC channel (was a dual-channel
        # H-bridge). Neutral 1500 us = stopped, >neutral forward, <neutral
        # reverse — linear map -1..1 -> neutral +/- half, same shape as steering.
        self.esc_ch = self.param("esc_channel", 14)
        self.esc_inverted = bool(self.param("esc_inverted", False))
        esc_min = self.param("esc_min_tick", 205)
        esc_max = self.param("esc_max_tick", 410)
        self.esc_neutral = self.param("esc_neutral_tick", 307)
        self.esc_half_range = min(
            self.esc_neutral - esc_min,
            esc_max - self.esc_neutral,
        )

        # ESC brake/reverse state machine: coming from forward (and often from
        # plain neutral), the ESC reads the first reverse pulse as BRAKE;
        # reverse only engages after brake -> neutral -> reverse (the
        # "double-tap"). Without it, engagement was a coin flip — the
        # 2026-07-02 pulse test sent the same ~-0.6 effort six times and got
        # 0.3 m/s three times and nothing three times. Tap at the commanded
        # magnitude, hold neutral for the gap, then stream reverse. Re-arm
        # from scratch whenever the output leaves reverse for rearm_reset_ms.
        self.double_tap = bool(self.param("esc_double_tap", True))
        self.tap_ns = int(max(0.0, self.param("esc_tap_ms", 150.0)) * 1e6)
        self.tap_gap_ns = int(max(0.0, self.param("esc_tap_gap_ms", 150.0)) * 1e6)
        self.rearm_reset_ns = int(max(0.0, self.param("esc_rearm_reset_ms", 500.0)) * 1e6)
        self._rev_armed = False
        self._rev_phase = None      # None | ["tap"|"gap", end_ns]
        self._last_rev_out_ns = 0

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

        # ESC: hold neutral through a direction flip until the opposite side has
        # idled reverse_delay_ns, so a forward/brake/reverse ESC crosses neutral
        # cleanly (the brake/reverse double-tap) instead of braking when we want
        # reverse. esc_throttle stays 0 (neutral) while blocked or commanded 0.
        esc_throttle = 0.0
        if throttle > 0.0 and now >= self.last_reverse_ns + self.reverse_delay_ns:
            self.last_forward_ns = now
            esc_throttle = throttle
            self._rev_armed = False
            self._rev_phase = None
        elif throttle < 0.0 and now >= self.last_forward_ns + self.reverse_delay_ns:
            self.last_reverse_ns = now
            esc_throttle = self._reverse_throttle(throttle, now)
        if self.esc_inverted:
            esc_throttle = -esc_throttle
        esc_tick = round(self.esc_neutral + esc_throttle * self.esc_half_range)

        try:
            self._set(self.steering_ch, steering_tick)
            self._set(self.esc_ch, esc_tick)
        except OSError as e:
            self.get_logger().error(f"I2C write failed: {e}", throttle_duration_sec=1.0)

    def _reverse_throttle(self, throttle, now):
        """Double-tap gate for reverse (see the __init__ note)."""
        if not self.double_tap:
            return throttle
        if self._rev_armed and now - self._last_rev_out_ns > self.rearm_reset_ns:
            # Output left reverse long enough for the ESC to forget its mode.
            self._rev_armed = False
        if not self._rev_armed:
            if self._rev_phase is None:
                self._rev_phase = ["tap", now + self.tap_ns]
            phase, end_ns = self._rev_phase
            if phase == "tap":
                if now < end_ns:
                    return throttle          # read as brake by the ESC
                self._rev_phase = ["gap", now + self.tap_gap_ns]
                return 0.0
            if now < end_ns:                 # neutral gap
                return 0.0
            self._rev_armed = True
            self._rev_phase = None
        self._last_rev_out_ns = now
        return throttle

    def _set(self, ch: int, tick):
        tick = max(0, min(4095, int(tick)))
        self.pca.channels[ch].duty_cycle = tick << 4

    def destroy_node(self):
        try:
            self._set(self.esc_ch, self.esc_neutral)  # ESC -> stopped, not a 0us disarm glitch
            self._set(self.steering_ch, 0)
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
