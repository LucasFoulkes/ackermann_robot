from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class PCA9685:
    # Minimal PCA9685 driver over I2C (SMBus). No external Adafruit deps.

    _MODE1 = 0x00
    _MODE2 = 0x01
    _PRESCALE = 0xFE
    _LED0_ON_L = 0x06

    def __init__(
        self,
        *,
        bus: int,
        address: int,
        osc_clock_hz: float = 25_000_000.0,
        pwm_hz: float = 50.0,
    ) -> None:
        self._address = address
        self._osc_clock_hz = float(osc_clock_hz)

        try:
            try:
                from smbus import SMBus  # type: ignore
            except Exception:
                from smbus2 import SMBus  # type: ignore
        except Exception as e:  # pragma: no cover
            raise RuntimeError(
                "SMBus is required to talk to PCA9685. Install with: sudo apt install python3-smbus"
            ) from e

        self._SMBus = SMBus
        self._bus = self._SMBus(bus)

        # Reset MODE1 (sleep=0, auto-inc off)
        self._write8(self._MODE1, 0x00)
        time.sleep(0.01)

        # MODE2: totem pole (OUTDRV)
        self._write8(self._MODE2, 0x04)
        time.sleep(0.01)

        self.set_pwm_freq(pwm_hz)

    def close(self) -> None:
        try:
            self._bus.close()
        except Exception:
            pass

    def _write8(self, reg: int, value: int) -> None:
        self._bus.write_byte_data(self._address, reg, value & 0xFF)

    def _read8(self, reg: int) -> int:
        return int(self._bus.read_byte_data(self._address, reg))

    def set_pwm_freq(self, freq_hz: float) -> None:
        freq_hz = float(freq_hz)
        if freq_hz <= 0.0:
            raise ValueError("pwm_hz must be > 0")

        prescaleval = (self._osc_clock_hz / (4096.0 * freq_hz)) - 1.0
        prescale = int(round(prescaleval))
        prescale = max(3, min(255, prescale))

        oldmode = self._read8(self._MODE1)
        sleepmode = (oldmode & 0x7F) | 0x10  # sleep
        self._write8(self._MODE1, sleepmode)
        self._write8(self._PRESCALE, prescale)
        self._write8(self._MODE1, oldmode)
        time.sleep(0.005)
        self._write8(self._MODE1, oldmode | 0xA1)  # restart + auto-inc

    def set_pwm(self, channel: int, on: int, off: int) -> None:
        if not (0 <= channel <= 15):
            raise ValueError("channel must be 0..15")
        if not (0 <= on <= 4095 and 0 <= off <= 4095):
            raise ValueError("on/off must be 0..4095")

        base = self._LED0_ON_L + 4 * channel
        data = [on & 0xFF, (on >> 8) & 0x0F, off & 0xFF, (off >> 8) & 0x0F]
        self._bus.write_i2c_block_data(self._address, base, data)

    def set_duty(self, channel: int, duty: int) -> None:
        duty = int(duty)
        duty = max(0, min(4095, duty))

        if duty == 0:
            # Full off
            self.set_pwm(channel, 0, 0)
            return
        if duty >= 4095:
            # Full on
            self.set_pwm(channel, 4095, 0)
            return

        self.set_pwm(channel, 0, duty)


@dataclass
class AxisMapping:
    deadband: float
    cap: float


@dataclass
class SteeringPWM:
    channel: int
    min_tick: int
    center_tick: int
    max_tick: int
    invert: bool


@dataclass
class ThrottlePWM:
    fwd_channel: int
    rev_channel: int
    min_tick: int
    max_tick: int
    invert: bool


def _apply_deadband(x: float, deadband: float) -> float:
    if abs(x) < deadband:
        return 0.0
    return x


def _norm_with_cap(x: float, cap: float) -> float:
    # Returns normalized magnitude 0..1 using cap threshold.
    cap = float(cap)
    if cap <= 0.0:
        return 0.0
    return min(1.0, abs(x) / cap)


class AckermannDriver(Node):
    def __init__(self) -> None:
        super().__init__("ackermann_driver")

        # I2C / PCA9685 params
        self.declare_parameter("pwm_hz", 50.0)
        self.declare_parameter("osc_clock_hz", 25_000_000.0)

        # Topic / timing
        self.declare_parameter("cmd_topic", "/ackermann/cmd")
        # self.declare_parameter("timeout_s", 0.5)
        self.declare_parameter("timeout_s", 30.0)
        self.declare_parameter("update_hz", 50.0)

        # Axis mappings (input is -100..100)
        self.declare_parameter("steering_deadband", 2.0)
        self.declare_parameter("steering_cap", 50.0)
        self.declare_parameter("throttle_deadband", 2.0)
        self.declare_parameter("throttle_cap", 100.0)

        # Output channels
        self.declare_parameter("steering_channel", 12)
        self.declare_parameter("motor_forward_channel", 14)
        self.declare_parameter("motor_backward_channel", 15)

        # Steering PWM range in PCA ticks (0..4095). Defaults match 1.0ms/1.5ms/2.0ms at 50Hz.
        self.declare_parameter("steering_min", 205)
        self.declare_parameter("steering_center", 307)
        self.declare_parameter("steering_max", 410)
        self.declare_parameter("invert_steering", False)

        # Throttle PWM range in PCA ticks (0..4095).
        # User requested min=20% (819) and max=40% (1638) of 4095.
        self.declare_parameter("throttle_min", 819)
        self.declare_parameter("throttle_max", 1638)
        self.declare_parameter("invert_throttle", False)

        # Safety / convenience
        self.declare_parameter("direction_change_brake_ms", 50.0)

        self._cmd_topic = str(self.get_parameter("cmd_topic").value)
        self._timeout_s = float(self.get_parameter("timeout_s").value)
        update_hz = float(self.get_parameter("update_hz").value)
        self._period_s = 1.0 / max(1.0, update_hz)

        self._steer_map = AxisMapping(
            deadband=float(self.get_parameter("steering_deadband").value),
            cap=float(self.get_parameter("steering_cap").value),
        )
        self._throttle_map = AxisMapping(
            deadband=float(self.get_parameter("throttle_deadband").value),
            cap=float(self.get_parameter("throttle_cap").value),
        )

        self._steering_pwm = SteeringPWM(
            channel=int(self.get_parameter("steering_channel").value),
            min_tick=int(self.get_parameter("steering_min").value),
            center_tick=int(self.get_parameter("steering_center").value),
            max_tick=int(self.get_parameter("steering_max").value),
            invert=bool(self.get_parameter("invert_steering").value),
        )
        self._throttle_pwm = ThrottlePWM(
            fwd_channel=int(self.get_parameter("motor_forward_channel").value),
            rev_channel=int(self.get_parameter("motor_backward_channel").value),
            min_tick=int(self.get_parameter("throttle_min").value),
            max_tick=int(self.get_parameter("throttle_max").value),
            invert=bool(self.get_parameter("invert_throttle").value),
        )

        self._direction_change_brake_ms = float(self.get_parameter("direction_change_brake_ms").value)

        try:
            self._pca: Optional[PCA9685] = PCA9685(
                bus=1,
                address=0x40,
                osc_clock_hz=float(self.get_parameter("osc_clock_hz").value),
                pwm_hz=float(self.get_parameter("pwm_hz").value),
            )
            self.get_logger().info(
                "PCA9685 ready on i2c-1 addr=0x40"
            )
        except Exception as e:
            raise RuntimeError(
                "Failed to init PCA9685 (HW-170). Check: I2C enabled, wiring OK, address/bus correct, and install SMBus support: sudo apt install python3-smbus i2c-tools"
            ) from e

        self._last_cmd_time = self.get_clock().now()
        self._last_cmd: Tuple[float, float] = (0.0, 0.0)
        self._last_throttle_sign: int = 0

        self._sub = self.create_subscription(Float32MultiArray, self._cmd_topic, self._on_cmd, 10)
        self._timer = self.create_timer(self._period_s, self._update_outputs)

        # Ensure safe outputs on startup
        self._write_outputs(steering_cmd=0.0, throttle_cmd=0.0, reason="startup")

        self.get_logger().info(
            f"Subscribed to {self._cmd_topic} as std_msgs/Float32MultiArray [steering, throttle] in [-100,100]"
        )

    def _on_shutdown(self) -> None:
        self._write_outputs(steering_cmd=0.0, throttle_cmd=0.0, reason="shutdown")
        if self._pca is not None:
            self._pca.close()

    def _on_cmd(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 2:
            self.get_logger().warn("/ackermann/cmd needs 2 floats: [steering, throttle]")
            return
        steering = float(msg.data[0])
        throttle = float(msg.data[1])
        self._last_cmd = (steering, throttle)
        self._last_cmd_time = self.get_clock().now()

    def _update_outputs(self) -> None:
        now = self.get_clock().now()
        age = (now - self._last_cmd_time).nanoseconds * 1e-9

        if age > self._timeout_s:
            steering_cmd, throttle_cmd = 0.0, 0.0
            reason = f"timeout ({age:.3f}s)"
        else:
            steering_cmd, throttle_cmd = self._last_cmd
            reason = "cmd"

        self._write_outputs(steering_cmd=steering_cmd, throttle_cmd=throttle_cmd, reason=reason)

    def _write_outputs(self, *, steering_cmd: float, throttle_cmd: float, reason: str) -> None:
        steering_cmd = max(-100.0, min(100.0, float(steering_cmd)))
        throttle_cmd = max(-100.0, min(100.0, float(throttle_cmd)))

        steering_cmd = _apply_deadband(steering_cmd, self._steer_map.deadband)
        throttle_cmd = _apply_deadband(throttle_cmd, self._throttle_map.deadband)

        if self._steering_pwm.invert:
            steering_cmd *= -1.0
        if self._throttle_pwm.invert:
            throttle_cmd *= -1.0

        # Steering mapping: centered PWM
        steer_norm = _norm_with_cap(steering_cmd, self._steer_map.cap)
        if steering_cmd >= 0.0:
            steering_tick = int(
                round(
                    self._steering_pwm.center_tick
                    + steer_norm * (self._steering_pwm.max_tick - self._steering_pwm.center_tick)
                )
            )
        else:
            steering_tick = int(
                round(
                    self._steering_pwm.center_tick
                    - steer_norm * (self._steering_pwm.center_tick - self._steering_pwm.min_tick)
                )
            )
        steering_tick = max(self._steering_pwm.min_tick, min(self._steering_pwm.max_tick, steering_tick))

        # Throttle mapping: split forward/back
        if throttle_cmd == 0.0:
            fwd_tick = 0
            rev_tick = 0
            throttle_sign = 0
        else:
            throttle_norm = _norm_with_cap(throttle_cmd, self._throttle_map.cap)
            duty = int(round(self._throttle_pwm.min_tick + throttle_norm * (self._throttle_pwm.max_tick - self._throttle_pwm.min_tick)))
            duty = max(0, min(4095, duty))
            throttle_sign = 1 if throttle_cmd > 0.0 else -1
            fwd_tick = duty if throttle_sign > 0 else 0
            rev_tick = duty if throttle_sign < 0 else 0

        # Optional brake when switching direction
        if throttle_sign != 0 and self._last_throttle_sign != 0 and throttle_sign != self._last_throttle_sign:
            self._set_channel(self._throttle_pwm.fwd_channel, 0)
            self._set_channel(self._throttle_pwm.rev_channel, 0)
            time.sleep(max(0.0, self._direction_change_brake_ms) / 1000.0)

        self._last_throttle_sign = throttle_sign

        self._set_channel(self._steering_pwm.channel, steering_tick)
        self._set_channel(self._throttle_pwm.fwd_channel, fwd_tick)
        self._set_channel(self._throttle_pwm.rev_channel, rev_tick)

    def _set_channel(self, channel: int, duty: int) -> None:
        if self._pca is None:
            raise RuntimeError("PCA9685 not initialized")
        try:
            self._pca.set_duty(int(channel), int(duty))
        except Exception as e:
            self.get_logger().error(f"Failed to set PCA9685 channel {channel} duty={duty}: {e}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AckermannDriver()
    try:
        rclpy.spin(node)
    finally:
        try:
            node._on_shutdown()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
