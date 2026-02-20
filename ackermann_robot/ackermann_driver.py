#!/usr/bin/env python3
import math
from typing import Dict, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from adafruit_pca9685 import PCA9685


def n01(x) -> float:
    try:
        x = float(x)
    except Exception:
        return 0.0
    if not math.isfinite(x):
        return 0.0
    return -1.0 if x < -1.0 else 1.0 if x > 1.0 else x


def sgn(x: float, eps: float) -> int:
    eps = abs(float(eps))
    return 1 if x > eps else -1 if x < -eps else 0


def u16_from_tick12(t: int) -> int:
    t = 0 if t < 0 else 4095 if t > 4095 else int(t)
    if t == 0:
        return 0
    if t >= 4095:
        return 0xFFFF
    return t << 4


def open_i2c(bus: int):
    try:
        from adafruit_extended_bus import ExtendedI2C as I2C
        return I2C(int(bus))
    except Exception:
        import board
        return board.I2C()


class ActuatorDriver(Node):
    NORMAL, HOLD = 0, 1

    def __init__(self) -> None:
        super().__init__("actuator_driver")

        P = {p.name: p.value for p in self.declare_parameters(
            "",
            [
                # renamed default topic (less dumb)
                ("cmd_topic", "/ackermann/driver/cmd"),
                ("timeout_s", 10.0),
                ("update_hz", 50.0),

                ("i2c_bus", 1),
                ("i2c_address", 64),
                ("pwm_hz", 50.0),
                ("osc_clock_hz", 25_000_000.0),

                ("steering_channel", 12),
                ("steering_min_tick", 205),
                ("steering_center_tick", 307),
                ("steering_max_tick", 410),
                ("invert_steering", False),

                ("motor_forward_channel", 14),
                ("motor_backward_channel", 15),
                ("throttle_min_tick", 819),
                ("throttle_max_tick", 1638),
                ("invert_throttle", False),

                ("throttle_zero_epsilon", 0.02),
                ("neutral_hold_ms", 100.0),

                ("min_tick_delta", 0),
            ],
        )}

        self.cmd_topic = str(P["cmd_topic"])
        self.timeout_s = float(P["timeout_s"])
        self.period_s = 1.0 / max(1.0, float(P["update_hz"]))

        self.i2c_bus = int(P["i2c_bus"])
        self.i2c_addr = int(P["i2c_address"])
        self.pwm_hz = float(P["pwm_hz"])
        self.osc_hz = int(P["osc_clock_hz"])

        self.st_ch = int(P["steering_channel"])
        self.st_min = int(P["steering_min_tick"])
        self.st_mid = int(P["steering_center_tick"])
        self.st_max = int(P["steering_max_tick"])
        self.inv_st = bool(P["invert_steering"])

        self.fw_ch = int(P["motor_forward_channel"])
        self.rv_ch = int(P["motor_backward_channel"])
        self.th_min = int(P["throttle_min_tick"])
        self.th_max = int(P["throttle_max_tick"])
        self.inv_th = bool(P["invert_throttle"])

        self.eps = float(P["throttle_zero_epsilon"])
        self.hold_ns = int(max(0.0, float(P["neutral_hold_ms"])) * 1e6)

        self.min_tick_delta = int(P["min_tick_delta"])

        if not (0 <= self.st_min <= self.st_mid <= self.st_max <= 4095):
            raise ValueError("steering ticks invalid")
        if not (0 <= self.th_min <= self.th_max <= 4095):
            raise ValueError("throttle ticks invalid")
        if len({self.st_ch, self.fw_ch, self.rv_ch}) != 3:
            raise ValueError("channels must be distinct")

        i2c = open_i2c(self.i2c_bus)
        self.pca = PCA9685(i2c, address=self.i2c_addr, reference_clock_speed=self.osc_hz)
        self.pca.frequency = self.pwm_hz

        now_ns = self.get_clock().now().nanoseconds
        self.last_cmd_ns = now_ns
        self.target: Tuple[float, float] = (0.0, 0.0)

        self.state = self.NORMAL
        self.pre_sign = 0
        self.hold_end_ns = now_ns
        self.applied_thr = 0.0

        self.last_written: Dict[int, int] = {}

        self.create_subscription(Float32MultiArray, self.cmd_topic, self.on_cmd, 10)
        self.create_timer(self.period_s, self.update)

        self.write_outputs(0.0, 0.0)  # startup safe

    def shutdown(self) -> None:
        try:
            self.write_outputs(0.0, 0.0)
        finally:
            try:
                self.pca.deinit()
            except Exception:
                pass

    def on_cmd(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 2:
            return
        self.target = (n01(msg.data[0]), n01(msg.data[1]))
        self.last_cmd_ns = self.get_clock().now().nanoseconds

    def update(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if (now_ns - self.last_cmd_ns) * 1e-9 > self.timeout_s:
            steer_t, thr_t = 0.0, 0.0
            self.state = self.NORMAL
            self.pre_sign = 0
            self.hold_end_ns = now_ns
            self.applied_thr = 0.0
        else:
            steer_t, thr_t = self.target

        if self.inv_st:
            steer_t = -steer_t
        if self.inv_th:
            thr_t = -thr_t

        self.write_outputs(steer_t, self.throttle_hold(now_ns, thr_t))

    def throttle_hold(self, now_ns: int, target: float) -> float:
        ts = sgn(target, self.eps)
        as_ = sgn(self.applied_thr, self.eps)

        if self.state == self.NORMAL:
            if as_ and ts and ts != as_:
                self.state = self.HOLD
                self.pre_sign = as_
                self.hold_end_ns = now_ns + self.hold_ns
                self.applied_thr = 0.0
                return 0.0
            self.applied_thr = target
            return target

        if ts == 0 or ts == self.pre_sign:
            self.state = self.NORMAL
            self.applied_thr = target
            return target

        if now_ns < self.hold_end_ns:
            self.applied_thr = 0.0
            return 0.0

        self.state = self.NORMAL
        self.applied_thr = target
        return target

    def steer_tick(self, steer: float) -> int:
        s = n01(steer)
        if s >= 0.0:
            t = self.st_mid + s * (self.st_max - self.st_mid)
        else:
            t = self.st_mid + s * (self.st_mid - self.st_min)
        t = int(round(t))
        return self.st_min if t < self.st_min else self.st_max if t > self.st_max else t

    def throttle_ticks(self, thr: float) -> Tuple[int, int]:
        t = n01(thr)
        s = sgn(t, self.eps)
        if s == 0:
            return 0, 0
        duty = int(round(self.th_min + abs(t) * (self.th_max - self.th_min)))
        duty = 0 if duty < 0 else 4095 if duty > 4095 else duty
        return (duty, 0) if s > 0 else (0, duty)

    def write_tick(self, ch: int, tick12: int) -> None:
        tick12 = 0 if tick12 < 0 else 4095 if tick12 > 4095 else int(tick12)
        prev = self.last_written.get(ch)

        if prev is not None and tick12 == prev:
            return
        if (
            self.min_tick_delta > 0 and prev is not None
            and prev != 0 and tick12 != 0
            and abs(tick12 - prev) < self.min_tick_delta
        ):
            return

        self.pca.channels[ch].duty_cycle = u16_from_tick12(tick12)
        self.last_written[ch] = tick12

    def write_outputs(self, steer: float, thr: float) -> None:
        st = self.steer_tick(steer)
        fw, rv = self.throttle_ticks(thr)
        if fw and rv:
            fw = rv = 0

        try:
            self.write_tick(self.st_ch, st)
            self.write_tick(self.fw_ch, fw)
            self.write_tick(self.rv_ch, rv)
        except Exception as e:
            self.get_logger().error(f"PWM write failed: {e}")


def main(args=None) -> None:
    rclpy.init(args=args)
    n = ActuatorDriver()
    try:
        rclpy.spin(n)
    finally:
        try:
            n.shutdown()
        except Exception:
            pass
        n.destroy_node()
        rclpy.shutdown()
