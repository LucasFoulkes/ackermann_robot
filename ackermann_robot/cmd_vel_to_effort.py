#!/usr/bin/env python3
"""cmd_vel -> [steer, throttle] on /ackermann/cmd_effort for ackermann_driver.

Pipeline:
  1. Bicycle model (wheelbase from URDF): v, w -> logical steer + logical throttle
  2. Optional PI trims from /odom twist (feedforward stays dominant, PI only
     removes steady-state error):
       - speed loop (closed_loop):     linear.x  error -> throttle trim
       - yaw  loop (yaw_closed_loop):  angular.z error -> steering trim
     The yaw loop is the only steering feedback we have (no steering-angle
     sensor): measured yaw rate + measured speed give an estimated effective
     steering angle, and the PI cancels linkage slop / asymmetric deadband.
     Deliberately NO D term anywhere: odom twist (rf2o + IMU) is too noisy to
     differentiate, and a jittery steering loop is dangerous.
  3. logical_to_raw(): compensate motor/servo deadband (tune in cmd_vel_to_effort.yaml)
  4. Publish Float32MultiArray [data[0]=steer, data[1]=throttle]

Measurements (robot.urdf.xacro):
  wheelbase = 0.2775 m  (rear axle to front axle)
  max_steer_angle = seed only — refine with field tests

max_speed: cmd_vel linear.x at this value -> logical throttle 1.0 -> raw ~1.0 after
deadband map. Tune from /odom (first log suggested ~0.85 m/s at strong forward effort).
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

from ackermann_robot.effort_deadband import logical_to_raw


def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))


class CmdVelToEffort(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_effort")
        p = lambda n, d: self.declare_parameter(n, d).value

        self.L = float(p("wheelbase", 0.2775))
        self.max_speed = max(0.05, float(p("max_speed", 0.85)))
        # Reverse drivetrain is far hotter than forward: 2026-06-11 logs show
        # cmd -0.2 m/s -> odom -0.8 m/s with a symmetric mapping (4x overshoot,
        # every reverse leg of a 3-point turn overshoots its cusp). Separate
        # scale keeps logical reverse throttle proportional to real speed.
        self.max_speed_rev = max(0.05, float(p("max_speed_rev", 3.4)))
        self.max_steer = max(0.05, float(p("max_steer_angle", math.radians(15.0))))
        # Bicycle model: |omega| <= |v| * kappa_max, kappa_max = tan(delta_max) / L.
        self.kappa_max = math.tan(self.max_steer) / self.L
        self.v_eps = float(p("v_epsilon", 0.02))
        self.timeout_ns = int(float(p("cmd_timeout_s", 0.5)) * 1e9)
        hz = max(1.0, float(p("control_hz", 50.0)))
        self.dt = 1.0 / hz

        self.apply_deadband = bool(p("apply_deadband", True))
        self.thr_dead_fwd = float(p("throttle_deadband_fwd", 0.45))
        self.thr_dead_rev = float(p("throttle_deadband_rev", 0.55))
        self.steer_dead_pos = float(p("steer_deadband_pos", 0.55))
        self.steer_dead_neg = float(p("steer_deadband_neg", 0.60))

        # --- speed PI (throttle trim from odom linear.x) ---
        self.closed_loop = bool(p("closed_loop", False))
        self.kp = float(p("vel_kp", 0.3))
        self.ki = float(p("vel_ki", 0.3))
        self.trim_max = float(p("vel_trim_max", 0.2))
        self.odom_timeout_ns = int(float(p("odom_timeout_s", 0.5)) * 1e9)

        # --- yaw-rate PI (steering trim from odom angular.z, only while moving) ---
        self.yaw_closed_loop = bool(p("yaw_closed_loop", False))
        self.yaw_kp = float(p("yaw_kp", 0.4))
        self.yaw_ki = float(p("yaw_ki", 0.3))
        self.yaw_trim_max = float(p("yaw_trim_max", 0.25))
        self.yaw_int_max = float(p("yaw_int_max", 0.3))
        # Below this |v| yaw rate carries no steering information (omega = v*tan(delta)/L).
        self.yaw_min_speed = float(p("yaw_min_speed", 0.10))

        # --- direction-change interlock ---
        # The drivetrain needs ~1.3 s to actually reverse direction (deadband +
        # ESC + momentum). Commanding the new direction immediately puts the
        # steering sign out of phase with the real motion — curvature comes out
        # mirrored and 3-point turns thrash (2026-06-11 logs: odom_v kept the
        # old sign >1 s after every cmd_v flip). Hold neutral until nearly stopped.
        self.interlock = bool(p("direction_interlock", True))
        self.interlock_v_stop = float(p("interlock_v_stop", 0.05))
        # ICP odometry over-reports linear speed during rotation (2026-06-13:
        # odom_v ~0.45 turning vs ~0.14 straight on a 0.20 cmd). That phantom
        # speed made the speed PI cut throttle to 0 mid-turn -> robot stalls in
        # corners and drifts during the steering lag. Above this yaw rate, hold
        # the speed trim and run feedforward only (cmd->throttle), which is right.
        self.speed_yaw_gate = float(p("speed_trim_yaw_gate", 0.15))

        # --- stiction kick ---
        # Breakaway throttle is higher than keep-rolling throttle, and the
        # reverse window is razor thin (raw -0.58 sometimes never moves, -0.66
        # was the 0.8 m/s runaway — 2026-06-11 logs: 37% of reverse time stalled,
        # one 19 s stall). When commanded to move but odom shows no motion for
        # kick_delay_s, ramp extra throttle until the robot breaks free, then
        # drop it immediately so it doesn't become a runaway.
        self.kick_enabled = bool(p("stiction_kick", True))
        self.kick_delay_s = float(p("kick_delay_s", 0.3))
        self.kick_rate = float(p("kick_rate", 0.4))      # logical effort per second
        self.kick_max = float(p("kick_max", 0.35))       # logical effort cap
        self.kick_v_moving = float(p("kick_v_moving", 0.05))  # m/s, kick ends here
        self._stall_since = None
        self._kick = 0.0

        # --- odometry-distrust watchdog ---
        # 2026-07-02 runaway: fast reverse the lidar ICP couldn't track read as
        # odom_v 0 while the robot backed away -- the stiction kick + speed trim
        # then piled effort onto a "stalled" robot that was actually
        # accelerating blind. Commanded motion with no measured motion for
        # watchdog_stall_s (or stale odom) means a hopeless stall or a blind
        # runaway; both end the same: neutral until the command releases or
        # odom shows motion again. A real stall loses nothing (the kick already
        # had its window); a blind runaway coasts to a stop.
        self.watchdog_enabled = bool(p("odom_watchdog", True))
        self.watchdog_stall_s = float(p("watchdog_stall_s", 2.0))
        self.watchdog_v_moving = float(p("watchdog_v_moving", 0.05))
        self._watch_since = None
        self._watch_tripped = False

        # --- raw output ceiling ---
        # Feedforward, trim and kick each obey their own caps but STACK: the
        # runaway run summed them to raw -0.8 on a -0.2 m/s command. No
        # combination may exceed a known-safe raw effort per direction.
        self.raw_max_fwd = float(p("raw_throttle_max_fwd", 0.90))
        self.raw_max_rev = float(p("raw_throttle_max_rev", 0.70))

        # --- steering hysteresis compensation ---
        # The loaded linkage is hysteretic (servo saver + tire bore friction):
        # wheel angle depends on motion history, not just the command —
        # 2026-06-12 logs: straight-line curvature scattered BOTH ways, lifted
        # robot centers fine. Backlash inverse overdrives by half the slop
        # width in the direction of motion; dither keeps the linkage unstuck.
        self.steer_backlash = float(p("steer_backlash", 0.12))   # effort units of slop
        self.steer_dither_amp = float(p("steer_dither_amp", 0.02))
        self.steer_dither_hz = float(p("steer_dither_hz", 4.0))
        self._steer_prev_des = 0.0
        self._steer_dir = 0.0

        self.publish_debug = bool(p("publish_debug", False))

        self.cmd = (0.0, 0.0)
        self.last_cmd_ns = self.get_clock().now().nanoseconds
        self.v_meas = 0.0
        self.w_meas = 0.0
        self.last_odom_ns = 0
        self.integral = 0.0
        self.yaw_integral = 0.0

        self.create_subscription(Twist, p("cmd_topic", "/cmd_vel"), self._on_cmd, 10)
        self.create_subscription(
            TwistStamped, p("cmd_stamped_topic", "/cmd_vel_nav"),
            lambda m: self._on_cmd(m.twist), 10)
        if (self.closed_loop or self.yaw_closed_loop or self.interlock
                or self.watchdog_enabled):
            self.create_subscription(Odometry, p("odom_topic", "/odom"), self._on_odom, 10)
        self.pub = self.create_publisher(
            Float32MultiArray, p("effort_topic", "/ackermann/cmd_effort"), 10)
        self.pub_debug = (
            self.create_publisher(Float32MultiArray, "/cmd_vel_to_effort/debug", 5)
            if self.publish_debug else None
        )
        self.create_timer(self.dt, self._tick)

        self.get_logger().info(
            f"wheelbase={self.L:.4f} m, max_speed={self.max_speed:.2f} m/s, "
            f"max_steer={math.degrees(self.max_steer):.1f} deg "
            f"(kappa_max={self.kappa_max:.2f} 1/m), deadband={self.apply_deadband}, "
            f"speed_pi={self.closed_loop} (kp={self.kp} ki={self.ki}), "
            f"yaw_pi={self.yaw_closed_loop} (kp={self.yaw_kp} ki={self.yaw_ki})")

    def _on_cmd(self, msg: Twist):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.last_cmd_ns = self.get_clock().now().nanoseconds

    def _on_odom(self, msg: Odometry):
        self.v_meas = msg.twist.twist.linear.x
        self.w_meas = msg.twist.twist.angular.z
        self.last_odom_ns = self.get_clock().now().nanoseconds

    def _raw_throttle(self, logical: float) -> float:
        if not self.apply_deadband:
            return clamp(logical)
        return logical_to_raw(logical, self.thr_dead_rev, self.thr_dead_fwd)

    def _raw_steer(self, logical: float) -> float:
        if not self.apply_deadband:
            return clamp(logical)
        return logical_to_raw(logical, self.steer_dead_neg, self.steer_dead_pos)

    def _tick(self):
        now = self.get_clock().now().nanoseconds
        if now - self.last_cmd_ns > self.timeout_ns:
            # Stale cmd: hard neutral. No trims either — the yaw PI must not
            # counter-steer a coasting robot that nobody is commanding.
            # Watchdog resets too: without this the trip LATCHED across stale
            # gaps (watch_since kept its old epoch), so every later command
            # re-tripped instantly and the robot was bricked (2026-07-02
            # pulse test: all pulses got eff 0.00).
            self.integral = 0.0
            self.yaw_integral = 0.0
            self._watch_since = None
            self._watch_tripped = False
            self.pub.publish(Float32MultiArray(data=[0.0, 0.0]))
            if self.pub_debug is not None:
                self.pub_debug.publish(Float32MultiArray(data=[
                    0.0, 0.0, float(self.v_meas), float(self.w_meas),
                    0.0, 0.0, 0.0, 0.0, 0.0, float(self.kappa_max), 0.0]))
            return
        v, w_cmd = self.cmd

        # Direction-change interlock: commanded sign opposes measured motion.
        # Neutral throttle + centered steer until the robot has nearly stopped,
        # then the new direction engages with the correct steering sign.
        odom_fresh = (now - self.last_odom_ns) <= self.odom_timeout_ns
        if (self.interlock and odom_fresh and abs(v) > self.v_eps
                and v * self.v_meas < 0.0
                and abs(self.v_meas) > self.interlock_v_stop):
            self.integral = 0.0
            self.yaw_integral = 0.0
            self.pub.publish(Float32MultiArray(data=[0.0, 0.0]))
            if self.pub_debug is not None:
                self.pub_debug.publish(Float32MultiArray(data=[
                    0.0, 0.0, float(self.v_meas), float(self.w_meas),
                    float(w_cmd), 0.0, 0.0, 0.0, 0.0,
                    float(self.kappa_max), 1.0]))
            return

        # Odometry-distrust watchdog (see __init__ note). Runs after the
        # interlock so a legitimate direction change doesn't count as a stall.
        if self.watchdog_enabled and abs(v) > self.v_eps:
            odom_alive = odom_fresh and abs(self.v_meas) >= self.watchdog_v_moving
            if odom_alive:
                self._watch_since = None
                self._watch_tripped = False
            elif self._watch_since is None:
                self._watch_since = now
            elif (now - self._watch_since) / 1e9 >= self.watchdog_stall_s:
                self._watch_tripped = True
        else:
            self._watch_since = None
            self._watch_tripped = False
        if self._watch_tripped:
            self.integral = 0.0
            self.yaw_integral = 0.0
            self._kick = 0.0
            self._stall_since = None
            self.get_logger().warning(
                f"odom watchdog: {self.watchdog_stall_s:.1f} s of commanded "
                "motion with no measured motion -- holding neutral "
                "(stall or blind odometry)", throttle_duration_sec=2.0)
            self.pub.publish(Float32MultiArray(data=[0.0, 0.0]))
            if self.pub_debug is not None:
                self.pub_debug.publish(Float32MultiArray(data=[
                    0.0, 0.0, float(self.v_meas), float(self.w_meas),
                    float(w_cmd), 0.0, 0.0, 0.0, 0.0,
                    float(self.kappa_max), 2.0]))
            return

        w_clamped = w_cmd
        w_limited = 0.0

        if abs(v) > self.v_eps:
            # Nav2 RPP has no Ackermann limit — it can command |w| >> |v|*kappa_max when
            # correcting path error. Clamp omega so steer effort matches physical full lock.
            w_max = abs(v) * self.kappa_max
            w_clamped = clamp(w_cmd, -w_max, w_max)
            if abs(w_clamped - w_cmd) > 1e-6:
                w_limited = 1.0
            steer_log = clamp(math.atan(self.L * w_clamped / v) / self.max_steer, -1.0, 1.0)
        else:
            steer_log = 0.0

        steer_trim = self._steer_trim(steer_log, v, now)
        steer_log_out = clamp(steer_log + steer_trim)
        steer_log_out = self._hysteresis_comp(steer_log_out, now)

        speed_scale = self.max_speed if v >= 0.0 else self.max_speed_rev
        throttle_log = clamp(v / speed_scale, -1.0, 1.0)
        speed_trim = self._speed_trim(v, now) if self.closed_loop else 0.0
        throttle_log = clamp(throttle_log + speed_trim)
        throttle_log = self._stiction_kick(throttle_log, v, now)
        # Trims may only modulate toward neutral, never cross it: catching a
        # reverse surge, kp*err railed the trim and FLIPPED the throttle sign —
        # the ESC drove forward mid-reverse and every 3-point-turn leg restarted
        # (2026-06-11 18:33 run, t=65: cmd -0.2, surge -0.62, eff_drive +0.53).
        if v > 0.0:
            throttle_log = max(0.0, throttle_log)
        elif v < 0.0:
            throttle_log = min(0.0, throttle_log)

        steer = self._raw_steer(steer_log_out)
        throttle = self._raw_throttle(throttle_log)
        # Per-direction ceiling on the final raw value (see __init__ note:
        # the individually-capped terms stack).
        throttle = clamp(throttle, -self.raw_max_rev, self.raw_max_fwd)
        steer_sat = 1.0 if abs(steer_log_out) >= 0.98 else 0.0

        self.pub.publish(Float32MultiArray(data=[float(steer), float(throttle)]))
        if self.pub_debug is not None:
            self.pub_debug.publish(Float32MultiArray(data=[
                float(speed_trim), float(steer_trim),
                float(self.v_meas), float(self.w_meas),
                float(w_cmd), float(w_clamped), float(steer_log_out),
                float(w_limited), float(steer_sat), float(self.kappa_max),
                0.0,  # 1.0 = interlock branch, 2.0 = watchdog branch
            ]))

    def _hysteresis_comp(self, steer, now):
        """Backlash inverse + anti-stiction dither for the loaded steering.

        Backlash inverse: the linkage has ~steer_backlash of slop; overdrive
        by half of it in the current direction of travel so the WHEELS land
        where the command says, not the servo horn. Dither: a small steering
        oscillation keeps the contact patches from settling into static
        friction (the gauge-tapping trick) so small corrections take effect.
        """
        if self.steer_backlash > 0.0:
            if steer > self._steer_prev_des + 1e-4:
                self._steer_dir = 1.0
            elif steer < self._steer_prev_des - 1e-4:
                self._steer_dir = -1.0
            self._steer_prev_des = steer
            steer = steer + 0.5 * self.steer_backlash * self._steer_dir
        if self.steer_dither_amp > 0.0:
            steer += self.steer_dither_amp * math.sin(
                2.0 * math.pi * self.steer_dither_hz * (now / 1e9))
        return clamp(steer)

    def _stiction_kick(self, throttle_log, v_cmd, now):
        """Ramp extra throttle while commanded to move but odom shows no motion.

        FORWARD ONLY: in reverse the driver's double-tap holds ~300 ms of
        tap+neutral before engaging, which reads as a stall and fired the kick
        right as the ESC bit -- the ESC then accelerates on its own (it needs
        speed shed, never added)."""
        if not self.kick_enabled or v_cmd < 0.05 \
                or (now - self.last_odom_ns) > self.odom_timeout_ns:
            self._stall_since = None
            self._kick = 0.0
            return throttle_log
        if abs(self.v_meas) >= self.kick_v_moving:
            # Rolling: drop the kick INSTANTLY. The 0.2 s bleed-off kept boosting
            # a drivetrain already past breakaway and fed the reverse surge that
            # the speed PI then overcorrected (2026-06-11 18:33 run).
            self._stall_since = None
            self._kick = 0.0
        else:
            if self._stall_since is None:
                self._stall_since = now
            elif (now - self._stall_since) / 1e9 >= self.kick_delay_s:
                self._kick = min(self.kick_max, self._kick + self.kick_rate * self.dt)
        if self._kick <= 0.0:
            return throttle_log
        return clamp(throttle_log + math.copysign(self._kick, v_cmd))

    def _speed_trim(self, v_cmd, now):
        if abs(v_cmd) < 0.05 or (now - self.last_odom_ns) > self.odom_timeout_ns:
            self.integral = 0.0
            return 0.0
        # Turning: odom_v is phantom-inflated by ICP rotation coupling, so the
        # error is garbage -> freeze the integral and run feedforward only.
        if abs(self.w_meas) > self.speed_yaw_gate:
            return 0.0
        err = v_cmd - self.v_meas
        self.integral = clamp(self.integral + err * self.dt, -0.3, 0.3)
        trim = clamp(self.kp * err + self.ki * self.integral,
                     -self.trim_max, self.trim_max)
        if v_cmd < 0.0:
            # Reverse trim is BRAKE-ONLY: the ESC accelerates at constant
            # effort (raw sweep 2026-07-02: -0.50 held 3 s went 0.2 -> 0.9
            # m/s), so the PI's job in reverse is purely to shed speed. During
            # the double-tap's ~300 ms the robot is commanded but stationary,
            # which wound the integral the wrong way and pinned trim at -0.12
            # (deeper reverse) right as the ESC engaged.
            self.integral = max(0.0, self.integral)
            trim = max(0.0, trim)
        return trim

    def _steer_trim(self, steer_log_des, v_cmd, now):
        """PI on the estimated effective steering angle from odom yaw rate.

        delta_est = atan(L * w_meas / v): with the bicycle model, measured yaw
        rate + speed imply what the front wheels are actually doing. Error is
        computed in normalized steer-effort space (same units as steer_log) so
        the gains are dimensionless. Using v in the atan keeps the sign correct
        when reversing (same steer angle gives opposite omega in reverse).
        """
        if not self.yaw_closed_loop:
            return 0.0
        if (now - self.last_odom_ns) > self.odom_timeout_ns:
            self.yaw_integral = 0.0
            return 0.0
        v_ref = self.v_meas if abs(self.v_meas) > self.v_eps else v_cmd
        if abs(v_ref) < self.yaw_min_speed:
            self.yaw_integral = 0.0
            return 0.0
        delta_est = math.atan(self.L * self.w_meas / v_ref)
        err = steer_log_des - clamp(delta_est / self.max_steer)
        self.yaw_integral = clamp(self.yaw_integral + err * self.dt,
                                  -self.yaw_int_max, self.yaw_int_max)
        return clamp(self.yaw_kp * err + self.yaw_ki * self.yaw_integral,
                     -self.yaw_trim_max, self.yaw_trim_max)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToEffort()
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
