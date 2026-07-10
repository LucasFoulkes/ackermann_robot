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
from sensor_msgs.msg import Imu
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
        self.interlock_max_hold_s = float(p("interlock_max_hold_s", 1.8))
        # Interlock only engages within this window after a commanded
        # direction flip — outside it, "opposing motion" is a lying odometer
        # (reverse ICP phantoms), not a real coast. See the _tick note.
        self.interlock_window_ns = int(p("interlock_window_s", 2.5) * 1e9)
        self._interlock_since = None
        self._flip_ns = 0
        # --- cusp dwell (steer-settle, 2026-07-04 masterplan T0) ---
        # Reeds-Shepp assumes the wheels can re-aim while stopped (nav2 #4425
        # maintainer guidance), but our servo-saver needs ~1 s under load. On a
        # commanded direction flip: throttle stays neutral for steer_settle_s
        # while the NEW steering is already published, so the wheels are aimed
        # when the leg starts. 0 disables.
        self.steer_settle_s = float(p("steer_settle_s", 0.8))
        self._settle_until_ns = 0
        self._drive_sign = 0.0
        # Pulse-and-coast crawl for sub-floor forward speeds (see _tick note).
        self.slow_pulse = bool(p("slow_pulse", True))
        self.slow_floor_v = float(p("slow_floor_v", 0.30))
        self.slow_period_s = float(p("slow_pulse_period_s", 0.7))
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
        self.kick_max = float(p("kick_max", 0.35))       # logical effort cap (forward)
        # Separate reverse cap (2026-07-04 00:10: kick_max was cut to 0.30
        # for the hot F/R FORWARD curve and silently capped reverse at raw
        # 0.657 — below its breakaway — reverse efforts pinned at exactly
        # ff+kick_max all session). Reverse hardware is weaker and its
        # threshold higher: it needs MORE kick headroom, not the same.
        self.kick_max_rev = float(p("kick_max_rev", 0.55))
        self.kick_v_moving = float(p("kick_v_moving", 0.05))  # m/s, kick ends here
        # F/R ESC mode: kick works in reverse too (see _stiction_kick note).
        self.kick_reverse = bool(p("kick_reverse", True))
        # Phase-3 link #1: seed forward kicks at auto_calib's live breakaway.
        self.kick_seed_min_n = float(p("kick_seed_min_n", 10.0))
        self.kick_seed_lag = float(p("kick_seed_lag", 0.05))  # odom-lag inflation
        self.kick_seed_explore_every = max(2, int(p("kick_seed_explore_every", 5)))
        self._kick_starts = 0
        self._breakaway_est = None
        self._breakaway_est_rev = None
        # M2 steering learning (see _on_calib): per-direction feedforward
        # gain correction, 1.0 until auto_calib has enough pairs.
        self.steer_learn = bool(p("steer_learn", True))
        self.steer_corr_max = float(p("steer_corr_max", 2.5))
        self._steer_corr_l = 1.0
        self._steer_corr_r = 1.0
        self._stall_since = None
        self._kick = 0.0
        self.create_subscription(
            Float32MultiArray, p("calib_topic", "/auto_calib/params"),
            self._on_calib, 5)

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
        self._watch_sign = 0.0

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

        # --- vibration metric (OBSERVE ONLY for now) ---
        # Envelope of accel-magnitude deviation from its slow baseline, from
        # the already-throttled 30 Hz IMU feed. A moving robot vibrates; a
        # robot jammed against a wall with a stalled motor doesn't. Logged as
        # the last debug field to learn real thresholds from a few drives —
        # the plan is to let it (a) trip the watchdog FAST when odom reads 0
        # but the chassis clearly moves (blind runaway) and (b) veto the
        # stiction kick when we're already moving (odom lag), but no control
        # decision reads it until thresholds are validated.
        self.vib_enabled = bool(p("vibration_metric", True))
        self._acc_base = 9.81
        self.vib = 0.0
        self.accel_fwd = 0.0
        self._accx_bias = 0.0
        # Kick releases on signed IMU accel (>= this, m/s^2) OR odom motion —
        # whichever comes first. IMU reacts in ~50 ms vs odom's ~200 ms, so
        # the breakaway punch is cut short before it becomes a lunge. 0
        # disables (odom-only release).
        self.kick_accel_release = float(p("kick_accel_release", 0.8))
        self.zupt_vib_still = float(p("zupt_vib_still", 0.12))  # 0 disables
        self.rev_brake_only = bool(p("rev_brake_only", False))  # true = F/B/R-era PI handcuff
        self.governor_v = float(p("governor_v", 0.40))  # m/s hard cap, 0 disables
        # Reverse duty-regulation band: coast above |cmd|+band, drive below
        # (torque-mode reverse can't be held by constant effort — see _tick).
        self.governor_rev_band = float(p("governor_rev_band", 0.10))
        # Reverse pulse-and-coast (see _tick note): open-loop burst bounding,
        # because reverse ICP feedback lags ~0.5 s and any measurement-gated
        # scheme fires too late.
        self.rev_pulse = bool(p("rev_pulse", True))
        self.rev_pulse_drive_s = float(p("rev_pulse_drive_s", 0.30))
        self.rev_pulse_coast_s = float(p("rev_pulse_coast_s", 0.45))
        self._rev_phase_drive = False
        self._rev_phase_until = 0
        self.envelope_margin = float(p("envelope_margin", 0.05))  # raw above breakaway while rolling
        self.launch_cooldown_s = float(p("launch_cooldown_s", 0.35))
        self._cooldown_until = 0
        if self.vib_enabled:
            self.create_subscription(
                Imu, p("imu_topic", "/imu/data_ekf"), self._on_imu, 10)

        self.publish_debug = bool(p("publish_debug", False))

        self.cmd = (0.0, 0.0)
        self.last_cmd_ns = self.get_clock().now().nanoseconds
        self.v_meas = 0.0
        self.w_meas = 0.0
        self.last_odom_ns = 0
        self.integral = 0.0
        # Per-direction cruise-memory bank (see _speed_trim): the active
        # integral is banked/restored on commanded-direction changes.
        self._integrals = {1.0: 0.0, -1.0: 0.0}
        self._int_sign = 1.0
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
        # ZUPT gate (the phase-1 "vibration bit", finally wired 2026-07-04):
        # lidar-ICP odometry reports phantom velocity on a physically still
        # robot (21% of at-rest ticks, 00:05 run — chops movement episodes,
        # deadlocks the interlock, confuses kick/watchdog). A rolling chassis
        # ALWAYS vibrates (measured: parked vib <= 0.03, rolling >= 0.3), so
        # when the chassis is quiet the velocity is zero, whatever ICP says.
        # ZUPT v3 (2026-07-04 11:32 run): parked-drift suppression ONLY.
        # v2 also zeroed "big readings on a quiet chassis" (>0.3 m/s at
        # vib<0.12) — but SMOOTH TILE AT SPEED can be that quiet, so real
        # 0.7 m/s motion was zeroed inside the control node: the governor
        # went blind (cut only 27% of overspeed), the PI stopped braking,
        # and the stall logic FIRED THE KICK AT FULL SPEED (the lunges).
        # Lesson: never let a plausibility filter override the primary
        # sensor for CONTROL decisions at speed; only gate the no-command
        # case, where physics is unambiguous.
        if (self.vib_enabled and self.zupt_vib_still > 0.0
                and self.vib < self.zupt_vib_still
                and abs(self.cmd[0]) < 0.02 and abs(self.v_meas) > 0.02):
            self.v_meas = 0.0
        self.last_odom_ns = self.get_clock().now().nanoseconds

    def _on_imu(self, msg: Imu):
        a = msg.linear_acceleration
        m = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
        self._acc_base += 0.02 * (m - self._acc_base)   # slow gravity baseline
        self.vib += 0.3 * (abs(m - self._acc_base) - self.vib)  # fast envelope
        # Signed forward acceleration (fast low-pass of accel x, bias-tracked
        # slowly): motion-onset detector for the kick release. Buzz is
        # zero-MEAN vibration (envelope 0.74 but mean ~0); breakaway is a
        # signed step (~3 m/s^2 measured) — this separates them where the
        # envelope cannot (2026-07-03 23:10 log: buzz 0.74 vs rolling 0.59,
        # indistinguishable by envelope).
        self._accx_bias += 0.005 * (a.x - self._accx_bias)
        self.accel_fwd += 0.4 * ((a.x - self._accx_bias) - self.accel_fwd)

    def _raw_throttle(self, logical: float) -> float:
        if not self.apply_deadband:
            return clamp(logical)
        return logical_to_raw(logical, self.thr_dead_rev, self.thr_dead_fwd)

    def _raw_steer(self, logical: float) -> float:
        if not self.apply_deadband:
            return clamp(logical)
        return logical_to_raw(logical, self.steer_dead_neg, self.steer_dead_pos)

    def _interlock_steer(self, v, w_cmd) -> float:
        """Raw steering aimed at the (v, w_cmd) leg — feedforward only, no PI.

        Shared by the direction interlock and the cusp dwell to PRE-AIM the
        wheels while throttle is held neutral, so the servo-saver's 1-2 s
        wind-in happens BEFORE the leg starts instead of during it. Uses the
        commanded v sign (not measured) so a reverse leg gets the correct
        (opposite) steering sign; |v| cancels in atan(L*w/v) so the raw
        magnitude is speed-independent. Bicycle model + per-side learned
        gain correction, same as the main path."""
        if abs(v) < self.v_eps:
            return 0.0
        w_max = abs(v) * self.kappa_max
        w_clamped = clamp(w_cmd, -w_max, w_max)
        steer_log = clamp(math.atan(self.L * w_clamped / v) / self.max_steer,
                          -1.0, 1.0)
        if steer_log > 0.0:
            steer_log = clamp(steer_log * self._steer_corr_l)
        elif steer_log < 0.0:
            steer_log = clamp(steer_log * self._steer_corr_r)
        return float(self._raw_steer(steer_log))

    def _tick(self):
        now = self.get_clock().now().nanoseconds
        if now - self.last_cmd_ns > self.timeout_ns:
            # Stale cmd: hard neutral. No trims either — the yaw PI must not
            # counter-steer a coasting robot that nobody is commanding.
            # Watchdog resets too: without this the trip LATCHED across stale
            # gaps (watch_since kept its old epoch), so every later command
            # re-tripped instantly and the robot was bricked (2026-07-02
            # pulse test: all pulses got eff 0.00).
            # SPEED integral NOT reset (2026-07-04 18:36 trace): goal gaps and
            # CM stops hit this branch constantly, so wiping it here made the
            # PI re-find today's plant from zero after every pause — cruise
            # never converged and ran on the stale static map (governor-chop
            # lunges). The integral is cruise memory (see _speed_trim note);
            # it persists across pauses and is banked per direction.
            self.yaw_integral = 0.0
            self._watch_since = None
            self._watch_tripped = False
            self.pub.publish(Float32MultiArray(data=[0.0, 0.0]))
            if self.pub_debug is not None:
                self.pub_debug.publish(Float32MultiArray(data=[
                    0.0, 0.0, float(self.v_meas), float(self.w_meas),
                    0.0, 0.0, 0.0, 0.0, 0.0, float(self.kappa_max), 0.0,
                    float(self.vib)]))
            return
        v, w_cmd = self.cmd

        # Commanded drive-direction flip tracking (2026-07-05): stamps the
        # flip time for the interlock window below and arms the cusp dwell.
        # Runs BEFORE the interlock so the stamp exists even during a hold.
        drive_sign = 1.0 if v > self.v_eps else (-1.0 if v < -self.v_eps else 0.0)
        if (drive_sign != 0.0 and self._drive_sign != 0.0
                and drive_sign != self._drive_sign):
            self._flip_ns = now
            if self.steer_settle_s > 0.0:
                self._settle_until_ns = now + int(self.steer_settle_s * 1e9)
        if drive_sign != 0.0:
            self._drive_sign = drive_sign

        # Direction-change interlock: commanded sign opposes measured motion.
        # Neutral throttle + centered steer until the robot has nearly stopped,
        # then the new direction engages with the correct steering sign.
        # MAX-HOLD ESCAPE (2026-07-03 18:26 log): phantom odometry can report
        # opposing motion that never ends (16% phantom ticks; reverse effort
        # was NEUTRAL for most reverse stalls -> "barely moves backward").
        # The drivetrain physically needs ~1.3 s to change direction, so
        # after interlock_max_hold_s of continuous hold we engage regardless:
        # a real coast has stopped by then; only a lying odometer hasn't.
        # Windowed to interlock_window_s after a commanded flip (2026-07-05,
        # 23:57 trace): while REVERSING, ICP loses tracking and reports
        # phantom FORWARD speeds up to +0.35 — the un-windowed interlock read
        # that as "opposing motion" and cut throttle mid-reverse (60% of
        # commanded-reverse ticks had zero effort). Opposing measured motion
        # is only physically plausible right after a flip (coasting the old
        # direction); outside the window it's a lying odometer — drive on.
        odom_fresh = (now - self.last_odom_ns) <= self.odom_timeout_ns
        if (self.interlock and odom_fresh and abs(v) > self.v_eps
                and v * self.v_meas < 0.0
                and abs(self.v_meas) > self.interlock_v_stop
                and (now - self._flip_ns) <= self.interlock_window_ns):
            if self._interlock_since is None:
                self._interlock_since = now
            held = (now - self._interlock_since) / 1e9
        else:
            self._interlock_since = None
            held = None
        if held is not None and held <= self.interlock_max_hold_s:
            # yaw trim resets (steering sign flips with direction); the speed
            # integral persists — it's banked per direction in _speed_trim.
            self.yaw_integral = 0.0
            # Throttle neutral, but STEER TOWARD THE NEW LEG (2026-07-05
            # 01:27 cusp trace): the old [0,0] CENTERED the wheels during the
            # hold, so a 3-point turn coasted to a stop straight, then the
            # new leg's steering only started winding in AFTER motion began —
            # the servo-saver's 1-2 s lag meant the whole reversal ran on
            # near-centered wheels and the turn failed. Pre-aim during the
            # hold (same math as the cusp dwell) so the wheels are already
            # wound toward the new leg when throttle engages.
            steer_hold = self._interlock_steer(v, w_cmd)
            self.pub.publish(Float32MultiArray(data=[steer_hold, 0.0]))
            if self.pub_debug is not None:
                self.pub_debug.publish(Float32MultiArray(data=[
                    0.0, 0.0, float(self.v_meas), float(self.w_meas),
                    float(w_cmd), 0.0, steer_hold, 0.0, 0.0,
                    float(self.kappa_max), 1.0, float(self.vib)]))
            return

        # Cusp dwell (see __init__ note): armed by the flip tracking at the
        # top of _tick; runs after the interlock so the robot has already
        # coasted to a stop. Pre-sets the steering with throttle neutral; the
        # kick timer is reset so its delay window starts when the dwell ends,
        # and the watchdog block below re-arms on the sign flip afterwards.
        if now < self._settle_until_ns and drive_sign != 0.0:
            self._kick = 0.0
            self._stall_since = None
            self.pub.publish(Float32MultiArray(
                data=[self._interlock_steer(v, w_cmd), 0.0]))
            if self.pub_debug is not None:
                self.pub_debug.publish(Float32MultiArray(data=[
                    0.0, 0.0, float(self.v_meas), float(self.w_meas),
                    float(w_cmd), 0.0, self._interlock_steer(v, w_cmd), 0.0, 0.0,
                    float(self.kappa_max), 3.0, float(self.vib)]))
            return

        # Odometry-distrust watchdog (see __init__ note). Runs after the
        # interlock so a legitimate direction change doesn't count as a stall.
        # A command sign flip re-arms it: driving the OTHER way is the escape
        # route from a stall, and Nav2 flips sign without releasing the cmd —
        # without this, a forward-stall trip also blocked the reverse escape
        # (2026-07-02 21:21 log, t=63.5/70: cmd -0.3 got eff 0.00).
        sign = 1.0 if v > self.v_eps else (-1.0 if v < -self.v_eps else 0.0)
        if sign != 0.0 and sign != self._watch_sign:
            self._watch_since = None
            self._watch_tripped = False
            self._watch_sign = sign
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
                    float(self.kappa_max), 2.0, float(self.vib)]))
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

        # Learned steering-gain correction (M2): the linkage delivers less
        # curvature than the ideal map assumes, asymmetrically. Applied to
        # the feedforward BEFORE the yaw PI so the PI only trims residuals.
        if steer_log > 0.0:
            steer_log = clamp(steer_log * self._steer_corr_l)
        elif steer_log < 0.0:
            steer_log = clamp(steer_log * self._steer_corr_r)

        steer_trim = self._steer_trim(steer_log, v, now)
        steer_log_out = clamp(steer_log + steer_trim)
        steer_log_out = self._hysteresis_comp(steer_log_out, now)

        # PULSE-AND-COAST crawl (2026-07-03): the drivetrain's slowest
        # CONTINUOUS forward speed is ~0.30 m/s (below raw ~0.53 it stalls;
        # at 0.53-0.56 it does ~0.32 — measured). Commands below the floor
        # are physically unreachable in steady state, so emulate them the way
        # a human crawls an RC car: drive at the floor speed for duty
        # fraction of each period, coast (neutral throttle, steering live)
        # the rest. v_thr feeds ONLY the throttle chain; steering, interlock
        # and watchdog still see the true command. Coast phases zero the
        # kick via its own |v_cmd|<0.05 gate, so coasting can't wind a punch.
        v_thr = v
        if self.slow_pulse and 0.05 < v < self.slow_floor_v:
            duty = max(0.35, min(1.0, v / self.slow_floor_v))
            phase = ((now / 1e9) % self.slow_period_s) / self.slow_period_s
            v_thr = self.slow_floor_v if phase < duty else 0.0

        speed_scale = self.max_speed if v >= 0.0 else self.max_speed_rev
        throttle_log = clamp(v_thr / speed_scale, -1.0, 1.0)
        speed_trim = self._speed_trim(v_thr, now) if self.closed_loop else 0.0
        throttle_log = clamp(throttle_log + speed_trim)
        throttle_log = self._stiction_kick(throttle_log, v_thr, now)
        # REVERSE PULSE-AND-COAST (2026-07-05 00:39 trace): 1.0 s of reverse
        # engagement effort (raw -0.52..-0.56) = 0 -> REAL -1.2 m/s, and the
        # ICP measurement lags ~0.5 s in reverse, so every feedback layer
        # (PI, envelope, hysteresis) fires only after the robot is already
        # 3x past the target. The burst length must be bounded OPEN-LOOP:
        # drive at most rev_pulse_drive_s (~0.3 m/s of speed gain at the
        # measured ~1 m/s^2), then coast rev_pulse_coast_s; at each cycle
        # boundary (where lag matters least) skip the next burst if the
        # average is already at/above target. The kick still handles
        # breakaway inside each drive phase (its timer restarts per phase).
        if self.rev_pulse and v < 0.0:
            if now >= self._rev_phase_until:
                if self._rev_phase_drive:
                    self._rev_phase_drive = False
                    self._rev_phase_until = now + int(self.rev_pulse_coast_s * 1e9)
                elif abs(self.v_meas) > abs(v):
                    self._rev_phase_until = now + int(self.rev_pulse_coast_s * 1e9)
                else:
                    self._rev_phase_drive = True
                    self._rev_phase_until = now + int(self.rev_pulse_drive_s * 1e9)
            if not self._rev_phase_drive:
                throttle_log = 0.0
                self._stall_since = None
                self._kick = 0.0
        elif self._rev_phase_drive or self._rev_phase_until:
            self._rev_phase_drive = False
            self._rev_phase_until = 0
        # PREVENTIVE EFFORT ENVELOPE (2026-07-04, replaces reliance on the
        # reactive governor — a 3 m/s^2 robot outruns a 10 Hz cut): while
        # ROLLING forward, never emit effort above learned-breakaway + margin.
        # Physics: speed just past breakaway is the minimum sustainable
        # speed on ANY battery state; surplus effort above it buys only
        # overspeed. The cap moves with the battery because breakaway
        # learning tracks the plant. (Kick is applied before this and may
        # exceed it briefly from standstill; it bleeds on motion.)
        # Motion gate 0.15 (was kick_v_moving 0.05, 2026-07-04 18:36 trace):
        # 0.05 is inside ICP's noise floor, so phantom jitter triggered the
        # cap on a PHYSICALLY STALLED robot — effort yo-yoed 0.56 <-> 0.37
        # (kick vs envelope) and starts never completed. Cap only on motion
        # the odometry can't fake.
        if (v > 0.0 and self._breakaway_est
                and abs(self.v_meas) >= 0.15 and self._kick <= 0.0):
            cap_raw = min(1.0, self._breakaway_est + self.envelope_margin)
            cap_log = (cap_raw - self.thr_dead_fwd) / max(1e-6, 1.0 - self.thr_dead_fwd)
            throttle_log = min(throttle_log, max(0.0, cap_log))
        # REVERSE envelope (2026-07-05, from the 23:57 trace: reverse swung
        # to -0.70 m/s with only the reactive governor defending — the
        # envelope was forward-only). Reverse is a TORQUE command (raw -0.50
        # held for 3 s ran 0.2 -> 0.9 m/s, 2026-07-02 sweep), so effort
        # deeper than the engagement zone only buys acceleration it can't
        # shed. Same preventive cap, reverse map, once genuinely rolling.
        if (v < 0.0 and self._breakaway_est_rev
                and abs(self.v_meas) >= 0.15 and self._kick <= 0.0):
            cap_raw = min(1.0, self._breakaway_est_rev + self.envelope_margin)
            cap_log = (cap_raw - self.thr_dead_rev) / max(1e-6, 1.0 - self.thr_dead_rev)
            throttle_log = max(throttle_log, -max(0.0, cap_log))
        # REVERSE speed hysteresis (2026-07-05 00:23 trace: constant eff -0.51
        # accelerated -0.06 -> -1.02 m/s in 1.6 s). CORRECTED MODEL (same
        # night, after checking ESC docs): reverse is NOT a torque command —
        # both directions are voltage control — but a general brushed ESC
        # maps full reverse to only ~50% power (QuicRun rev stage: 30 A vs
        # 60 A), compressing the whole curve: the effort that HOLDS a slow
        # reverse speed exists but in an even thinner sliver than forward's,
        # and everything past it has a terminal speed >= 1 m/s (the 07-02
        # "constant acceleration" sweep was the transient toward it). So:
        # coast above |cmd| + band, re-engage below — a BOUNDED layer while
        # the per-direction PI + sigma-delta dither search for the true
        # holding effort; as the reverse integral converges these coasts
        # should become rare. Forward keeps the plain last-resort governor.
        if (v < 0.0 and self.v_meas < 0.0
                and abs(self.v_meas) > abs(v) + self.governor_rev_band):
            throttle_log = 0.0
        # Reactive governor kept as last resort (measured speed hard cap).
        if (self.governor_v > 0.0 and abs(self.v_meas) > self.governor_v
                and v * self.v_meas > 0.0):
            throttle_log = min(throttle_log, 0.0) if v > 0.0 \
                else max(throttle_log, 0.0)
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
                float(self.vib),
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

        BOTH directions since 2026-07-03 (kick_reverse): the old FORWARD-ONLY
        gate existed because the driver's double-tap read as a stall and fired
        the kick right as the ESC bit; the ESC Mode jumper is now F/R
        (single-click reverse, esc_double_tap false) so that failure mode is
        gone. Reverse kick authority is bounded by raw_throttle_max_rev.

        Forward kicks SEED at auto_calib's live breakaway estimate (phase-3
        link #1): instead of ramping from zero through the buzz zone, jump
        straight to just under the measured breakaway-of-the-moment — which
        tracks surface (carpet vs tile) and motor temperature — then ramp the
        rest. Falls back to plain ramping until enough events are collected."""
        gate = (v_cmd < 0.05) if not self.kick_reverse else (abs(v_cmd) < 0.05)
        if not self.kick_enabled or gate \
                or (now - self.last_odom_ns) > self.odom_timeout_ns:
            self._stall_since = None
            self._kick = 0.0
            return throttle_log
        imu_moving = (self.kick_accel_release > 0.0 and self.vib_enabled
                      and v_cmd * self.accel_fwd > 0.0
                      and abs(self.accel_fwd) >= self.kick_accel_release)
        if abs(self.v_meas) >= self.kick_v_moving or imu_moving:
            # Rolling: drop the kick INSTANTLY. The 0.2 s bleed-off kept boosting
            # a drivetrain already past breakaway and fed the reverse surge that
            # the speed PI then overcorrected (2026-06-11 18:33 run).
            # imu_moving: signed-accel onset (~50 ms) beats odom (~200 ms) to
            # the release, cutting the breakaway punch before it lunges.
            if self._kick > 0.0 and self.launch_cooldown_s > 0.0:
                # LAUNCH COOLDOWN (2026-07-03 23:15 log: starts peaked 0.64
                # m/s at effort only 0.53-0.58): crossing breakaway leaves a
                # huge torque surplus (static >> rolling friction), and the
                # feedforward kept pushing through the PI's ~0.3 s blind
                # spot. Coast (neutral throttle) briefly right at motion
                # onset to shed the slingshot instead of feeding it.
                self._cooldown_until = now + int(self.launch_cooldown_s * 1e9)
            self._stall_since = None
            self._kick = 0.0
        else:
            if self._stall_since is None:
                self._stall_since = now
            elif (now - self._stall_since) / 1e9 >= self.kick_delay_s:
                est = self._breakaway_est if v_cmd > 0.0 else self._breakaway_est_rev
                if self._kick == 0.0 and est:
                    self._kick_starts += 1
                    if self._kick_starts % self.kick_seed_explore_every == 0:
                        # EXPLORER start: plain ramp, no seed. A seeded start
                        # jumps effort over the threshold, so auto_calib
                        # can't measure from it (it rejects jump-onsets);
                        # without these periodic honest sweeps the estimate
                        # could only ratchet UP, never track a surface
                        # getting easier. Costs one slow buzzy start in N.
                        pass
                    else:
                        # Seed just under the learned breakaway (median is
                        # inflated ~kick_seed_lag by odom latency); the ramp
                        # covers the rest. Works in BOTH directions since
                        # 2026-07-04 (reverse est from the raw-sweep seed +
                        # live events).
                        dead = self.thr_dead_fwd if v_cmd > 0.0 else self.thr_dead_rev
                        cap = self.kick_max if v_cmd > 0.0 else self.kick_max_rev
                        need = ((est - self.kick_seed_lag - dead)
                                / max(1e-6, 1.0 - dead)) - abs(throttle_log)
                        self._kick = max(0.0, min(cap, need))
                cap = self.kick_max if v_cmd > 0.0 else self.kick_max_rev
                self._kick = min(cap, self._kick + self.kick_rate * self.dt)
        if now < self._cooldown_until:
            # Launch cooldown: coast off the breakaway surplus — but ONLY
            # while there is surplus. Unconditional coasting (first version)
            # shed all momentum at low cruise speeds -> re-stall -> re-kick:
            # 78 hop-starts in 157 s, continuity 0/100 (2026-07-03 23:38).
            if abs(self.v_meas) > abs(v_cmd) * 1.2:
                return 0.0
            self._cooldown_until = 0   # surplus gone; resume feedforward
        if self._kick <= 0.0:
            return throttle_log
        return clamp(throttle_log + math.copysign(self._kick, v_cmd))

    def _on_calib(self, msg):
        """/auto_calib/params: [fwd_breakaway_med, n_events, ...].

        Plausibility band 0.40..0.88: a healthy drivetrain NEVER needs
        full throttle to break away — estimates above 0.88 mean the learned
        state is poisoned (2026-07-03: F/B-jumper chaos taught it 1.00 and
        every start became a full-throttle rocket). Poisoned values are
        refused, not clamped, so the explorer starts can re-learn truth."""
        if len(msg.data) >= 2 and msg.data[1] >= self.kick_seed_min_n:
            est = float(msg.data[0])
            if 0.40 <= est <= 0.88:
                if self._breakaway_est is None:
                    self.get_logger().info(
                        f"kick seeding live: breakaway={est:.2f} "
                        f"(n={int(msg.data[1])}) from auto_calib")
                self._breakaway_est = est
            else:
                if self._breakaway_est is not None or est > 0.88:
                    self.get_logger().warn(
                        f"refusing implausible breakaway estimate {est:.2f} "
                        "(poisoned state? seeding disabled until it recovers)",
                        throttle_duration_sec=30.0)
                self._breakaway_est = None
        # Fields 10,11: REVERSE breakaway (learned since 2026-07-04, seeded
        # from the manual raw sweep: engaged at -0.60). Same plausibility
        # guard, mirrored.
        if len(msg.data) >= 12 and msg.data[11] >= 3:
            est = float(msg.data[10])
            if 0.45 <= est <= 0.85:
                if self._breakaway_est_rev is None:
                    self.get_logger().info(
                        f"REV kick seeding live: breakaway={est:.2f} "
                        f"(n={int(msg.data[11])})")
                self._breakaway_est_rev = est
            else:
                self._breakaway_est_rev = None
        # Fields 12-15: learned steering gain (kappa per unit effort), LEFT
        # then RIGHT (M2, 2026-07-04: measured 0.40/0.67 of ideal — wide
        # corners + strong asymmetry). Correction = ideal/measured, bounded:
        # never below 1.0 (the linkage under-steers, never over) and capped
        # so a bad estimate can't slam the servo.
        if len(msg.data) >= 16 and self.steer_learn:
            ideal = self.kappa_max  # tan(max_steer)/L = kappa at effort 1.0
            for i, attr in ((12, "_steer_corr_l"), (14, "_steer_corr_r")):
                g, n = float(msg.data[i]), float(msg.data[i + 1])
                if n >= 15 and 0.1 <= g <= 2.0:
                    # Floor 0.5 (was 1.0): the new servo's RIGHT side measures
                    # gain 1.04 vs ideal 0.80 — it physically OVERSHOOTS, and
                    # a >=1.0 floor forbade correcting down ("overshoots
                    # often", 2026-07-04). Corrections may now shrink as well
                    # as grow.
                    corr = max(0.5, min(self.steer_corr_max, ideal / g))
                    if getattr(self, attr) == 1.0 and corr > 1.05:
                        self.get_logger().info(
                            f"steering correction live ({attr[-1].upper()}): "
                            f"x{corr:.2f} (measured gain {g:.2f}, n={int(n)})")
                    setattr(self, attr, corr)

    def _speed_trim(self, v_cmd, now):
        if abs(v_cmd) < 0.05 or (now - self.last_odom_ns) > self.odom_timeout_ns:
            # No reset (2026-07-04): idle ticks between commands must not
            # erase the learned cruise trim — see the stale-cmd note in _tick.
            return 0.0
        # Per-direction cruise memory (2026-07-04 18:36 trace): forward and
        # reverse plants differ, and direction flips every few seconds in
        # maneuver-heavy driving — one shared integral (reset on every flip
        # by the old branches) meant the PI NEVER converged on either plant.
        # Bank the active integral on a sign change and restore the other
        # side's last value; the static map is just the anchor, this is what
        # tracks today's battery/temp/surface until M3's learned map owns it.
        sign = 1.0 if v_cmd > 0 else -1.0
        if sign != self._int_sign:
            self._integrals[self._int_sign] = self.integral
            self.integral = self._integrals[sign]
            self._int_sign = sign
        # Turning: odom_v is phantom-inflated by ICP rotation coupling, so the
        # error is garbage -> freeze the integral and run feedforward only.
        if abs(self.w_meas) > self.speed_yaw_gate:
            return 0.0
        # STALLED: starting is the KICK's job, not the PI's. Letting the
        # integral wind during a stall stacked +0.09 raw on top of the kick,
        # and unlike the kick it does NOT drop on motion -- it unwound over
        # ~1 s of post-breakaway surge (2026-07-03 18:23 log: efforts pinned
        # at ceiling, lunge on every start). Freeze AND bleed it while
        # stationary so breakaway happens at kick-level effort only.
        if abs(self.v_meas) < 0.05 and abs(v_cmd) >= 0.05:
            # HOLD the integral during stalls — don't wind it (lunge fuel,
            # 18:23 fix) but ALSO don't bleed it (the 0.9 decay erased the
            # learned keep-rolling trim on every micro-stall, so the robot
            # always resumed at too-weak effort and hopped forever —
            # 2026-07-03 23:38: 78 hop-starts). Starting is the kick's job;
            # the integral is the cruise memory and must survive the stall.
            return clamp(self.kp * (v_cmd - self.v_meas), 0.0, 0.05) \
                if v_cmd > 0 else 0.0
        err = v_cmd - self.v_meas
        self.integral = clamp(self.integral + err * self.dt, -0.3, 0.3)
        trim = clamp(self.kp * err + self.ki * self.integral,
                     -self.trim_max, self.trim_max)
        if v_cmd < 0.0 and self.rev_brake_only:
            # F/B/R-era protection: that ESC mode self-accelerated in reverse
            # at constant effort (raw sweep 2026-07-02), so the PI was only
            # allowed to SHED reverse speed, never add. In F/R mode reverse
            # is a proportional drive and this handcuff caused pulsing —
            # kick starts it, feedforward alone can't sustain it, PI
            # forbidden to help, stall, repeat (2026-07-04). Now full
            # bidirectional trim by default; the watchdog + ceiling + ZUPT
            # still bound a blind-reverse runaway. Set rev_brake_only true
            # again if the jumper ever returns to F/B/R.
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
