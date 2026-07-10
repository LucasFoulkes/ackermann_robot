#!/usr/bin/env python3
"""Online drivetrain self-calibration (observe mode).

Learns the static feedforward map from normal driving, using estimators chosen
to survive closed-loop operation (naive regression on drive logs fails two
ways, both hit 2026-07-02: dynamic fits are biased by the PI anti-correlating
effort with speed, and steady-state fits are biased by terrain selection --
the PI settles at high effort exactly where the ground is hard, so passive
equilibria trace the terrain distribution, not the motor curve).

What IS identifiable from passive driving:
  * FORWARD BREAKAWAY u0 -- an event, not an equilibrium: during a stiction-
    kick ramp, the effort at which |v| first crosses v_go. Validated offline
    on 2026-07-02 logs: 51 events, median 0.65, IQR 0.64-0.68 (vs yaml 0.45!).
    Reverse is NOT observable this way (engagement is double-tap-gated, not
    torque-gated); reverse u0 comes from the raw sweep (0.45).
  * STEADY (u, v) pairs -- published for logging and fitted with a guard: a
    negative apparent gain means terrain variation dominates and the fit is
    flagged, not trusted. A trustworthy gain needs active excitation (a small
    known dither to correlate against) -- that's phase 2, deliberately not
    done here.

OBSERVE ONLY: reports every report_period_s, publishes /auto_calib/params
[fwd_breakaway_med, n_breakaway, G_fwd, u0_fwd, n_steady_fwd, G_rev, u0_rev,
n_steady_rev, fit_trust_fwd, fit_trust_rev]. Nothing feeds control.
"""
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray


def fit_line(pairs):
    """v = G*u + b -> (G, u0=-b/G). None if degenerate."""
    n = len(pairs)
    if n < 6:
        return None
    su = sum(p[0] for p in pairs)
    sv = sum(p[1] for p in pairs)
    suu = sum(p[0] * p[0] for p in pairs)
    suv = sum(p[0] * p[1] for p in pairs)
    den = n * suu - su * su
    if abs(den) < 1e-9:
        return None
    G = (n * suv - su * sv) / den
    b = (sv - G * su) / n
    if abs(G) < 1e-6:
        return None
    return G, -b / G


class AutoCalib(Node):
    def __init__(self):
        super().__init__("auto_calib")
        p = lambda n, d: self.declare_parameter(n, d).value

        self.w_gate = float(p("w_gate", 0.2))
        self.u_min = float(p("u_min", 0.25))
        self.v_still = float(p("v_still", 0.02))
        self.v_go = float(p("v_go", 0.06))
        self.still_min_s = float(p("still_min_s", 0.4))
        self.steady_win_s = float(p("steady_win_s", 0.5))
        self.max_pairs = int(p("max_steady_pairs", 400))
        self.report_period = float(p("report_period_s", 15.0))
        self.recent_window = max(3, int(p("recent_window", 12)))
        self.jump_gate = float(p("jump_gate", 0.12))  # seeded-start rejection
        self._u_hist = []

        self.breakaways = []            # forward breakaway efforts
        # Reverse ring starts with the 2026-07-04 manual raw sweep result
        # (robot engaged at raw -0.60 under load, F/R mode, 100 Hz PWM) so
        # reverse seeding works from the first drive; live events replace it.
        self.rev_breakaways = [0.60, 0.60, 0.60]
        self.pairs = {1: [], -1: []}    # steady (u, v) per direction
        self.steer_pairs = {1: [], -1: []}  # (|steer_effort|, |kappa|) per side
        self._s = None                  # latest steering effort
        self._s_hist = []               # (t, steer effort) for settledness
        self._u = None                  # (t, u) latest effort
        self._still_since = None
        self._win = []                  # rolling (t, u, v, |w|) for steady detect

        # Persistence (Duckietown pattern, docs/platform_survey.md): learned
        # state survives restarts, so kick seeding engages from launch instead
        # of re-earning n>=10 events every session (sessions are ~5 min; a
        # fresh node never got there — 2026-07-03 00:31 run: n=2 at teardown).
        # Old events age out naturally via the 200-event ring in _on_odom.
        self.state_path = os.path.expanduser(
            str(p("state_path", "~/.ros/auto_calib_state.json")))
        self._load_state()
        self.create_timer(float(p("save_period_s", 30.0)), self._save_state)

        self.create_subscription(
            Float32MultiArray, p("effort_topic", "/ackermann/cmd_effort"),
            self._on_effort, 10)
        self.create_subscription(
            Odometry, p("odom_topic", "/odom"), self._on_odom, 10)
        self.pub = self.create_publisher(Float32MultiArray, "/auto_calib/params", 5)
        self.create_timer(self.report_period, self._report)
        self.get_logger().info("observe mode (breakaway + steady pairs)")

    def _load_state(self):
        try:
            with open(self.state_path) as fh:
                s = json.load(fh)
            self.breakaways = [float(x) for x in s.get("breakaways", [])][-200:]
            if s.get("rev_breakaways"):
                self.rev_breakaways = [float(x) for x in s["rev_breakaways"]][-200:]
            for k in (1, -1):
                if s.get(f"steer_pairs_{k}"):
                    self.steer_pairs[k] = [tuple(map(float, t))
                                           for t in s[f"steer_pairs_{k}"]][-300:]
            for k in (1, -1):
                self.pairs[k] = [tuple(map(float, t))
                                 for t in s.get(f"pairs_{k}", [])][-self.max_pairs:]
            if self.breakaways or self.pairs[1] or self.pairs[-1]:
                self.get_logger().info(
                    f"restored state: {len(self.breakaways)} breakaway events, "
                    f"{len(self.pairs[1])}/{len(self.pairs[-1])} steady pairs "
                    f"(saved {s.get('saved_at', '?')})")
        except FileNotFoundError:
            pass
        except Exception as e:
            self.get_logger().warn(f"state load failed ({e}); starting fresh")

    def _save_state(self):
        try:
            tmp = self.state_path + ".tmp"
            with open(tmp, "w") as fh:
                json.dump({
                    "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "breakaways": self.breakaways,
                    "rev_breakaways": self.rev_breakaways,
                    "pairs_1": self.pairs[1],
                    "pairs_-1": self.pairs[-1],
                    "steer_pairs_1": self.steer_pairs[1],
                    "steer_pairs_-1": self.steer_pairs[-1],
                }, fh)
            os.replace(tmp, self.state_path)
        except Exception as e:
            self.get_logger().warn(f"state save failed: {e}",
                                   throttle_duration_sec=60.0)

    def _on_effort(self, msg):
        if len(msg.data) >= 2:
            now = self.get_clock().now().nanoseconds / 1e9
            self._u = (now, float(msg.data[1]))
            self._s = float(msg.data[0])   # steering effort (M2, 2026-07-04)
            self._u_hist.append(self._u)
            while self._u_hist and now - self._u_hist[0][0] > 1.0:
                self._u_hist.pop(0)

    def _on_odom(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        if self._u is None or now - self._u[0] > 0.5:
            self._still_since = None
            self._win = []
            return
        u = self._u[1]
        v = msg.twist.twist.linear.x
        w = abs(msg.twist.twist.angular.z)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

        # --- breakaway detector, BOTH directions (reverse added 2026-07-04:
        # the forward-only condition left reverse on hand-set constants, and
        # nobody noticed until reverse died at a stale ceiling and a HUMAN
        # had to run the raw sweep the robot should have been doing) ---
        if abs(v) < self.v_still:
            if self._still_since is None:
                self._still_since = t
        elif abs(v) >= self.v_go:
            if (self._still_since is not None
                    and (t - self._still_since) > self.still_min_s
                    and abs(u) > self.u_min and u * v > 0):
                # Anti-ratchet (2026-07-03 00:53: published median climbed
                # 0.65 -> 0.98 across seeded sessions): a KICK-SEEDED start
                # JUMPS effort over the true threshold, so effort-at-onset
                # measures the seed, not breakaway — feeding it back ratchets
                # the estimate to the ceiling. Only a SWEPT onset (effort
                # rose gradually through the threshold) is a measurement:
                # reject events where |effort| jumped > jump_gate within the
                # 0.4 s before onset. Seeded starts produce no events; the
                # explorer starts in cmd_vel_to_effort (kick_seed_explore)
                # keep honest events flowing so the estimate can also FALL.
                past = [abs(uu) for (tt, uu) in self._u_hist
                        if 0.15 <= (self._u[0] - tt) <= 0.45]
                jumped = past and (abs(u) - min(past)) > self.jump_gate
                if not jumped:
                    ring = self.breakaways if v > 0 else self.rev_breakaways
                    ring.append(abs(u))
                    if len(ring) > 200:
                        ring.pop(0)
            self._still_since = None

        # --- steering-gain pairs (M2, 2026-07-04): while genuinely rolling
        # and steering, record (steer_effort, measured curvature). Both
        # sides of the pair are MEASURED, so unlike breakaway this estimator
        # cannot ratchet on its own corrections — applying a gain fix just
        # moves the operating point; the plant map s->kappa stays what it is.
        # 00:42 run measured achieved/commanded curvature 0.40 LEFT / 0.67
        # RIGHT — steering was the last unlearned map and the worst one. ---
        w_signed = msg.twist.twist.angular.z
        if self._s is not None:
            self._s_hist.append((t, self._s))
            while self._s_hist and t - self._s_hist[0][0] > 1.0:
                self._s_hist.pop(0)
        # SETTLED-ONLY sampling (2026-07-04): the linkage winds in over
        # ~1-2 s, so per-tick pairs mostly catch the wheels MID-SWING —
        # measured gains flip-flopped 7x between sessions and went negative.
        # A pair counts only if steering effort has been steady ~0.8 s
        # (wind-in complete), like the bench sweeps that measure cleanly.
        settled = (len(self._s_hist) >= 6
                   and t - self._s_hist[0][0] >= 0.75
                   and max(abs(s - self._s) for _, s in self._s_hist) < 0.06)
        if (self._s is not None and abs(self._s) > 0.12
                and abs(v) > 0.10 and abs(w_signed) > 0.03
                and v > 0 and settled):
            kappa = w_signed / v
            if kappa * self._s > 0:   # same sign = plausible sample
                ring = self.steer_pairs[1 if self._s > 0 else -1]
                ring.append((abs(self._s), abs(kappa)))
                if len(ring) > 300:
                    ring.pop(0)

        # --- steady-pair detector ---
        self._win.append((t, u, v, w))
        t0 = self._win[0][0]
        if t - t0 < self.steady_win_s:
            return
        while self._win and t - self._win[0][0] > self.steady_win_s:
            self._win.pop(0)
        us = [x[1] for x in self._win]
        vs = [x[2] for x in self._win]
        ws = [x[3] for x in self._win]
        um = sum(us) / len(us)
        vm = sum(vs) / len(vs)
        same_sign = all(x > self.u_min for x in us) or all(x < -self.u_min for x in us)
        moving = abs(vm) > 0.05 and (vm > 0) == (um > 0)
        if (same_sign and moving and max(us) - min(us) < 0.06
                and max(vs) - min(vs) < 0.08 and max(ws) < self.w_gate):
            d = 1 if um > 0 else -1
            self.pairs[d].append((um, vm))
            if len(self.pairs[d]) > self.max_pairs:
                self.pairs[d].pop(0)
            self._win = []          # decimate: no overlapping windows

    def _report(self):
        out = []
        if self.breakaways:
            b = sorted(self.breakaways)
            med = b[len(b) // 2]
            # PUBLISHED value = median of the RECENT window, not all-time:
            # with persistence the ring mixes surfaces (tile ~0.65 + carpet
            # >=0.90 -> all-time median 0.82, 2026-07-03 00:43), and seeding
            # a tile start at a carpet-biased value LUNGES. The recent window
            # tracks the surface under the wheels within ~a dozen starts;
            # the full ring still feeds the logged IQR/stats.
            recent = sorted(self.breakaways[-int(self.recent_window):])
            rmed = recent[len(recent) // 2]
            self.get_logger().info(
                f"FWD breakaway: n={len(b)} median={med:.2f} "
                f"IQR {b[len(b)//4]:.2f}-{b[3*len(b)//4]:.2f} "
                f"recent{len(recent)}={rmed:.2f} <- published "
                f"(odom lag inflates ~0.05)")
            out.extend([rmed, float(len(b))])
        else:
            self.get_logger().info("FWD breakaway: no events yet")
            out.extend([0.0, 0.0])
        for d, name in ((1, "FWD"), (-1, "REV")):
            pairs = self.pairs[d]
            r = fit_line(pairs)
            trust = 0.0
            if r is None:
                self.get_logger().info(f"{name} steady: n={len(pairs)} (no fit)")
                out.extend([0.0, 0.0, float(len(pairs))])
            else:
                G, u0 = r
                # closed-loop terrain bias makes G <= 0; only a positive-gain
                # fit with decent effort spread deserves any trust
                span = max(p[0] for p in pairs) - min(p[0] for p in pairs)
                trust = 1.0 if (G * d > 0 and abs(span) > 0.08) else 0.0
                trust_s = "OK" if trust else \
                    "LOW (terrain-biased or narrow span; needs phase-2 dither)"
                self.get_logger().info(
                    f"{name} steady: n={len(pairs)} G={G:+.2f} u0={u0:+.2f} "
                    f"span={span:.2f} trust={trust_s}")
                out.extend([G, u0, float(len(pairs))])
            out.append(trust)
        # Fields 10,11: REVERSE breakaway (recent-window median, n) — same
        # estimator as forward since 2026-07-04; consumed by cmd_vel_to_effort
        # for reverse kick seeding.
        if self.rev_breakaways:
            recent = sorted(self.rev_breakaways[-int(self.recent_window):])
            rmed = recent[len(recent) // 2]
            self.get_logger().info(
                f"REV breakaway: n={len(self.rev_breakaways)} "
                f"recent{len(recent)}={rmed:.2f} <- published")
            out.extend([rmed, float(len(self.rev_breakaways))])
        else:
            out.extend([0.0, 0.0])
        # Fields 12-15: measured steering gain kappa-per-effort, LEFT then
        # RIGHT (median of recent k/s ratios, n) — M2 2026-07-04. Consumed
        # by cmd_vel_to_effort as a per-direction feedforward correction.
        for side, name in ((1, "LEFT"), (-1, "RIGHT")):
            ring = self.steer_pairs[side][-60:]
            if len(ring) >= 15:
                g = sorted(k / s for s, k in ring)[len(ring) // 2]
                self.get_logger().info(
                    f"STEER {name}: n={len(self.steer_pairs[side])} "
                    f"gain={g:.2f} 1/m per effort <- published")
                out.extend([g, float(len(self.steer_pairs[side]))])
            else:
                out.extend([0.0, float(len(self.steer_pairs[side]))])
        self.pub.publish(Float32MultiArray(data=[float(x) for x in out]))


def main(args=None):
    rclpy.init(args=args)
    node = AutoCalib()
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
