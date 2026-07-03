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
import math

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

        self.breakaways = []            # forward breakaway efforts
        self.pairs = {1: [], -1: []}    # steady (u, v) per direction
        self._u = None                  # (t, u) latest effort
        self._still_since = None
        self._win = []                  # rolling (t, u, v, |w|) for steady detect

        self.create_subscription(
            Float32MultiArray, p("effort_topic", "/ackermann/cmd_effort"),
            self._on_effort, 10)
        self.create_subscription(
            Odometry, p("odom_topic", "/odom"), self._on_odom, 10)
        self.pub = self.create_publisher(Float32MultiArray, "/auto_calib/params", 5)
        self.create_timer(self.report_period, self._report)
        self.get_logger().info("observe mode (breakaway + steady pairs)")

    def _on_effort(self, msg):
        if len(msg.data) >= 2:
            self._u = (self.get_clock().now().nanoseconds / 1e9, float(msg.data[1]))

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

        # --- forward breakaway detector ---
        if abs(v) < self.v_still:
            if self._still_since is None:
                self._still_since = t
        elif abs(v) >= self.v_go:
            if (self._still_since is not None
                    and (t - self._still_since) > self.still_min_s
                    and u > self.u_min and v > 0):
                self.breakaways.append(u)
                if len(self.breakaways) > 200:
                    self.breakaways.pop(0)
            self._still_since = None

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
            self.get_logger().info(
                f"FWD breakaway: n={len(b)} median={med:.2f} "
                f"IQR {b[len(b)//4]:.2f}-{b[3*len(b)//4]:.2f} "
                f"(yaml deadband_fwd is what feedforward uses; odom lag "
                f"inflates this ~0.05)")
            out.extend([med, float(len(b))])
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
