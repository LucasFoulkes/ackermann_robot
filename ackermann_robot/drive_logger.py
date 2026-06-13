#!/usr/bin/env python3
"""Single CSV drive log (default with drive.launch.py).

Logs Nav2 twist, Ackermann-clamped twist, odom, motor effort, turn diagnostics,
and map pose to ~/ros2_ws/logs/drive_log_<stamp>.csv @ 10 Hz.

Turn / path-following columns (filter on w_limited=1 or |w_track_err|>0.15):
  cmd_w          Nav2 requested yaw rate (rad/s)
  w_clamped      after Ackermann kappa cap in cmd_vel_to_effort
  odom_w         measured yaw rate
  kappa_cmd      cmd_w / cmd_v  (path curvature Nav2 asked for)
  kappa_use      w_clamped / cmd_v (curvature we actually command)
  kappa_odom     odom_w / odom_v (curvature robot actually does)
  kappa_max      physical limit (~1.2 1/m)
  w_limited      1 if Nav2 cmd_w was clipped to kappa_max * |v|
  steer_sat      1 if logical steer at full lock (|steer_log| >= 0.98)
  w_track_err    w_clamped - odom_w (command vs measured turn)
  nav2_track_err cmd_w - odom_w (Nav2 vs measured, includes clamp + tracking)
"""
import math
import os
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformListener

NAN = float("nan")
V_MIN = 0.05


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def kappa(w, v):
    if abs(v) < V_MIN:
        return NAN
    return w / v


class DriveLogger(Node):
    def __init__(self):
        super().__init__("drive_logger")
        self.cmd_v = self.cmd_w = NAN
        self.odom_v = self.odom_w = NAN
        self.eff_d = self.eff_s = NAN
        self.w_clamped = self.steer_log = NAN
        self.w_limited = self.steer_sat = 0.0
        self.kappa_max = NAN
        self.trim_sp = self.trim_st = NAN
        self.cpu_prev = self._cpu_read()

        log_dir = os.path.expanduser(
            str(self.declare_parameter("log_dir", "~/ros2_ws/logs").value))
        os.makedirs(log_dir, exist_ok=True)
        path = os.path.join(log_dir, time.strftime("drive_log_%Y%m%d_%H%M%S.csv"))
        self.f = open(path, "w", buffering=1)
        link = os.path.join(log_dir, "drive_log_latest.csv")
        try:
            if os.path.islink(link) or os.path.exists(link):
                os.remove(link)
            os.symlink(path, link)
        except OSError:
            pass
        self.get_logger().info(f"drive log -> {path}")

        self.create_subscription(TwistStamped, "/cmd_vel_nav",
                                 lambda m: self._on_cmd(m.twist), 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        self.create_subscription(Float32MultiArray, "/ackermann/cmd_effort",
                                 self._on_eff, 10)
        self.create_subscription(Float32MultiArray, "/cmd_vel_to_effort/debug",
                                 self._on_dbg, 10)
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

        self.f.write(
            "t,cmd_v,cmd_w,w_clamped,odom_v,odom_w,eff_steer,eff_drive,"
            "kappa_cmd,kappa_use,kappa_odom,kappa_max,w_limited,steer_sat,"
            "w_track_err,nav2_track_err,trim_speed,trim_steer,"
            "map_x,map_y,map_yaw_deg,cpu_pct\n"
        )
        self.t0 = self.get_clock().now().nanoseconds
        hz = max(1.0, float(self.declare_parameter("rate_hz", 10.0).value))
        self.create_timer(1.0 / hz, self._tick)

    def _on_cmd(self, msg: Twist):
        self.cmd_v, self.cmd_w = msg.linear.x, msg.angular.z

    def _on_odom(self, msg: Odometry):
        self.odom_v = msg.twist.twist.linear.x
        self.odom_w = msg.twist.twist.angular.z

    def _on_eff(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            self.eff_s, self.eff_d = msg.data[0], msg.data[1]

    def _on_dbg(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            self.trim_sp, self.trim_st = msg.data[0], msg.data[1]
        if len(msg.data) >= 10:
            self.w_clamped = msg.data[5]
            self.steer_log = msg.data[6]
            self.w_limited = msg.data[7]
            self.steer_sat = msg.data[8]
            self.kappa_max = msg.data[9]

    def _cpu_read(self):
        with open("/proc/stat") as f:
            p = [int(x) for x in f.readline().split()[1:]]
        return sum(p), p[3]

    def cpu_pct(self):
        tot, idle = self._cpu_read()
        dt, di = tot - self.cpu_prev[0], idle - self.cpu_prev[1]
        self.cpu_prev = (tot, idle)
        return 100.0 * (dt - di) / dt if dt > 0 else NAN

    def _tick(self):
        mx = my = myaw = NAN
        try:
            tf = self.tf_buf.lookup_transform("map", "base_link",
                                              rclpy.time.Time())
            mx = tf.transform.translation.x
            my = tf.transform.translation.y
            myaw = math.degrees(yaw_of(tf.transform.rotation))
        except Exception:
            pass

        wc = self.w_clamped if math.isfinite(self.w_clamped) else self.cmd_w
        k_cmd = kappa(self.cmd_w, self.cmd_v)
        k_use = kappa(wc, self.cmd_v)
        k_odom = kappa(self.odom_w, self.odom_v)
        w_err = wc - self.odom_w if math.isfinite(wc) and math.isfinite(self.odom_w) else NAN
        nav2_err = self.cmd_w - self.odom_w if math.isfinite(self.cmd_w) and math.isfinite(self.odom_w) else NAN

        t = (self.get_clock().now().nanoseconds - self.t0) / 1e9
        self.f.write(
            f"{t:.2f},{self.cmd_v:.4f},{self.cmd_w:.4f},{wc:.4f},"
            f"{self.odom_v:.4f},{self.odom_w:.4f},"
            f"{self.eff_s:.4f},{self.eff_d:.4f},"
            f"{k_cmd:.4f},{k_use:.4f},{k_odom:.4f},{self.kappa_max:.4f},"
            f"{int(self.w_limited)},{int(self.steer_sat)},"
            f"{w_err:.4f},{nav2_err:.4f},"
            f"{self.trim_sp:.4f},{self.trim_st:.4f},"
            f"{mx:.4f},{my:.4f},{myaw:.3f},{self.cpu_pct():.1f}\n"
        )


def main():
    rclpy.init()
    node = DriveLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.f.close()


if __name__ == "__main__":
    main()
