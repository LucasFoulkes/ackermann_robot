#!/usr/bin/env python3
"""CSV telemetry logger, one timestamped file per run (started by drive.launch).

Columns: t, cmd_v, cmd_w (twist we send), odom_v, odom_w (twist from /odom),
eff_drive, eff_steer (what reaches the motors), cpu_pct, trim_speed,
trim_steer (PI corrections), map_x, map_y, map_yaw_deg (map->base_link).

Writes to <log_dir>/drive_log_<stamp>.csv at 10 Hz and updates the
drive_log_latest.csv symlink. log_dir default ~/ros2_ws/logs (gitignored).
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


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class DriveLogger(Node):
    def __init__(self):
        super().__init__("drive_logger")
        self.cmd_v = self.cmd_w = NAN
        self.odom_v = self.odom_w = NAN
        self.eff_d = self.eff_s = NAN
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
        self.get_logger().info(f"logging to {path}")

        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(TwistStamped, "/cmd_vel_nav",
                                 lambda m: self.on_cmd(m.twist), 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 20)
        self.create_subscription(Float32MultiArray, "/ackermann/cmd_effort",
                                 self.on_eff, 10)
        self.create_subscription(Float32MultiArray, "/cmd_vel_to_effort/debug",
                                 self.on_dbg, 10)
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

        self.f.write("t,cmd_v,cmd_w,odom_v,odom_w,eff_drive,eff_steer,"
                     "cpu_pct,trim_speed,trim_steer,map_x,map_y,map_yaw_deg\n")
        self.t0 = self.get_clock().now().nanoseconds
        self.create_timer(0.1, self.tick)

    def on_cmd(self, m):
        self.cmd_v, self.cmd_w = m.linear.x, m.angular.z

    def on_odom(self, m):
        self.odom_v = m.twist.twist.linear.x
        self.odom_w = m.twist.twist.angular.z

    def on_eff(self, m):
        if len(m.data) >= 2:  # /ackermann/cmd_effort is [steer, throttle]
            self.eff_s, self.eff_d = m.data[0], m.data[1]

    def on_dbg(self, m):
        if len(m.data) >= 2:
            self.trim_sp, self.trim_st = m.data[0], m.data[1]

    def _cpu_read(self):
        with open("/proc/stat") as f:
            p = [int(x) for x in f.readline().split()[1:]]
        return sum(p), p[3]  # total, idle

    def cpu_pct(self):
        tot, idle = self._cpu_read()
        dt, di = tot - self.cpu_prev[0], idle - self.cpu_prev[1]
        self.cpu_prev = (tot, idle)
        return 100.0 * (dt - di) / dt if dt > 0 else NAN

    def tick(self):
        mx = my = myaw = NAN
        try:
            tf = self.tf_buf.lookup_transform("map", "base_link",
                                              rclpy.time.Time())
            mx = tf.transform.translation.x
            my = tf.transform.translation.y
            myaw = math.degrees(yaw_of(tf.transform.rotation))
        except Exception:
            pass
        t = (self.get_clock().now().nanoseconds - self.t0) / 1e9
        self.f.write(f"{t:.2f},{self.cmd_v:.4f},{self.cmd_w:.4f},"
                     f"{self.odom_v:.4f},{self.odom_w:.4f},"
                     f"{self.eff_d:.4f},{self.eff_s:.4f},{self.cpu_pct():.1f},"
                     f"{self.trim_sp:.4f},{self.trim_st:.4f},"
                     f"{mx:.4f},{my:.4f},{myaw:.3f}\n")


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
