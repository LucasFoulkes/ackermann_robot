#!/usr/bin/env python3
"""Log CPU/memory load into the same terminal as drive.launch.py.

Use to see whether SLAM/Nav2 are CPU-bound when scans queue up or TF extrapolates.
"""
from __future__ import annotations

import os
import subprocess
import time

import rclpy
from rclpy.node import Node

# Match ps COMM (15 chars) or full cmdline via second pass below.
_ROS_COMM_HINTS = (
    "slam_toolbox",
    "async_slam",
    "ceres",
    "planner_serv",
    "controller_s",
    "behavior_ser",
    "bt_navigator",
    "lifecycle_ma",
    "rf2o_laser",
    "robot_locali",
    "ekf_node",
    "imu_filter",
    "imu_throttle",
    "rplidar",
    "ackermann",
    "cmd_vel",
)


def _loadavg() -> tuple[float, float, float]:
    with open("/proc/loadavg", encoding="ascii") as f:
        a, b, c, *_ = f.read().split()
    return float(a), float(b), float(c)


def _mem_used_percent() -> float:
    total_kib = avail_kib = 0
    with open("/proc/meminfo", encoding="ascii") as f:
        for line in f:
            if line.startswith("MemTotal:"):
                total_kib = int(line.split()[1])
            elif line.startswith("MemAvailable:"):
                avail_kib = int(line.split()[1])
    if total_kib <= 0:
        return 0.0
    return 100.0 * (1.0 - avail_kib / total_kib)


def _cpu_percent(interval_s: float) -> float:
    def snap() -> tuple[int, int]:
        with open("/proc/stat", encoding="ascii") as f:
            parts = [int(x) for x in f.readline().split()[1:]]
        idle = parts[3] + (parts[4] if len(parts) > 4 else 0)
        return idle, sum(parts)

    idle0, total0 = snap()
    time.sleep(interval_s)
    idle1, total1 = snap()
    dt = total1 - total0
    if dt <= 0:
        return 0.0
    return 100.0 * (1.0 - (idle1 - idle0) / dt)


def _cmdline_label(pid: int) -> str:
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            raw = f.read().replace(b"\x00", b" ").decode(errors="replace").strip()
    except OSError:
        return ""
    for token in ("async_slam_toolbox", "slam_toolbox", "planner_server",
                  "controller_server", "rf2o_laser", "ekf_node", "imu_throttle"):
        if token in raw:
            return token
    if not raw:
        return ""
    return raw.split()[-1][-20:]


def _top_ros_processes() -> str:
    try:
        out = subprocess.check_output(
            ["ps", "-eo", "pcpu,rss,comm,pid", "--sort=-pcpu"],
            text=True,
            timeout=3,
        )
    except (subprocess.SubprocessError, OSError):
        return "ps unavailable"

    hits: list[str] = []
    for line in out.strip().splitlines()[1:]:
        parts = line.split(None, 3)
        if len(parts) < 4:
            continue
        cpu_s, rss_s, comm, pid_s = parts[0], parts[1], parts[2], parts[3]
        label = comm
        if any(h in comm for h in _ROS_COMM_HINTS):
            label = _cmdline_label(int(pid_s)) or comm
        elif "slam" not in comm and "nav2" not in comm and "rf2o" not in comm:
            cl = _cmdline_label(int(pid_s))
            if not cl or not any(
                x in cl for x in ("slam", "planner", "controller", "rf2o", "ekf", "imu")
            ):
                continue
            label = cl
        else:
            label = _cmdline_label(int(pid_s)) or comm
        try:
            cpu = float(cpu_s)
            rss_mb = int(rss_s) / 1024.0
        except ValueError:
            continue
        if cpu < 0.5 and rss_mb < 30:
            continue
        hits.append(f"{label}={cpu:.0f}%/{rss_mb:.0f}MB")
        if len(hits) >= 8:
            break
    return ", ".join(hits) if hits else "no heavy ros procs"


class SystemStats(Node):
    def __init__(self):
        super().__init__("system_stats")
        period = float(self.declare_parameter("period_s", 5.0).value)
        period = max(2.0, period)
        self._cpu_sample = min(1.0, period * 0.2)
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f"system_stats: logging load/cpu/mem every {period:.0f}s "
            "(grep [system_stats] in terminal)"
        )

    def _tick(self):
        la = _loadavg()
        mem = _mem_used_percent()
        cpu = _cpu_percent(self._cpu_sample)
        procs = _top_ros_processes()
        temp_c = ""
        thermal = "/sys/class/thermal/thermal_zone0/temp"
        if os.path.isfile(thermal):
            try:
                with open(thermal, encoding="ascii") as f:
                    temp_c = f", temp={int(f.read()) / 1000:.0f}C"
            except OSError:
                pass
        self.get_logger().info(
            f"[system_stats] load={la[0]:.2f}/{la[1]:.2f}/{la[2]:.2f} "
            f"cpu={cpu:.0f}% mem={mem:.0f}%{temp_c} | {procs}"
        )


def main():
    rclpy.init()
    node = SystemStats()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
