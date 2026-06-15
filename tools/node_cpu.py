#!/usr/bin/env python3
"""Per-node CPU snapshot for the running stack (no ROS discovery, just /proc).

Run while the stack is up:
  python3 tools/node_cpu.py            # 2 s sample
  python3 tools/node_cpu.py 5          # 5 s sample

Samples each process's utime+stime from /proc/<pid>/stat twice over the interval
and reports %CPU (100% = one full core). Groups the lightweight logger/monitor
nodes and subtotals them, so you can see exactly what trimming them would buy.
"""
import glob
import os
import sys
import time

HZ = os.sysconf("SC_CLK_TCK")
NCPU = os.cpu_count() or 4
LOGGERS = ("drive_logger", "twist_logger", "tf_health", "system_stats",
           "comms_monitor", "stuck_monitor", "scan_throttle")


def node_name(pid):
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            parts = f.read().split(b"\x00")
    except OSError:
        return None
    cl = [p.decode(errors="replace") for p in parts if p]
    if not cl:
        return None
    txt = " ".join(cl)
    if "ros" not in txt and "ackermann" not in txt and "rclcpp" not in txt:
        # keep only things that look like ROS nodes
        if not any(k in txt for k in ("slam", "realsense", "nav2", "ekf", "rtabmap",
                                      "laser_filt", "robot_state", "component")):
            return None
    # node name from -r __node:=NAME if present
    for i, tok in enumerate(cl):
        if tok == "__node:=" and i + 1 < len(cl):
            return cl[i + 1]
        if tok.startswith("__node:="):
            return tok.split(":=", 1)[1]
    # else the executable basename
    for tok in cl:
        if "/lib/" in tok and os.path.basename(tok):
            return os.path.basename(tok)
    base = os.path.basename(cl[0])
    return base if base not in ("python3", "bash", "/bin/bash") else (
        os.path.basename(cl[1]) if len(cl) > 1 else base)


def jiffies(pid):
    try:
        with open(f"/proc/{pid}/stat") as f:
            f2 = f.read().rsplit(")", 1)[1].split()
        return int(f2[11]) + int(f2[12])  # utime + stime (fields 14,15)
    except (OSError, IndexError, ValueError):
        return None


def main():
    interval = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    pids = [int(os.path.basename(p)) for p in glob.glob("/proc/[0-9]*")]
    names, t0 = {}, {}
    for pid in pids:
        n = node_name(pid)
        if n:
            j = jiffies(pid)
            if j is not None:
                names[pid], t0[pid] = n, j
    time.sleep(interval)
    rows = []
    for pid, n in names.items():
        j1 = jiffies(pid)
        if j1 is None:
            continue
        pct = 100.0 * (j1 - t0[pid]) / HZ / interval
        rows.append((pct, n, pid))
    rows.sort(reverse=True)
    print(f"{'%CPU':>6}  node  (cores={NCPU}, 100%=1 core)")
    log_total = 0.0
    for pct, n, pid in rows:
        if pct < 0.3:
            continue
        tag = "  [logger]" if any(l in n for l in LOGGERS) else ""
        if tag:
            log_total += pct
        print(f"{pct:6.1f}  {n}{tag}")
    print(f"\nloggers/monitors subtotal: {log_total:.1f}% of one core "
          f"({log_total/NCPU:.1f}% of total {NCPU}-core capacity)")
    with open("/proc/loadavg") as f:
        print(f"loadavg: {f.read().split()[0]} (>{NCPU} = oversubscribed)")


if __name__ == "__main__":
    main()
