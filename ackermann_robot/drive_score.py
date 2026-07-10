"""Session scorecard: turn a drive_logger CSV into a 0-100 driving score.

Usage:
    ros2 run ackermann_robot drive_score [csv_path]

Defaults to logs/drive_log_latest.csv. Prints per-component scores, appends
one history row to logs/scores.csv so progress is trackable across sessions
(the project goal is "gets better as it drives" — this is the ruler).

Components (weights) — each mapped linearly between a target (=1.0) and a
failure value (=0.0), chosen from the 2026-07-02/03 tuning-night baselines:

  starts     (25): forward stall fraction. 0% -> 1, 60% -> 0
                   (measures breakaway seeding + kick quality)
  smooth     (15): lunges per minute (0->0.4 m/s in <0.5 s). 0 -> 1, 3 -> 0
  continuity (15): median movement-episode duration. 8 s -> 1, 1 s -> 0
                   (measures stop-and-go churn: collision halts, replans)
  tracking   (15): median |odom_v - cmd_v| while moving forward.
                   0.05 -> 1, 0.30 -> 0 (feedforward + PI calibration)
  reverse    (15): p90 |odom_v| in reverse. <=0.35 -> 1, 1.0 -> 0
  odometry   (15): phantom-motion fraction at rest. <=2% -> 1, 15% -> 0
"""

import csv
import math
import os
import sys
import time


def _load(path):
    rows = []
    with open(path) as fh:
        for row in csv.DictReader(fh):
            try:
                rows.append({k: (float(v) if v != "nan" else math.nan)
                             for k, v in row.items()})
            except (ValueError, TypeError):
                pass
    return rows


def _median(a):
    s = sorted(a)
    return s[len(s) // 2] if s else float("nan")


def _p90(a):
    s = sorted(a)
    return s[int(0.9 * len(s))] if s else float("nan")


def _lin(x, good, bad):
    """1.0 at good, 0.0 at bad, linear between (works either direction)."""
    if math.isnan(x):
        return None
    if good == bad:
        return 1.0
    t = (x - bad) / (good - bad)
    return max(0.0, min(1.0, t))


def compute(rows):
    nn = lambda x: not math.isnan(x)
    act = [r for r in rows if nn(r["cmd_v"])]
    fwd = [r for r in act if r["cmd_v"] >= 0.1]
    mov = [r for r in fwd if nn(r["odom_v"]) and r["odom_v"] > 0.05]
    stall = [r for r in fwd if nn(r["odom_v"]) and abs(r["odom_v"]) < 0.05]
    rev = [r for r in act if r["cmd_v"] < -0.05]

    m = {}
    m["duration_s"] = rows[-1]["t"] if rows else 0.0
    denom = len(mov) + len(stall)
    m["fwd_stall_pct"] = 100.0 * len(stall) / denom if denom else float("nan")

    lunges = 0
    for i in range(5, len(rows)):
        a, b = rows[i - 5], rows[i]
        if nn(a["odom_v"]) and nn(b["odom_v"]) \
                and abs(a["odom_v"]) < 0.05 and b["odom_v"] > 0.4:
            lunges += 1
    m["lunges_per_min"] = 60.0 * lunges / m["duration_s"] if m["duration_s"] else float("nan")

    segs, cur = [], None
    for r in rows:
        moving = nn(r["odom_v"]) and abs(r["odom_v"]) > 0.05
        if moving:
            cur = [r["t"], r["t"]] if cur is None else [cur[0], r["t"]]
        else:
            if cur and cur[1] - cur[0] > 0.2:
                segs.append(cur[1] - cur[0])
            cur = None
    if cur:
        segs.append(cur[1] - cur[0])
    m["continuity_s"] = _median(segs)

    m["tracking_err"] = _median([abs(r["odom_v"] - r["cmd_v"]) for r in mov]) \
        if mov else float("nan")

    rm = [abs(r["odom_v"]) for r in rev if nn(r["odom_v"]) and r["odom_v"] < -0.05]
    m["rev_p90"] = _p90(rm) if len(rm) >= 10 else float("nan")

    runs, cur = [], []
    for r in rows:
        if nn(r["cmd_v"]) and abs(r["cmd_v"]) <= 0.01:
            cur.append(r)
        else:
            if len(cur) >= 30:
                runs.append(cur)
            cur = []
    if len(cur) >= 30:
        runs.append(cur)
    vs = [r["odom_v"] for run in runs for r in run[15:] if nn(r["odom_v"])]
    m["phantom_pct"] = (100.0 * sum(1 for x in vs if abs(x) > 0.05) / len(vs)) \
        if len(vs) > 20 else float("nan")

    # PROGRESS: actual ground covered per commanded-motion minute, from map
    # pose. Added 2026-07-03 after a hop-fest run scored 59 while barely
    # moving — every other component can look good on a robot vibrating in
    # place; this one can't.
    dist = 0.0
    last = None
    for r in rows:
        if nn(r["map_x"]) and nn(r["map_y"]):
            if last is not None:
                step = math.hypot(r["map_x"] - last[0], r["map_y"] - last[1])
                if step < 0.3:   # ignore SLAM pose jumps
                    dist += step
            last = (r["map_x"], r["map_y"])
    cmd_min = sum(1 for r in act if abs(r["cmd_v"]) > 0.05) / 600.0  # ticks->min
    m["progress_m_per_min"] = dist / cmd_min if cmd_min > 0.05 else float("nan")
    return m


WEIGHTS = [
    ("progress",   25, "progress_m_per_min", 9.0, 0.5),
    ("starts",     20, "fwd_stall_pct",  0.0, 60.0),
    ("smooth",     10, "lunges_per_min", 0.0, 3.0),
    ("continuity", 15, "continuity_s",   8.0, 1.0),
    ("tracking",   10, "tracking_err",   0.05, 0.30),
    ("reverse",    10, "rev_p90",        0.35, 1.00),
    ("odometry",   10, "phantom_pct",    2.0, 15.0),
]


def main():
    ws = os.path.expanduser("~/ros2_ws")
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        ws, "logs", "drive_log_latest.csv")
    path = os.path.realpath(os.path.expanduser(path))
    rows = _load(path)
    if len(rows) < 100:
        print(f"not enough data in {path}")
        return
    m = compute(rows)

    total, wsum = 0.0, 0
    print(f"\n== drive score: {os.path.basename(path)} "
          f"({m['duration_s']:.0f}s) ==")
    for name, w, key, good, bad in WEIGHTS:
        s = _lin(m[key], good, bad)
        if s is None:
            print(f"  {name:<11} --    ({key}: no data)")
            continue
        total += w * s
        wsum += w
        bar = "#" * round(10 * s)
        print(f"  {name:<11} {100*s:3.0f}  |{bar:<10}| {key}={m[key]:.2f}")
    score = 100.0 * total / wsum if wsum else 0.0
    print(f"  {'TOTAL':<11} {score:3.0f} / 100\n")

    hist = os.path.join(ws, "logs", "scores.csv")
    new = not os.path.exists(hist)
    with open(hist, "a") as fh:
        w = csv.writer(fh)
        if new:
            w.writerow(["when", "log", "score"] + [k for _, _, k, _, _ in WEIGHTS])
        w.writerow([time.strftime("%Y-%m-%d %H:%M"), os.path.basename(path),
                    f"{score:.1f}"] +
                   [f"{m[k]:.3f}" for _, _, k, _, _ in WEIGHTS])
    try:
        with open(hist) as fh:
            lines = fh.read().strip().splitlines()[1:]
        if len(lines) > 1:
            print("history:")
            for ln in lines[-6:]:
                c = ln.split(",")
                print(f"  {c[0]}  {c[2]:>5}  {c[1]}")
    except Exception:
        pass


if __name__ == "__main__":
    main()
