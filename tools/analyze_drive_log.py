#!/usr/bin/env python3
"""Standard drive-log analysis report (numpy, no pandas needed).

Usage:
  python3 tools/analyze_drive_log.py                 # latest drive log
  python3 tools/analyze_drive_log.py <drive_log.csv>
  python3 tools/analyze_drive_log.py --compare <other.csv>   # A/B two runs

Sections: health, steering transfer (steady-state, sign-stable — immune to the
RPP-weave artifact that polluted ad-hoc analyses), stop/go duty cycle, CPU,
and console-log correlation (collision-ahead / planner failures) when the
matching ~/.ros/log files are present.
"""
import argparse
import csv
import glob
import math
import os
import re
import sys

import numpy as np

LOGDIR = os.path.expanduser("~/ros2_ws/logs")
ROSLOG = os.path.expanduser("~/.ros/log")


def load(path):
    rows = list(csv.DictReader(open(path)))
    if not rows:
        sys.exit(f"empty csv: {path}")
    cols = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
    return cols


def stable_mask(sig, thresh, win):
    """True where sig has held one sign (beyond thresh) for >= win samples."""
    s = np.sign(np.where(np.abs(sig) > thresh, sig, 0))
    m = np.zeros(len(s), bool)
    run = 0
    for i in range(len(s)):
        run = run + 1 if s[i] != 0 and (i == 0 or s[i] == s[i - 1]) else (1 if s[i] != 0 else 0)
        m[i] = run >= win
    return m, s


def steering_report(d):
    t, cv, ov, ow, es = d["t"], d["cmd_v"], d["odom_v"], d["odom_w"], d["eff_steer"]
    dt = np.median(np.diff(t)) or 0.1
    win = max(2, int(round(0.8 / dt)))
    mv, sv = stable_mask(cv, 0.03, win)
    ms, ss = stable_mask(es, 0.10, win)
    fwd = mv & ms & (sv > 0) & (ov > 0.15)
    kappa = np.where(np.abs(ov) > 0.05, ow / np.maximum(np.abs(ov), 1e-6) * np.sign(ov), np.nan)
    print("\n== STEERING TRANSFER (steady fwd, steer-sign stable 0.8s) ==")
    print("  band       sign    n   median_k  p25     p75   wrong-sign")
    for lo, hi, name in [(0.95, 1.01, "full-lock"), (0.4, 0.95, "mid"), (0.10, 0.4, "light")]:
        for sgn in (1, -1):
            sel = fwd & (np.abs(es) >= lo) & (np.abs(es) < hi) & (ss == sgn) & ~np.isnan(kappa)
            if sel.sum() >= 8:
                k = kappa[sel]
                wrong = (k * sgn < -0.05).mean()
                print(f"  {name:10} {'+' if sgn > 0 else '-':>3} {sel.sum():>5}"
                      f"   {np.median(k):+6.2f}  {np.percentile(k,25):+6.2f} {np.percentile(k,75):+6.2f}"
                      f"   {100*wrong:3.0f}%")
    # left/right symmetry verdict at full lock
    kl = kappa[fwd & (es >= 0.95) & ~np.isnan(kappa)]
    kr = kappa[fwd & (es <= -0.95) & ~np.isnan(kappa)]
    if len(kl) >= 8 and len(kr) >= 8:
        l, r = np.median(kl), -np.median(kr)
        print(f"  full-lock symmetry: left k={l:+.2f} vs right k={r:+.2f}"
              f"  ({'OK' if min(l, r) > 0.6 * max(l, r) and min(l, r) > 0 else 'ASYMMETRIC — check steering_center_tick / linkage'})")


def stop_go_report(d):
    t, ov = d["t"], d["odom_v"]
    moving = np.abs(ov) > 0.05
    stops, runs = [], []
    i = 0
    while i < len(moving):
        j = i
        while j < len(moving) and moving[j] == moving[i]:
            j += 1
        (runs if moving[i] else stops).append((t[i], t[j - 1] - t[i]))
        i = j
    drive = sum(x for _, x in runs)
    stop = sum(x for _, x in stops)
    print(f"\n== STOP/GO ==  driving {drive:.0f}s vs stopped {stop:.0f}s"
          f"  (duty {100*drive/max(1e-9, drive+stop):.0f}%)")
    if runs:
        print(f"  go-segments: {len(runs)}, median {np.median([x for _, x in runs]):.1f}s")
    long_stops = [(s, x) for s, x in stops if x >= 2.0 and s > 5]
    if long_stops:
        print(f"  stops >=2s: {len(long_stops)}: "
              + ", ".join(f"t={s:.0f}({x:.0f}s)" for s, x in long_stops[:14]))
    return long_stops


def cpu_report(d):
    c = d.get("cpu_pct")
    if c is None:
        return
    c = c[~np.isnan(c) & (c > 0)]
    if len(c):
        print(f"\n== CPU ==  median {np.median(c):.0f}%  p90 {np.percentile(c,90):.0f}%  max {c.max():.0f}%")


def console_correlation(csv_path, long_stops):
    """Find this run's controller/planner logs via the drive_logger T0 line."""
    t0 = None
    for f in sorted(glob.glob(os.path.join(ROSLOG, "python3_*.log")),
                    key=os.path.getmtime, reverse=True)[:30]:
        for line in open(f, errors="ignore"):
            if "drive_logger" in line and os.path.basename(csv_path) in line:
                m = re.match(r"\[INFO\] \[(\d+\.\d+)\]", line)
                if m:
                    t0 = float(m.group(1))
                break
        if t0:
            break
    if t0 is None:
        print("\n(console correlation skipped: no matching drive_logger log line found)")
        return
    print(f"\n== CONSOLE CORRELATION (T0={t0:.0f}) ==")
    for pat, label in [("controller_server_*.log", "collision ahead!"),
                       ("planner_server_*.log", "no valid path found"),
                       ("planner_server_*.log", "Start occupied")]:
        hits = []
        for f in sorted(glob.glob(os.path.join(ROSLOG, pat)),
                        key=os.path.getmtime, reverse=True)[:1]:
            for line in open(f, errors="ignore"):
                if label in line:
                    m = re.match(r"\[WARN\] \[(\d+\.\d+)\]", line)
                    if m:
                        hits.append(float(m.group(1)) - t0)
        in_run = [h for h in hits if h >= 0]
        print(f"  {label}: {len(in_run)}")
        # which long stops have a console hit just before/within them?
        explained = sum(1 for s, dur in long_stops
                        if any(s - 2 <= h <= s + dur for h in in_run))
        if long_stops and in_run:
            print(f"    explains {explained}/{len(long_stops)} of the >=2s stops")


def report(path):
    print(f"### {path}")
    d = load(path)
    print(f"rows {len(d['t'])}, duration {d['t'][-1]:.0f}s, "
          f"moving {(np.abs(d['odom_v']) > 0.05).mean()*100:.0f}% of samples")
    # sanity: cmd vs effort sign mismatch (throttle direction bug detector)
    cv, ed = d["cmd_v"], d["eff_drive"]
    bad = ((np.abs(cv) > 0.03) & ~np.isnan(ed) & (cv * ed < -0.01)).sum()
    print(f"cmd/effort sign mismatches: {bad} (should be ~0)")
    steering_report(d)
    stops = stop_go_report(d)
    cpu_report(d)
    console_correlation(path, stops)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?",
                    default=os.path.join(LOGDIR, "drive_log_latest.csv"))
    ap.add_argument("--compare", help="second csv to print side by side")
    args = ap.parse_args()
    report(os.path.realpath(args.csv))
    if args.compare:
        print("\n" + "=" * 60)
        report(os.path.realpath(args.compare))


if __name__ == "__main__":
    main()
