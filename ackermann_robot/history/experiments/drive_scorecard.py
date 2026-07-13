#!/usr/bin/env python3
"""Reproducible drive scorecard (masterplan Stage A).

Scores one 50 Hz adaptive-controller drive CSV against the champion actuation
model: dense steering maps + fixed direction-specific response model
(forward: pure time lag; reverse: time lag + rolling-distance term, measured
2026-07-11). Emits a JSON scorecard plus a human-readable table so learned
challengers can be compared on identical inputs.

Usage:
  python3 drive_scorecard.py <drive_log.csv> [--json-out PATH]

Eligibility mirrors the learning gates: rolling state, fresh non-outlier
odometry, speed above the curvature-observation floor.
"""

import argparse
import bisect
import csv
import json
import math
import os
import statistics
import sys

# Champion steering response model (measured 2026-07-11).
FWD_TIME_LAG_S = 0.235
REV_TIME_LAG_S = 0.168
REV_DISTANCE_LAG_M = 0.033

SPEED_FLOOR_MPS = 0.10
CURVATURE_ACTIVE_1PM = 0.15
SATURATION_1PM = 1.14
LAUNCH_PEAK_LIMIT_MPS = 0.40


def load_rows(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit(f'no rows in {path}')
    return rows


def f(row, key):
    v = row.get(key, '')
    return float(v) if v not in ('', None) else 0.0


def branch_name(direction_fwd, curvature):
    side = 'left' if curvature > 0 else 'right'
    return f"{'fwd' if direction_fwd else 'rev'}_{side}"


def score(path):
    rows = load_rows(path)
    t0 = f(rows[0], 'monotonic_s')
    times = [f(r, 'monotonic_s') for r in rows]
    cmd_curv = [f(r, 'accepted_curvature_1pm') for r in rows]

    def command_at(t):
        i = bisect.bisect_right(times, t) - 1
        return cmd_curv[i] if i >= 0 else 0.0

    steering = {}
    launch_peak = 0.0
    over_peak_rows = 0
    stall_rows = 0
    active_rows = 0
    sat_rows = 0
    curv_rows = 0
    recovery_eps = 0
    recovery_s = 0.0
    prev_state = None
    prev_t = t0

    for r in rows:
        t = f(r, 'monotonic_s')
        state = r['state']
        v = f(r, 'measured_v_mps')
        w = f(r, 'measured_w_radps')
        kc_now = f(r, 'accepted_curvature_1pm')

        if state == 'recovery':
            recovery_s += t - prev_t
            if prev_state != 'recovery':
                recovery_eps += 1
        prev_state, prev_t = state, t

        if abs(f(r, 'accepted_v_mps')) > 0.01:
            active_rows += 1
            if abs(f(r, 'raw_v_mps')) < 0.03:
                stall_rows += 1
        if abs(kc_now) > 0.01:
            curv_rows += 1
            if abs(kc_now) >= SATURATION_1PM:
                sat_rows += 1
        if state == 'rolling' and r.get('odom_outlier') != '1':
            if abs(v) > launch_peak:
                launch_peak = abs(v)
            if abs(v) > LAUNCH_PEAK_LIMIT_MPS:
                over_peak_rows += 1

        # Steering prediction eligibility.
        if state != 'rolling' or r.get('fresh_odom') != '1' \
                or r.get('odom_outlier') == '1' or abs(v) < SPEED_FLOOR_MPS:
            continue
        fwd = v > 0
        delay = FWD_TIME_LAG_S if fwd \
            else REV_TIME_LAG_S + REV_DISTANCE_LAG_M / abs(v)
        kc = command_at(t - delay)
        if abs(kc) < 0.01 and abs(kc_now) > 0.3:
            continue  # command just changed hugely; alignment unreliable
        km = w / v
        b = steering.setdefault(
            branch_name(fwd, kc if abs(kc) > 0.05 else km),
            {'err': [], 'wrong': 0, 'n_active': 0})
        b['err'].append(abs(km - kc))
        if abs(kc) > CURVATURE_ACTIVE_1PM:
            b['n_active'] += 1
            if km * kc < 0 and abs(km) > 0.1:
                b['wrong'] += 1

    out = {
        'csv': os.path.basename(path),
        'duration_s': round(times[-1] - t0, 1),
        'steering_prediction': {},
        'throttle': {
            'launch_peak_mps': round(launch_peak, 3),
            'rows_over_0p40': over_peak_rows,
            'stall_fraction': round(stall_rows / max(1, active_rows), 3),
            'recovery_episodes': recovery_eps,
            'recovery_total_s': round(recovery_s, 1),
        },
        'curvature_saturation_fraction': round(sat_rows / max(1, curv_rows), 3),
        'delay_estimator': {
            'start_s': f(rows[0], 'estimated_steering_delay_s'),
            'end_s': f(rows[-1], 'estimated_steering_delay_s'),
            'start_confidence': f(rows[0], 'steering_delay_confidence'),
            'end_confidence': f(rows[-1], 'steering_delay_confidence'),
        },
    }
    all_err = []
    for name, b in sorted(steering.items()):
        errs = b['err']
        all_err.extend(errs)
        errs.sort()
        out['steering_prediction'][name] = {
            'samples': len(errs),
            'median_abs_err_1pm': round(statistics.median(errs), 4),
            'p90_abs_err_1pm': round(errs[int(0.9 * len(errs))], 4)
            if errs else None,
            'wrong_sign_rate': round(b['wrong'] / max(1, b['n_active']), 3),
        }
    if all_err:
        all_err.sort()
        out['steering_prediction']['overall'] = {
            'samples': len(all_err),
            'median_abs_err_1pm': round(statistics.median(all_err), 4),
            'p90_abs_err_1pm': round(all_err[int(0.9 * len(all_err))], 4),
        }
    return out


def dynamics_split(path):
    """Gradual-state vs fixed-lag prediction, split transient/settled.

    Requires the effective_kappa_1pm column (Stage C shadow/applied) and the
    learned steering map on disk; returns None when unavailable.
    """
    import yaml
    map_path = os.path.expanduser('~/.robot/learned_steering_map.yaml')
    if not os.path.exists(map_path):
        return None
    rows = load_rows(path)
    if 'effective_kappa_1pm' not in rows[0]:
        return None
    memory = yaml.safe_load(open(map_path))

    def make(direction):
        entry = memory['directions'][direction]
        pts = [(p, k) for p, k, n in zip(entry['knots_pulse_us'],
                                         entry['kappa_1pm'],
                                         entry['samples_per_knot']) if n >= 30]

        def fn(x):
            if x <= pts[0][0]:
                return pts[0][1]
            if x >= pts[-1][0]:
                return pts[-1][1]
            i = bisect.bisect_right([p for p, _ in pts], x) - 1
            (x0, y0), (x1, y1) = pts[i], pts[i + 1]
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
        return fn

    maps = {'forward': make('forward'), 'reverse': make('reverse')}
    times = [f(r, 'monotonic_s') for r in rows]
    pulses = [f(r, 'steering_us') for r in rows]
    cmds = [f(r, 'accepted_curvature_1pm') for r in rows]

    def at(series, t):
        i = bisect.bisect_right(times, t) - 1
        return series[i] if i >= 0 else series[0]

    out = {}
    for i, r in enumerate(rows):
        if r['state'] != 'rolling' or r.get('fresh_odom') != '1' \
                or r.get('odom_outlier') == '1':
            continue
        v = f(r, 'measured_v_mps')
        if abs(v) < 0.12:
            continue
        measured = f(r, 'measured_w_radps') / v
        if abs(measured) > 2.5:
            continue
        d = 'forward' if v > 0 else 'reverse'
        lag = FWD_TIME_LAG_S if v > 0 \
            else REV_TIME_LAG_S + REV_DISTANCE_LAG_M / abs(v)
        gradual = abs(measured - f(r, 'effective_kappa_1pm'))
        fixed = abs(measured - maps[d](at(pulses, times[i] - lag)))
        key = 'transient' if abs(at(cmds, times[i])
                                 - at(cmds, times[i] - 0.7)) > 0.12 \
            else 'settled'
        out.setdefault(key, ([], []))
        out[key][0].append(gradual)
        out[key][1].append(fixed)
    return {k: {'n': len(g),
                'gradual_median': round(statistics.median(g), 4),
                'fixed_median': round(statistics.median(x), 4)}
            for k, (g, x) in out.items() if g} or None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv_path')
    ap.add_argument('--json-out')
    args = ap.parse_args()
    card = score(args.csv_path)
    dyn = dynamics_split(args.csv_path)
    if dyn:
        card['dynamics_split'] = dyn

    print(f"\n=== drive scorecard: {card['csv']} "
          f"({card['duration_s']} s) ===")
    print('steering prediction (champion dense map + fixed lag):')
    for name, s in card['steering_prediction'].items():
        line = (f"  {name:10s} n={s['samples']:5d} "
                f"median={s['median_abs_err_1pm']:.3f} "
                f"p90={s['p90_abs_err_1pm']:.3f} 1/m")
        if 'wrong_sign_rate' in s:
            line += f"  wrong-sign={s['wrong_sign_rate']:.1%}"
        print(line)
    t = card['throttle']
    print(f"throttle: launch_peak={t['launch_peak_mps']} m/s  "
          f"rows>0.40={t['rows_over_0p40']}  stall={t['stall_fraction']:.1%}  "
          f"recovery {t['recovery_episodes']}x/{t['recovery_total_s']} s")
    print(f"curvature saturation: "
          f"{card['curvature_saturation_fraction']:.1%}")
    d = card['delay_estimator']
    print(f"delay estimator: {d['start_s']:.3f}s(conf {d['start_confidence']:.2f})"
          f" -> {d['end_s']:.3f}s(conf {d['end_confidence']:.2f})")
    if dyn:
        print('dynamics (gradual vs fixed-lag prediction):')
        for regime, s in dyn.items():
            print(f"  {regime:9s} n={s['n']:4d}  gradual "
                  f"{s['gradual_median']:.4f}  fixed {s['fixed_median']:.4f}")

    if args.json_out:
        with open(args.json_out, 'w') as fh:
            json.dump(card, fh, indent=2)
        print(f'json -> {args.json_out}')


if __name__ == '__main__':
    main()
