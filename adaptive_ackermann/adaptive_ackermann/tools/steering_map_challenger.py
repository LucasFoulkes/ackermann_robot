#!/usr/bin/env python3
"""Stage B: monotone learned steering-map challenger (shadow, offline).

Fits a monotone piecewise-linear effort->curvature map per direction from
eligible observations in TRAINING drive CSVs, seeded by a generic geometry
prior (wheelbase + cautious wheel angle) rather than this chassis's dense
measured map. Evaluates held-out CSVs against the champion (dense map +
fixed lag), which predicts kappa = commanded curvature at the lag-aligned
time; the challenger predicts kappa = f_learned(commanded effort) at the same
time. Lower held-out |error| vs measured curvature wins.

Usage: python3 steering_map_challenger.py [--out PATH]
Reads the train/held-out split from ~/.robot/scorecards/bag_manifest.yaml
by timestamp; any drive_logs CSV not matching a held-out timestamp trains.
"""

import argparse
import bisect
import csv
import glob
import json
import math
import os
import statistics

import numpy as np

WHEELBASE_M = 0.2775
FWD_TIME_LAG_S = 0.235
REV_TIME_LAG_S = 0.168
REV_DISTANCE_LAG_M = 0.033
SPEED_FLOOR_MPS = 0.10
SETTLE_TOL_1PM = 0.10          # command must be settled across the lag window
KNOT_SPACING_EFFORT = 0.06
MIN_BIN_N = 4                  # bins thinner than this lean on the prior

# Generic cautious prior: +-18 deg wheel angle at +-0.86 effort (the old
# +-430 us over a ~500 us half-span), centered on the declared neutral.
# Deliberately NOT this robot's measured map.
PRIOR_CENTER_EFFORT = 0.0
PRIOR_SLOPE_1PM_PER_EFFORT = math.tan(math.radians(18.0)) / WHEELBASE_M / 0.86

HELD_OUT_STAMPS = ('20260712_1359', '20260712_1518', '20260712_1543')


def eligible_samples(path):
    """Yield (direction, effort_lag_aligned, measured_kappa) tuples."""
    try:
        with open(path, errors='replace') as f:
            rows = list(csv.DictReader(
                line.replace('\x00', '') for line in f))
    except Exception:
        return []
    need = {'monotonic_s', 'state', 'fresh_odom', 'odom_outlier',
            'measured_v_mps', 'measured_w_radps', 'steering_effort',
            'accepted_curvature_1pm'}
    if not rows or not need.issubset(rows[0].keys()):
        return []

    def safe(r, key):
        try:
            return float(r.get(key) or 0)
        except ValueError:
            return None

    numeric = ('monotonic_s', 'steering_effort', 'accepted_curvature_1pm',
               'measured_v_mps', 'measured_w_radps')
    rows = [r for r in rows if r.get('state')
            and all(safe(r, k) is not None for k in numeric)]
    if not rows:
        return []
    times = [float(r['monotonic_s']) for r in rows]
    efforts = [float(r['steering_effort'] or 0) for r in rows]
    cmds = [float(r['accepted_curvature_1pm'] or 0) for r in rows]

    def at(series, t):
        i = bisect.bisect_right(times, t) - 1
        return series[i] if i >= 0 else series[0]

    out = []
    for r in rows:
        if r['state'] != 'rolling' or r['fresh_odom'] != '1' \
                or r['odom_outlier'] == '1':
            continue
        v = float(r['measured_v_mps'] or 0)
        if abs(v) < SPEED_FLOOR_MPS:
            continue
        lag = FWD_TIME_LAG_S if v > 0 \
            else REV_TIME_LAG_S + REV_DISTANCE_LAG_M / abs(v)
        t = float(r['monotonic_s'])
        # settled: command curvature stable across the lag window
        if abs(at(cmds, t - lag) - at(cmds, t)) > SETTLE_TOL_1PM:
            continue
        effort = at(efforts, t - lag)
        if abs(effort) > 1.2:
            continue
        km = float(r['measured_w_radps'] or 0) / v
        if abs(km) > 2.5:
            continue
        out.append(('fwd' if v > 0 else 'rev', effort, km))
    return out


def pava_monotone(y, w):
    """Pool-adjacent-violators: weighted isotonic (non-decreasing) fit."""
    y = list(y)
    w = list(w)
    blocks = [[y[i], w[i], i, i] for i in range(len(y))]
    merged = []
    for b in blocks:
        merged.append(b)
        while len(merged) > 1 and merged[-2][0] > merged[-1][0]:
            b2 = merged.pop()
            b1 = merged.pop()
            tw = b1[1] + b2[1]
            merged.append([(b1[0] * b1[1] + b2[0] * b2[1]) / tw,
                           tw, b1[2], b2[3]])
    out = [0.0] * len(y)
    for val, _, i0, i1 in merged:
        for i in range(i0, i1 + 1):
            out[i] = val
    return out


def fit_direction(samples):
    """Monotone PWL knots from samples, prior-backed in thin regions."""
    if not samples:
        return None
    efforts = np.array([s[1] for s in samples])
    kappas = np.array([s[2] for s in samples])
    lo = math.floor(efforts.min() / KNOT_SPACING_EFFORT) * KNOT_SPACING_EFFORT
    hi = math.ceil(efforts.max() / KNOT_SPACING_EFFORT) * KNOT_SPACING_EFFORT
    knots = np.arange(lo, hi + KNOT_SPACING_EFFORT, KNOT_SPACING_EFFORT)
    means, weights = [], []
    for k in knots:
        m = np.abs(efforts - k) <= KNOT_SPACING_EFFORT
        n = int(m.sum())
        prior = (k - PRIOR_CENTER_EFFORT) * PRIOR_SLOPE_1PM_PER_EFFORT
        if n >= MIN_BIN_N:
            # robust bin center: median, prior only as a vanishing nudge
            means.append((statistics.median(kappas[m]) * n + prior * 2)
                         / (n + 2))
            weights.append(n)
        else:
            means.append(prior)
            weights.append(1)
    fitted = pava_monotone(means, weights)
    return {
        'knots_effort': [round(float(k), 4) for k in knots],
        'kappa_1pm': [round(float(v), 4) for v in fitted],
        'samples_per_knot': [int(w) if w > 1 else 0 for w in weights],
        'samples': len(samples),
        'bins_with_data': int(sum(1 for w in weights if w > 1)),
    }


def predict(model, effort):
    k = model['knots_effort']
    v = model['kappa_1pm']
    if effort <= k[0]:
        return v[0]
    if effort >= k[-1]:
        return v[-1]
    i = bisect.bisect_right(k, effort) - 1
    f = (effort - k[i]) / (k[i + 1] - k[i])
    return v[i] + f * (v[i + 1] - v[i])


def evaluate(path, models):
    """Champion vs challenger abs errors on one held-out CSV."""
    rows = list(csv.DictReader(open(path)))
    if not rows or 'steering_effort' not in rows[0]:
        return [], [], 0, 0   # pre-effort-era CSV: not comparable
    times = [float(r['monotonic_s']) for r in rows]
    efforts = [float(r['steering_effort'] or 0) for r in rows]
    cmds = [float(r['accepted_curvature_1pm'] or 0) for r in rows]

    def at(series, t):
        i = bisect.bisect_right(times, t) - 1
        return series[i] if i >= 0 else series[0]

    champ, chall = [], []
    sign_bad = 0
    sign_n = 0
    for r in rows:
        if r['state'] != 'rolling' or r['fresh_odom'] != '1' \
                or r['odom_outlier'] == '1':
            continue
        v = float(r['measured_v_mps'] or 0)
        if abs(v) < SPEED_FLOOR_MPS:
            continue
        d = 'fwd' if v > 0 else 'rev'
        if models.get(d) is None:
            continue
        lag = FWD_TIME_LAG_S if v > 0 \
            else REV_TIME_LAG_S + REV_DISTANCE_LAG_M / abs(v)
        t = float(r['monotonic_s'])
        if abs(at(cmds, t - lag) - at(cmds, t)) > SETTLE_TOL_1PM:
            continue
        km = float(r['measured_w_radps'] or 0) / v
        if abs(km) > 2.5:
            continue
        kc = at(cmds, t - lag)
        kp = predict(models[d], at(efforts, t - lag))
        champ.append(abs(km - kc))
        chall.append(abs(km - kp))
        if abs(km) > 0.3:
            sign_n += 1
            if kp * km < 0:
                sign_bad += 1
    return champ, chall, sign_bad, sign_n


def write_bootstrap_map():
    """Amnesia/second-vehicle boot: a learned-memory file from the generic
    geometric prior alone (zero samples). The controller treats an all-prior
    map as BOOTSTRAP authority: reduced trust region, cautious driving,
    replaced by evidence-backed refits as ordinary sessions accumulate."""
    import yaml
    knots = [round(-0.9 + 0.06 * i, 4) for i in range(31)]
    memory = {
        'schema_version': 1,
        'source': 'geometric_prior_bootstrap',
        'training_files': 0,
        'directions': {},
    }
    for direction in ('forward', 'reverse'):
        memory['directions'][direction] = {
            'knots_effort': knots,
            'kappa_1pm': [round((k - PRIOR_CENTER_EFFORT)
                                * PRIOR_SLOPE_1PM_PER_EFFORT, 4) for k in knots],
            'samples_per_knot': [0] * len(knots),
            'total_samples': 0,
        }
    path = os.path.expanduser('~/.robot/learned_steering_map.yaml')
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        yaml.safe_dump(memory, f, default_flow_style=None)
    os.replace(tmp, path)
    print(f'bootstrap (prior-only) map -> {path}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out',
                    default=os.path.expanduser(
                        '~/.robot/scorecards/steering_challenger_v1.json'))
    ap.add_argument('--bootstrap', action='store_true',
                    help='write a prior-only map and exit (amnesia boot)')
    ap.add_argument('--since', default=None,
                    help='use only drive CSVs newer than this stamp '
                         '(YYYYMMDD_HHMMSS) — REQUIRED for an honest '
                         'amnesia-era refit so pre-wipe data cannot leak in')
    args = ap.parse_args()
    if args.bootstrap:
        write_bootstrap_map()
        return

    logs = sorted(glob.glob(os.path.expanduser('~/.robot/drive_logs/*.csv')))
    if args.since:
        cutoff = f'adaptive_drive_{args.since}'
        logs = [p for p in logs if os.path.basename(p) >= cutoff]
        print(f'--since {args.since}: {len(logs)} eligible CSVs')
    train = [p for p in logs
             if not any(s in os.path.basename(p) for s in HELD_OUT_STAMPS)]
    held = [p for p in logs
            if any(s in os.path.basename(p) for s in HELD_OUT_STAMPS)]

    samples = {'fwd': [], 'rev': []}
    used_files = 0
    for p in train:
        got = eligible_samples(p)
        if got:
            used_files += 1
        for d, effort, km in got:
            samples[d].append((d, effort, km))
    print(f'training files with eligible samples: {used_files}/{len(train)}')
    print(f"samples: fwd={len(samples['fwd'])} rev={len(samples['rev'])}")

    models = {d: fit_direction(s) for d, s in samples.items()}
    for d, m in models.items():
        if m:
            print(f"{d}: {len(m['knots_effort'])} knots over "
                  f"{m['knots_effort'][0]:.2f}..{m['knots_effort'][-1]:.2f} "
                  f"effort, {m['bins_with_data']} data-backed bins")

    result = {'models': models, 'held_out': {}}
    print('\nheld-out evaluation (median / p90 abs curvature error, 1/m):')
    for p in held:
        champ, chall, sbad, sn = evaluate(p, models)
        if not champ:
            continue
        champ.sort()
        chall.sort()

        def q(a, f):
            return a[min(len(a) - 1, int(f * len(a)))]
        name = os.path.basename(p)
        result['held_out'][name] = {
            'n': len(champ),
            'champion_median': round(statistics.median(champ), 4),
            'champion_p90': round(q(champ, 0.9), 4),
            'challenger_median': round(statistics.median(chall), 4),
            'challenger_p90': round(q(chall, 0.9), 4),
            'challenger_wrong_sign': f'{sbad}/{sn}',
        }
        r = result['held_out'][name]
        print(f"  {name}: n={r['n']}  champion {r['champion_median']:.3f}/"
              f"{r['champion_p90']:.3f}  challenger "
              f"{r['challenger_median']:.3f}/{r['challenger_p90']:.3f}  "
              f"wrong-sign {r['challenger_wrong_sign']}")

    with open(args.out, 'w') as f:
        json.dump(result, f, indent=2)
    print(f'\nmodel + eval -> {args.out}')

    # Learned-memory export for the controller (Stage D). Direction names
    # match the controller's forward/reverse convention.
    memory = {
        'schema_version': 1,
        'source': 'steering_map_challenger_v1',
        'training_files': used_files,
        'directions': {},
    }
    for src, dst in (('fwd', 'forward'), ('rev', 'reverse')):
        m = models.get(src)
        if m:
            memory['directions'][dst] = {
                'knots_effort': m['knots_effort'],
                'kappa_1pm': m['kappa_1pm'],
                'samples_per_knot': m['samples_per_knot'],
                'total_samples': m['samples'],
            }
    mem_path = os.path.expanduser('~/.robot/learned_steering_map.yaml')
    import yaml
    tmp = mem_path + '.tmp'
    with open(tmp, 'w') as f:
        yaml.safe_dump(memory, f, default_flow_style=None)
    os.replace(tmp, mem_path)
    print(f'learned memory -> {mem_path}')


if __name__ == '__main__':
    main()
