#!/usr/bin/env python3
"""Stage C: gradual steering-state challenger vs fixed-delay champion.

Models effective curvature as a first-order response toward the target
curvature (from the Stage B learned monotone map), with an optional
rolling-distance term in the time constant:

    tau_eff = tau + dist_const / |v|
    eff += (target - eff) * (1 - exp(-dt / tau_eff))

Fits (tau, dist_const) per direction on TRAINING CSVs by grid search, then
compares held-out transient prediction against the champion pure-delay model
(fwd 0.235 s; rev 0.168 s + 0.033 m). Evaluation uses ALL eligible rolling
samples, not just settled ones — transients are exactly where the models
differ.
"""

import bisect
import csv
import glob
import json
import math
import os
import statistics

import numpy as np

FWD_TIME_LAG_S = 0.235
REV_TIME_LAG_S = 0.168
REV_DISTANCE_LAG_M = 0.033
SPEED_FLOOR_MPS = 0.10
HELD_OUT_STAMPS = ('20260712_1359', '20260712_1518', '20260712_1543')

MAP_PATH = os.path.expanduser(
    '~/.robot/scorecards/steering_challenger_v1.json')
OUT_PATH = os.path.expanduser(
    '~/.robot/scorecards/steering_dynamics_v1.json')

TAUS = [0.05, 0.08, 0.12, 0.16, 0.20, 0.25, 0.30, 0.40, 0.50, 0.65]
DISTS = [0.0, 0.01, 0.02, 0.03, 0.05]
# A sample is a TRANSIENT if the map target moved by more than this within
# the trailing window; the fit objective balances transient and settled
# medians so a long tau cannot buy transient wins by over-smoothing the
# steady state (session-10 live-shadow lesson).
TRANSIENT_DELTA_1PM = 0.12
TRANSIENT_WINDOW_S = 0.7


def load_map():
    models = json.load(open(MAP_PATH))['models']

    def make(model):
        k = model['knots_us']
        v = model['kappa_1pm']

        def f(pulse):
            if pulse <= k[0]:
                return v[0]
            if pulse >= k[-1]:
                return v[-1]
            i = bisect.bisect_right(k, pulse) - 1
            fr = (pulse - k[i]) / (k[i + 1] - k[i])
            return v[i] + fr * (v[i + 1] - v[i])
        return f
    return {d: make(m) for d, m in models.items() if m}


def load_series(path):
    """Uniform per-row series needed for simulation and scoring."""
    try:
        with open(path, errors='replace') as f:
            rows = list(csv.DictReader(
                line.replace('\x00', '') for line in f))
    except Exception:
        return None
    need = {'monotonic_s', 'state', 'fresh_odom', 'odom_outlier',
            'measured_v_mps', 'measured_w_radps', 'steering_us'}
    if not rows or not need.issubset(rows[0].keys()):
        return None

    def safe(r, k):
        try:
            return float(r.get(k) or 0)
        except ValueError:
            return None

    out = {'t': [], 'pulse': [], 'v': [], 'km': [], 'eligible': []}
    for r in rows:
        t = safe(r, 'monotonic_s')
        pulse = safe(r, 'steering_us')
        v = safe(r, 'measured_v_mps')
        w = safe(r, 'measured_w_radps')
        if None in (t, pulse, v, w):
            continue
        elig = (r['state'] == 'rolling' and r['fresh_odom'] == '1'
                and r['odom_outlier'] != '1' and abs(v) >= SPEED_FLOOR_MPS)
        km = w / v if abs(v) >= SPEED_FLOOR_MPS else 0.0
        if abs(km) > 2.5:
            elig = False
        out['t'].append(t)
        out['pulse'].append(pulse)
        out['v'].append(v)
        out['km'].append(km)
        out['eligible'].append(elig)
    return out if out['t'] else None


def transient_flags(series, maps):
    """Per-row: did the map target move > delta within the trailing window."""
    ts = series['t']
    targets = []
    for i in range(len(ts)):
        d = 'fwd' if series['v'][i] >= 0 else 'rev'
        targets.append(maps[d](series['pulse'][i]) if d in maps else 0.0)
    flags = [False] * len(ts)
    j = 0
    for i in range(len(ts)):
        while ts[i] - ts[j] > TRANSIENT_WINDOW_S:
            j += 1
        flags[i] = abs(targets[i] - targets[j]) > TRANSIENT_DELTA_1PM
    return flags


def simulate_errors(series, maps, combos, flags):
    """Per-combo transient/settled absolute-error lists, per direction."""
    n_c = len(combos)
    eff = {'fwd': np.zeros(n_c), 'rev': np.zeros(n_c)}
    errs = {d: [{'t': [], 's': []} for _ in range(n_c)]
            for d in ('fwd', 'rev')}
    taus = np.array([c[0] for c in combos])
    dists = np.array([c[1] for c in combos])
    prev_t = None
    for i in range(len(series['t'])):
        t = series['t'][i]
        dt = 0.0 if prev_t is None else max(0.0, min(0.5, t - prev_t))
        prev_t = t
        v = series['v'][i]
        d = 'fwd' if v >= 0 else 'rev'
        if d not in maps:
            continue
        target = maps[d](series['pulse'][i])
        speed = max(abs(v), 0.05)
        tau_eff = taus + dists / speed
        alpha = 1.0 - np.exp(-dt / np.maximum(tau_eff, 1e-3))
        eff[d] += (target - eff[d]) * alpha
        # opposite-direction state relaxes toward same target (servo moves
        # regardless of drive direction)
        od = 'rev' if d == 'fwd' else 'fwd'
        eff[od] += (target - eff[od]) * alpha
        if series['eligible'][i]:
            km = series['km'][i]
            e = np.abs(km - eff[d])
            key = 't' if flags[i] else 's'
            for c in range(n_c):
                errs[d][c][key].append(float(e[c]))
    return errs


def champion_errors(series, maps, flags):
    """Pure-delay champion transient/settled absolute errors, per direction."""
    times = series['t']
    pulses = series['pulse']

    def pulse_at(t):
        i = bisect.bisect_right(times, t) - 1
        return pulses[i] if i >= 0 else pulses[0]

    errs = {d: {'t': [], 's': []} for d in ('fwd', 'rev')}
    for i in range(len(times)):
        if not series['eligible'][i]:
            continue
        v = series['v'][i]
        d = 'fwd' if v >= 0 else 'rev'
        if d not in maps:
            continue
        lag = FWD_TIME_LAG_S if v >= 0 \
            else REV_TIME_LAG_S + REV_DISTANCE_LAG_M / max(abs(v), 0.05)
        target = maps[d](pulse_at(times[i] - lag))
        errs[d]['t' if flags[i] else 's'].append(
            abs(series['km'][i] - target))
    return errs


def main():
    maps = load_map()
    logs = sorted(glob.glob(os.path.expanduser('~/.robot/drive_logs/*.csv')))
    train = [p for p in logs
             if not any(s in os.path.basename(p) for s in HELD_OUT_STAMPS)]
    held = [p for p in logs
            if any(s in os.path.basename(p) for s in HELD_OUT_STAMPS)]
    combos = [(t, dc) for t in TAUS for dc in DISTS]

    agg = {d: [{'t': [], 's': []} for _ in combos] for d in ('fwd', 'rev')}
    files_used = 0
    for p in train:
        s = load_series(p)
        if s is None or not any(s['eligible']):
            continue
        files_used += 1
        flags = transient_flags(s, maps)
        errs = simulate_errors(s, maps, combos, flags)
        for d in ('fwd', 'rev'):
            for c in range(len(combos)):
                agg[d][c]['t'].extend(errs[d][c]['t'])
                agg[d][c]['s'].extend(errs[d][c]['s'])
    print(f'training files used: {files_used}/{len(train)}')

    best = {}
    for d in ('fwd', 'rev'):
        scores = []
        for c in range(len(combos)):
            bt, bs = agg[d][c]['t'], agg[d][c]['s']
            mt = statistics.median(bt) if bt else 9e9
            ms = statistics.median(bs) if bs else 9e9
            scores.append(mt + ms)
        c = int(np.argmin(scores))
        mt = statistics.median(agg[d][c]['t'])
        ms = statistics.median(agg[d][c]['s'])
        best[d] = {'tau_s': combos[c][0], 'dist_m': combos[c][1],
                   'train_median': round((mt + ms) / 2, 4),
                   'train_transient_median': round(mt, 4),
                   'train_settled_median': round(ms, 4),
                   'train_n': len(agg[d][c]['t']) + len(agg[d][c]['s'])}
        print(f"{d}: best tau={combos[c][0]} s dist={combos[c][1]} m "
              f"(train transient {mt:.4f} settled {ms:.4f})")

    best_combos = [(best['fwd']['tau_s'], best['fwd']['dist_m']),
                   (best['rev']['tau_s'], best['rev']['dist_m'])]
    result = {'model': best, 'held_out': {}}
    print('\nheld-out evaluation (median abs err 1/m, '
          'gradual vs fixed, by regime):')
    for p in held:
        s = load_series(p)
        if s is None:
            continue
        flags = transient_flags(s, maps)
        ch = champion_errors(s, maps, flags)
        errs = simulate_errors(s, maps, best_combos, flags)
        name = os.path.basename(p)
        entry = {}
        for d, ci in (('fwd', 0), ('rev', 1)):
            entry[d] = {}
            for key, label in (('t', 'transient'), ('s', 'settled')):
                a, b = errs[d][ci][key], ch[d][key]
                if not a or not b:
                    continue
                entry[d][label] = {
                    'n': len(a),
                    'challenger_median': round(statistics.median(a), 4),
                    'champion_median': round(statistics.median(b), 4),
                }
                e = entry[d][label]
                print(f"  {name} {d} {label:9s} n={e['n']:4d}  "
                      f"gradual {e['challenger_median']:.3f}  "
                      f"fixed {e['champion_median']:.3f}")
        result['held_out'][name] = entry

    with open(OUT_PATH, 'w') as f:
        json.dump(result, f, indent=2)
    print(f'\n-> {OUT_PATH}')

    # Learned-memory export for the controller (Stage C shadow).
    import yaml
    memory = {
        'schema_version': 1,
        'source': 'steering_dynamics_challenger_v1',
        'directions': {
            'forward': {'tau_s': best['fwd']['tau_s'],
                        'dist_m': best['fwd']['dist_m'],
                        'train_median_1pm': best['fwd']['train_median'],
                        'train_samples': best['fwd']['train_n']},
            'reverse': {'tau_s': best['rev']['tau_s'],
                        'dist_m': best['rev']['dist_m'],
                        'train_median_1pm': best['rev']['train_median'],
                        'train_samples': best['rev']['train_n']},
        },
    }
    mem_path = os.path.expanduser('~/.robot/learned_steering_dynamics.yaml')
    tmp = mem_path + '.tmp'
    with open(tmp, 'w') as f:
        yaml.safe_dump(memory, f)
    os.replace(tmp, mem_path)
    print(f'learned memory -> {mem_path}')


if __name__ == '__main__':
    main()
