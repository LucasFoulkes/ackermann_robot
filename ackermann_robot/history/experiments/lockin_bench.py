#!/usr/bin/env python3
"""Stage F step 1: offline lock-in bench (docs/stage-f-probe-design.md §4.1).

Validates the probe/demodulator design against REAL lidar-odometry noise
before any hardware probe runs:

  synthetic pulse probe (triangular wave, known f, amplitude A)
    -> first-order ESC lag -> known local slope -> + real speed residuals
    -> optional battery-sag drift ramp
    -> quadrature lock-in at f over N cycles -> slope estimate

Reports estimate error vs (A, N), with and without drift, and the noise
spectrum near candidate probe frequencies. Success criterion: sd < 15% of
the true slope at some amplitude small enough to stay inside the path-error
comfort budget.
"""

import glob
import math
import os
import sys

import numpy as np

FS_HZ = 10.0                     # fresh odometry rate
TRUE_SLOPE = 0.030               # m/s per us, from the measured anchors
ESC_TAU_S = 0.25                 # first-order ESC/drivetrain response
DRIFT_MPS_PER_S = -0.004         # battery-sag style ramp for rejection test
TRIALS = 60


def real_speed_residuals():
    """Pool of measured-speed residuals from recent rolling segments."""
    import csv
    residuals = []
    for path in sorted(glob.glob(
            os.path.expanduser('~/.robot/drive_logs/*.csv')))[-12:]:
        try:
            with open(path, errors='replace') as f:
                rows = list(csv.DictReader(
                    line.replace('\x00', '') for line in f))
        except Exception:
            continue
        speeds = []
        def flush():
            nonlocal speeds
            if len(speeds) >= 40:
                arr = np.array(speeds)
                kernel = np.ones(15) / 15.0
                trend = np.convolve(arr, kernel, mode='same')
                residuals.extend((arr - trend)[8:-8])
            speeds = []

        for r in rows:
            try:
                if r.get('state') != 'rolling':
                    flush()
                    continue
                if r.get('fresh_odom') != '1' or r.get('odom_outlier') == '1':
                    continue  # between odometry samples; not a segment break
                v = float(r['measured_v_mps'])
                if 0.10 < abs(v) < 0.6:
                    speeds.append(abs(v))
                else:
                    flush()
            except (ValueError, KeyError):
                continue
        flush()
    return np.array(residuals)


def triangle_wave(t, freq):
    """Unit-amplitude triangular wave (absolutely continuous density)."""
    phase = (t * freq) % 1.0
    return 4.0 * np.abs(phase - 0.5) - 1.0


def lag_filter(x, dt, tau):
    y = np.zeros_like(x)
    alpha = 1.0 - math.exp(-dt / tau)
    for i in range(1, len(x)):
        y[i] = y[i - 1] + alpha * (x[i] - y[i - 1])
    return y


def lockin_slope(speed, probe_units, t, freq):
    """Quadrature lock-in: transfer amplitude at f => slope estimate."""
    ref_i = np.sin(2.0 * math.pi * freq * t)
    ref_q = np.cos(2.0 * math.pi * freq * t)

    def amp(sig):
        return 2.0 * math.hypot(np.mean(sig * ref_i), np.mean(sig * ref_q))
    out = amp(speed)
    inp = amp(probe_units)          # fundamental of the triangular probe
    return out / inp if inp > 0 else float('nan')


def run_bench(noise_pool):
    rng = np.random.default_rng(7)
    print(f'noise pool: {len(noise_pool)} residual samples, '
          f'sd = {np.std(noise_pool)*1000:.1f} mm/s')
    # candidate probe frequencies: noise power in a narrow band around each
    print('\nresidual noise power by candidate frequency (lower = better):')
    n = min(len(noise_pool), 4096)
    seg = noise_pool[:n] - np.mean(noise_pool[:n])
    spectrum = np.abs(np.fft.rfft(seg)) ** 2
    freqs = np.fft.rfftfreq(n, d=1.0 / FS_HZ)
    for f in (0.5, 0.7, 0.9, 1.1, 1.3):
        band = spectrum[(freqs > f - 0.1) & (freqs < f + 0.1)]
        print(f'  {f:.1f} Hz: {np.mean(band):.4f}')

    freq = 0.9
    print(f'\nbench at {freq} Hz, true slope {TRUE_SLOPE} m/s/us, '
          f'ESC tau {ESC_TAU_S}s, {TRIALS} trials each:')
    lag_gain = 1.0 / math.hypot(1.0, 2.0 * math.pi * freq * ESC_TAU_S)
    print(f'(first-order lag gain at {freq} Hz: {lag_gain:.3f} — '
          'corrected in estimates)')
    header = 'A(us)  N(cyc)  est mean   sd%    drift-sd%   ripple(m/s)'
    print(header)
    for amplitude in (1.0, 2.0, 4.0, 8.0):
        for cycles in (5, 10, 20):
            duration = cycles / freq
            t = np.arange(0, duration, 1.0 / FS_HZ)
            probe = amplitude * triangle_wave(t, freq)
            response = TRUE_SLOPE * lag_filter(probe, 1.0 / FS_HZ, ESC_TAU_S)
            estimates, drift_estimates = [], []
            for _ in range(TRIALS):
                noise = rng.choice(noise_pool, size=len(t))
                est = lockin_slope(response + noise, probe, t, freq) / lag_gain
                estimates.append(est)
                drift = DRIFT_MPS_PER_S * t
                est_d = lockin_slope(response + noise + drift,
                                     probe, t, freq) / lag_gain
                drift_estimates.append(est_d)
            estimates = np.array(estimates)
            drift_estimates = np.array(drift_estimates)
            ripple = float(np.max(np.abs(response)))
            print(f'{amplitude:5.1f}  {cycles:5d}   '
                  f'{np.mean(estimates):7.4f}  '
                  f'{100*np.std(estimates)/TRUE_SLOPE:5.1f}  '
                  f'{100*np.std(drift_estimates)/TRUE_SLOPE:8.1f}   '
                  f'{ripple:.3f}')
    print('\nsuccess criterion: sd% < 15 with ripple acceptably small; '
          'drift-sd% ~ sd% proves 1/f rejection.')


if __name__ == '__main__':
    pool = real_speed_residuals()
    if len(pool) < 500:
        sys.exit(f'not enough rolling residuals ({len(pool)})')
    run_bench(pool)
