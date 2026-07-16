#!/usr/bin/env python3
"""One-time conversion of ~/.robot learned memory from pulse microseconds to
normalized effort (2026-07-13 hardware split). Endpoints come from the birth
certificate, so the transform is the exact one the driver applies in reverse:

  throttle: effort = (us - neutral) / (safe_max - neutral)   [spans symmetric]
  steering: effort = (us - neutral) / (neutral - safe_min)   below neutral
                     (us - neutral) / (safe_max - neutral)   above neutral

Everything already in physical units (speeds, curvatures, taus, RLS gains,
floor observer, trackability) is left untouched. Originals are backed up to
~/.robot_pre_effort_backup_<stamp>/ before writing.
"""

import os
import shutil
import sys
import time

import yaml

ROBOT = os.path.expanduser('~/.robot')
CERT = os.path.expanduser(
    '~/ros2_ws/install/ackermann_robot/share/ackermann_robot/config/birth_certificate.yaml')


def load(path):
    with open(path) as handle:
        return yaml.safe_load(handle)


def save(path, data):
    tmp = path + '.tmp'
    with open(tmp, 'w') as handle:
        yaml.safe_dump(data, handle, default_flow_style=None)
    os.replace(tmp, path)


def main():
    certificate = load(CERT)
    throttle = certificate['actuators']['throttle']
    steering = certificate['actuators']['steering']
    t_neutral = float(throttle['neutral_us'])
    t_span = float(throttle['safe_max_us']) - t_neutral
    s_neutral = float(steering['approximate_neutral_us'])
    s_up = float(steering['safe_max_us']) - s_neutral
    s_down = s_neutral - float(steering['safe_min_us'])

    def t_effort(us):
        return round((float(us) - t_neutral) / t_span, 4)

    def s_effort(us):
        us = float(us)
        span = s_up if us >= s_neutral else s_down
        return round((us - s_neutral) / span, 4)

    stamp = time.strftime('%Y%m%d_%H%M%S')
    backup = os.path.expanduser(f'~/.robot_pre_effort_backup_{stamp}')
    os.makedirs(backup, exist_ok=True)
    for name in ('adaptive_ackermann_runtime.yaml',
                 'learned_steering_map.yaml',
                 'learned_steering_dynamics.yaml'):
        path = os.path.join(ROBOT, name)
        if os.path.exists(path):
            shutil.copy2(path, backup)
    print(f'backup -> {backup}')

    # --- runtime memory ---
    runtime_path = os.path.join(ROBOT, 'adaptive_ackermann_runtime.yaml')
    if os.path.exists(runtime_path):
        runtime = load(runtime_path)
        version = int(runtime.get('version', 0))
        if version >= 8:
            print('runtime already effort-native (v%d); skipping' % version)
        else:
            runtime['version'] = 8
            if 'trim_us' in runtime:
                runtime['trim'] = {k: round(float(v) / t_span, 4)
                                   for k, v in runtime.pop('trim_us').items()}
            for model in runtime.get('breakaway_models', {}).values():
                if 'pulse_us' in model:
                    model['effort'] = t_effort(model.pop('pulse_us'))
            for anchor in runtime.get('cruise_anchor', {}).values():
                if 'pulse_us' in anchor:
                    anchor['effort'] = t_effort(anchor.pop('pulse_us'))
                if 'spread_us' in anchor:
                    anchor['spread'] = round(
                        float(anchor.pop('spread_us')) / t_span, 4)
            probes = []
            for probe in runtime.get('throttle_probe_observations', []):
                if 'pulse_center_us' in probe:
                    probe['center_effort'] = t_effort(
                        probe.pop('pulse_center_us'))
                if 'amplitude_us' in probe:
                    probe['amplitude_effort'] = round(
                        float(probe.pop('amplitude_us')) / t_span, 5)
                if 'slope_mps_per_us' in probe:
                    probe['slope_mps_per_effort'] = round(
                        float(probe.pop('slope_mps_per_us')) * t_span, 4)
                probes.append(probe)
            if probes:
                runtime['throttle_probe_observations'] = probes
            save(runtime_path, runtime)
            print('runtime converted: breakaway '
                  + str({d: m.get('effort')
                         for d, m in runtime.get('breakaway_models',
                                                 {}).items()}))

    # --- learned steering map ---
    map_path = os.path.join(ROBOT, 'learned_steering_map.yaml')
    if os.path.exists(map_path):
        memory = load(map_path)
        converted = False
        for entry in memory.get('directions', {}).values():
            if 'knots_pulse_us' in entry:
                entry['knots_effort'] = [
                    s_effort(k) for k in entry.pop('knots_pulse_us')]
                converted = True
        if converted:
            memory['source'] = str(
                memory.get('source', '')) + '+effort_converted'
            save(map_path, memory)
            print('learned steering map converted')
        else:
            print('learned steering map already effort-native; skipping')

    # learned_steering_dynamics.yaml: tau_s / dist_m are physical, untouched.
    print('done')
    return 0


if __name__ == '__main__':
    sys.exit(main())
