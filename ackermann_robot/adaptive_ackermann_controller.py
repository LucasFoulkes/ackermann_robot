#!/usr/bin/env python3
"""Safe adaptive bridge from Nav2 Twist commands to PCA9685 ESC and steering."""

import csv
from collections import deque
import json
import math
import os
import tempfile
import time

import rclpy
import yaml
from ackermann_robot.adaptive_model import (
    DelayEstimator, PathGeometry, compose_preview_curvature,
    gentle_motion_requested, limit_ackermann_twist,
    limit_gentle_launch_pulse, polyline_projection, scan_point_clearance,
    stopping_clearance)
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from nav2_msgs.msg import SpeedLimit
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

TICK_US = 1_000_000.0 / (196.8876 * 4096.0)


def clamp(value, low, high):
    return max(low, min(high, value))


def pairs(flat):
    if len(flat) < 4 or len(flat) % 2:
        raise ValueError('calibration maps require at least two x,y pairs')
    return sorted((float(flat[i]), float(flat[i + 1]))
                  for i in range(0, len(flat), 2))


def interpolate(points, x):
    if x <= points[0][0]:
        a, b = points[0], points[1]
    elif x >= points[-1][0]:
        a, b = points[-2], points[-1]
    else:
        for a, b in zip(points, points[1:]):
            if a[0] <= x <= b[0]:
                break
    fraction = (x - a[0]) / (b[0] - a[0])
    return a[1] + fraction * (b[1] - a[1])


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_difference(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class PCA9685:
    def __init__(self, bus_number, address, prescale):
        from smbus2 import SMBus
        self.bus = SMBus(bus_number)
        self.address = address
        self.bus.write_byte_data(address, 0x00, 0x10)
        self.bus.write_byte_data(address, 0xFE, prescale)
        self.bus.write_byte_data(address, 0x00, 0x20)
        time.sleep(0.01)

    def pulse(self, channel, pulse_us):
        tick = int(round(pulse_us / TICK_US))
        base = 0x06 + 4 * channel
        self.bus.write_i2c_block_data(self.address, base,
                                      [0, 0, tick & 0xff, tick >> 8])

    def release(self, channel):
        base = 0x06 + 4 * channel
        self.bus.write_i2c_block_data(self.address, base, [0, 0, 0, 0x10])

    def close(self):
        self.bus.close()


class AdaptiveAckermannController(Node):
    def __init__(self):
        super().__init__('adaptive_ackermann_controller')
        defaults = {
            'arm_hardware': False, 'i2c_bus': 1, 'pca9685_address': 64,
            'pca9685_prescale': 30, 'throttle_channel': 14,
            'steering_channel': 12, 'throttle_neutral_us': 1500.0,
            'steering_center_us': 1463.0, 'control_rate_hz': 50.0,
            'command_timeout_s': .5, 'odom_timeout_s': .5,
            'scan_timeout_s': .5, 'obstacle_stop_m': .2,
            'maximum_forward_speed_mps': .35, 'maximum_reverse_speed_mps': .30,
            # Normal Nav2 cruise ceiling.  The larger actuator maximum above
            # remains a hard safety clamp, not a request to make RPP drive at
            # that speed.
            'navigation_speed_limit_mps': .30,
            'maximum_measured_speed_mps': .55, 'maximum_curvature_1pm': 1.15,
            # Smac plans inside this fraction of the learned physical envelope,
            # leaving curvature authority for transient tracking corrections.
            'planning_curvature_utilization': .80,
            # Bootstrap only. The dispatcher replaces this prior with persisted
            # four-branch path-tracking evidence on later launches.
            'planner_trackability_prior_radius_m': 1.30,
            'planner_trackability_state_path': (
                '~/.robot/planner_trackability.yaml'),
            'minimum_braking_deceleration_mps2': .45,
            'sensor_reaction_time_s': .10,
            'odom_velocity_median_samples': 3,
            'motion_threshold_mps': .025, 'breakaway_threshold_mps': .045,
            # Measured 2026-07-11 (esc_sustain_floor_id): rolling dies below
            # ~0.08 m/s in both directions; 0.10 crawl is marginal. Never
            # command slower than this or the robot stalls into recovery.
            'minimum_sustain_speed_mps': .12,
            'throttle_kp_us_per_mps': 0.0, 'throttle_ki_us_per_m': 30.0,
            'throttle_rolling_slew_us_per_odom': 6.0,
            'forward_breakaway_prior_us': 1403.0,
            'reverse_breakaway_prior_us': 1611.0,
            'breakaway_forgetting_factor': .90,
            # Session warm-up: battery state changes between sessions, so the
            # persisted breakaway is stale at boot (fresh charge after a
            # sagged evening produced 0.5-0.8 m/s launches for ~3 min on
            # 2026-07-11 19:59). First launches per direction start the ramp
            # weaker and re-learn the threshold with faster forgetting.
            'session_warmup_launches': 5,
            'session_warmup_weaken_us': 6.0,
            'session_warmup_forgetting': .70,
            # Shadow effort-scale estimator: one latent gain between the maps
            # and the world (battery, rug, temperature -- the cause does not
            # matter, only the current value). Measured from steady rolling
            # samples in BOTH directions as (pulse-neutral)/(map(v)-neutral);
            # variance grows over evidence gaps, so a boot after charging is
            # automatically uncertain. Not applied to commands yet.
            'effort_scale_measurement_sd': 0.05,
            'effort_scale_process_per_s': 0.0005,
            'effort_scale_boot_sd': 0.10,
            'startup_drop_through_mps': 0.15,
            # The ESC cannot roll continuously at arbitrarily low speed. For
            # gentle requests and short committed segments, pre-steer, apply a
            # bounded learned breakaway pulse, then coast before re-arming.
            'gentle_request_max_mps': .10,
            'gentle_segment_length_m': .15,
            'gentle_steering_ready_us': 60.0,
            'gentle_min_coast_s': .35,
            'gentle_max_pulse_s': .45,
            'gentle_rearm_speed_mps': .025,
            'gentle_launch_max_extra_us': 6.0,
            # Suppress the recovery kick when stalling this close to a path
            # cusp: the stop is intended, and the kick was pure lurch. If the
            # command persists past the wait, recovery proceeds as fallback.
            'planned_stop_cusp_m': 0.40,
            'planned_stop_wait_s': 2.0,
            # RPP on Jazzy flip-flops command sign near path inversions
            # (+-+-+ traces, 21:34 drive) while the robot still carries the
            # old motion -- the real cause of the 'direction mismatch' storm
            # (the flip experiment measured ZERO vehicle rock-back). Hold
            # neutral until the commanded sign is stable this long.
            'direction_flip_debounce_s': 0.5,
            # Stage-2 probe scheduler: tiny steering taps during steady
            # straight cruising, for the identification estimators. An 8 us
            # tap is ~0.024 1/m of curvature for half a second: ~2 mm of
            # lateral deviation, well inside tracking noise. Measure-only:
            # taps are logged (probe_id / probe_offset_us) but nothing
            # learns from them yet.
            'probe_enabled': False,
            'probe_amplitude_us': 8.0,
            'probe_duration_s': 0.5,
            'probe_interval_s': 4.0,
            'probe_max_curvature_1pm': 0.15,
            'probe_clearance_margin_m': 0.30,
            'probe_min_stop_distance_m': 1.0,
            'maximum_throttle_trim_us': 40.0, 'forward_rolling_min_us': 1380.0,
            'reverse_rolling_max_us': 1640.0, 'forward_recovery_limit_us': 1180.0,
            'reverse_recovery_limit_us': 1820.0, 'recovery_step_us': 10.0,
            'recovery_limit_dwell_samples': 10, 'steering_slew_us_per_s': 1000.0,
            'steering_slowdown_error_us': 30.0,
            'steering_learning_min_speed_mps': .10,
            'steering_identification_window_s': .55,
            'steering_identification_min_distance_m': .06,
            'steering_rls_forgetting_factor': .995,
            'steering_rls_min_observations': 12,
            'steering_rls_full_confidence_observations': 42,
            'steering_rls_residual_limit_1pm': .50,
            'steering_delay_candidates_s': [0.0, .10, .20, .30, .40, .50],
            'steering_delay_forgetting_factor': .98,
            'steering_delay_min_observations': 12,
            # Two-speed step experiment 2026-07-11: forward lag is a pure time
            # delay; reverse adds a rolled-distance term (trailing steer axle).
            # Measured through the odometry pipeline, so valid for prediction.
            'steering_lag_time_forward_s': .235,
            'steering_lag_time_reverse_s': .168,
            'steering_lag_distance_forward_m': 0.0,
            'steering_lag_distance_reverse_m': .033,
            # MPPI already predicts path motion and owns path inversions.
            # The pure-pursuit feed-forward lead is retained only for legacy
            # controller experiments and must stay off with MPPI.
            'apply_path_preview_compensation': False,
            # MPPI chose only ~0.22 1/m for a certified 1.15 1/m reverse arc
            # in the 11:17 drive. Supply the missing same-sign path curvature,
            # while preserving stronger/opposite MPPI recovery corrections.
            'apply_path_curvature_floor': False,
            'path_curvature_floor_max_lateral_error_m': .20,
            'path_curvature_floor_max_heading_error_rad': .50,
            # False: RLS keeps identifying and logging but corrections are not
            # applied to commands (shadow mode; static maps verified 2026-07-11).
            'apply_steering_models': False,
            'vehicle_wheelbase_m': .2775,
            # Committed paths are authoritative until Nav2 replaces them.
            # Set positive only for a stack that republishes plans periodically.
            'path_timeout_s': 0.0,
            'path_inversion_tolerance_m': .12,
            # If the old-segment command carries the vehicle away after it has
            # entered the cusp ball, stop that direction. Opposite-direction
            # recovery remains allowed, unlike the removed hard cusp gate.
            'cusp_guard_entry_m': .18,
            'cusp_guard_departure_m': .08,
            # The FollowPath proxy is the sole owner of cusp chronology. The
            # actuator bridge must not interpret the planner's full /plan.
            'enable_external_path_interpretation': False,
            'lidar_x_m': .237,
            'lidar_y_m': 0.0,
            'lidar_yaw_rad': math.pi,
            'footprint_front_x_m': .40,
            'footprint_rear_x_m': -.10,
            'footprint_half_width_m': .16,
            'rpp_lookahead_time_s': 1.5,
            'rpp_min_lookahead_m': .30,
            'rpp_max_lookahead_m': .75,
            'runtime_model_path': '~/.robot/adaptive_ackermann_runtime.yaml',
            'drive_log_directory': '~/.robot/drive_logs',
            'forward_throttle_map': [.2, 1419.38, .25, 1417.74],
            'reverse_throttle_map': [.2, 1592.51, .25, 1593.52],
            'forward_steering_map': [-1.15, 1077.0, 0., 1451.5, 1.15, 1764.0],
            'reverse_steering_map': [-1.15, 1070.0, 0., 1475.5, 1.15, 1792.0],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        self.throttle_maps = {d: pairs(self.p[f'{d}_throttle_map'])
                              for d in ('forward', 'reverse')}
        self.steering_maps = {d: pairs(self.p[f'{d}_steering_map'])
                              for d in ('forward', 'reverse')}
        self.runtime_path = os.path.expanduser(self.p['runtime_model_path'])
        log_directory = os.path.expanduser(self.p['drive_log_directory'])
        os.makedirs(log_directory, exist_ok=True)
        self.drive_log_path = os.path.join(
            log_directory, time.strftime('adaptive_drive_%Y%m%d_%H%M%S.csv'))
        self.drive_log = open(self.drive_log_path, 'w', newline='', buffering=1)
        self.drive_writer = csv.DictWriter(self.drive_log, fieldnames=[
            'monotonic_s', 'fresh_odom', 'state', 'fault', 'direction',
            'planner_v_mps', 'planner_w_radps', 'post_collision_v_mps',
            'post_collision_w_radps', 'planner_command_age_s',
            'post_collision_command_age_s', 'accepted_v_mps',
            'rpp_curvature_1pm', 'accepted_curvature_1pm', 'measured_v_mps',
            'measured_w_radps', 'raw_v_mps', 'raw_w_radps',
            'odom_outlier', 'throttle_us', 'steering_us',
            'closest_obstacle_m', 'front_clearance_m', 'rear_clearance_m',
            'low_samples', 'limit_dwell',
            'throttle_forward_trim_us', 'throttle_reverse_trim_us',
            'curvature_error_ema', 'steering_rls_models_json',
            'estimated_steering_delay_s',
            'steering_delay_confidence', 'steering_delay_weights_json',
            'path_curvature_now_1pm', 'path_curvature_preview_1pm',
            'rpp_feedback_curvature_1pm', 'preview_distance_m',
            'preview_horizon_s', 'preview_blend', 'preview_curvature_1pm',
            'steering_assist_active', 'steering_command_source',
            'cusp_distance_m', 'path_end_distance_m',
            'cross_track_error_m', 'heading_error_rad',
            'nearest_path_distance_m', 'path_direction',
            'required_stop_clearance_m', 'throttle_antiwindup_state'])
        self.drive_writer.fieldnames.extend([
            'limited_v_mps', 'limited_w_radps',
            'command_gate_reason', 'path_progress_index',
            'path_segment_end_index',
            'breakaway_models_json', 'startup_effort_attempts',
            'startup_causal_pulse_us', 'floor_estimate_forward_mps',
            'floor_estimate_reverse_mps', 'floor_observer_json',
            'probe_id', 'probe_offset_us',
            'effort_scale', 'effort_scale_sd',
            'segment_remaining_m', 'gentle_motion',
            'gentle_coast_active', 'gentle_pulse_active'])
        self.drive_writer.writeheader()
        self.trim = {'throttle_forward': 0., 'throttle_reverse': 0.}
        self.curvature_error_ema = 0.0
        self.curvature_observations = 0
        self.breakaway_models = {
            'forward': {'pulse_us': self.p['forward_breakaway_prior_us'],
                        'observations': 0},
            'reverse': {'pulse_us': self.p['reverse_breakaway_prior_us'],
                        'observations': 0}}
        # The experiment maps are the prior. Each model continuously identifies
        # measured_curvature = gain * baseline_curvature + bias from ordinary
        # Nav2 driving. Corrections remain in shadow mode until confidence grows.
        self.steering_models = {
            f'{direction}_{side}': {
                'gain': 1.0, 'bias': 0.0,
                'covariance': [[4.0, 0.0], [0.0, 0.25]],
                'observations': 0, 'residual_ema': 1.0,
            }
            for direction in ('forward', 'reverse')
            for side in ('negative', 'positive')}
        self.delay_candidates = [
            float(x) for x in self.p['steering_delay_candidates_s']]
        self.delay_estimator = DelayEstimator(
            self.delay_candidates,
            self.p['steering_delay_forgetting_factor'],
            self.p['steering_delay_min_observations'])
        self.estimated_steering_delay_s = 0.0
        self.pulse_history = deque(maxlen=600)
        self.throttle_history = deque(maxlen=600)
        self.motion_history = deque(maxlen=80)
        self.last_identification_time = 0.0
        # Shadow stop-decay observer: learns the minimum sustainable speed
        # from ordinary driving (died = entered recovery while commanded;
        # held = sustained slow rolling). Estimate is logged and persisted
        # but does NOT yet gate commands; minimum_sustain_speed_mps does.
        self.floor_observer = {
            direction: {'died_ema_mps': 0.0, 'died_observations': 0,
                        'held_ema_mps': 0.0, 'held_observations': 0}
            for direction in ('forward', 'reverse')}
        self.floor_streak = 0
        self.floor_streak_min = math.inf
        self.recent_rolling_speeds = deque(maxlen=5)
        # The value persists across boots, while its uncertainty deliberately
        # resets.  Initialize before loading schema-v7 runtime state so the
        # persisted value is not overwritten later in __init__.
        self.effort_scale = 1.0
        self.effort_scale_var = self.p['effort_scale_boot_sd'] ** 2
        self.effort_scale_time = time.monotonic()
        self._load_runtime()
        self.cmd = Twist(); self.planner_cmd = Twist(); self.limited_cmd = Twist()
        self.cmd_time = None; self.planner_cmd_time = None; self.scan_time = None
        self.closest = math.inf; self.closest_forward = math.inf
        self.closest_reverse = math.inf; self.pose = None; self.odom_time = None
        self.speed = 0.; self.yaw_rate = 0.; self.last_tf_stamp = None
        self.raw_speed = 0.; self.raw_yaw_rate = 0.
        self.odom_outlier = False
        observer_samples = int(self.p['odom_velocity_median_samples'])
        if observer_samples < 1 or observer_samples % 2 == 0:
            raise ValueError('odom_velocity_median_samples must be a positive odd integer')
        self.speed_history = deque(maxlen=observer_samples)
        self.yaw_rate_history = deque(maxlen=observer_samples)
        self.odom_sequence = 0; self.controlled_odom_sequence = 0
        self.last_control_odom_stamp = None
        self.state = 'stopped'; self.direction = None; self.low_samples = 0
        self.limit_dwell = 0; self.throttle_us = self.p['throttle_neutral_us']
        self.steering_us = self.p['steering_center_us']; self.last_tick = time.monotonic()
        self.learn_start = None; self.learn_target = None; self.last_save = time.monotonic()
        self.startup_last_effort_time = None
        self.startup_effort_attempts = 0
        self.startup_causal_pulse_us = math.nan
        self.startup_kick_pulse_us = math.nan
        self.planned_stop_since = 0.0
        self.control_segment_samples = None
        self.control_segment_length = 0.0
        self.control_segment_time = None
        self.segment_remaining_m = math.inf
        self.gentle_motion = False
        self.gentle_coast_active = False
        self.gentle_coast_since = 0.0
        self.gentle_pulse_active = False
        self.gentle_pulse_since = 0.0
        self.session_launches = {'forward': 0, 'reverse': 0}
        self.last_cmd_sign = 0
        self.cmd_sign_since = 0.0
        self.rolling_since = 0.0
        self.probe_id = 0
        self.probe_sign = 1.0
        self.probe_until = 0.0
        self.probe_last_end = 0.0
        self.probe_offset_us = 0.0
        self.path_geometry = None; self.path_time = None
        self.path_segment_start = 0
        self.path_segment_end = 0
        self.path_progress_index = 0
        self.path_segment_direction = 0
        self.command_gate_reason = ''
        self.cusp_min_distance = math.inf
        self.cusp_missed = False
        self.preview = self._empty_preview()
        self.throttle_antiwindup_state = 'inactive'
        self.fault = ''
        self.required_stop_clearance = self.p['obstacle_stop_m']
        self.hardware = None
        if self.p['arm_hardware']:
            self.hardware = PCA9685(self.p['i2c_bus'], self.p['pca9685_address'],
                                    self.p['pca9685_prescale'])
            self.hardware.pulse(self.p['throttle_channel'], self.p['throttle_neutral_us'])
            time.sleep(3.0)
            self.get_logger().warn('HARDWARE ARMED: neutral held for 3 seconds')
        else:
            self.get_logger().warn('Hardware DISARMED; actuator output is simulated')
        self.limited_cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd, 10)
        self.create_subscription(
            Twist, '/cmd_vel_nav_raw', self._planner_cmd, 10)
        if self.p['enable_external_path_interpretation']:
            self.create_subscription(Path, '/plan', self._path, 10)
        # Unlike the planner's mutable full /plan, this is the exact
        # one-direction path currently owned by the dispatcher.
        self.create_subscription(
            Path, '/controller_segment_plan', self._control_segment, 10)
        self.create_subscription(TFMessage, '/tf', self._tf, 50)
        self.create_subscription(LaserScan, '/scan', self._scan, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self.debug_pub = self.create_publisher(String, '/controller/debug', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.speed_limit_pub = self.create_publisher(SpeedLimit, '/speed_limit', 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.limit_pub = self.create_publisher(String, '/controller/limits', qos)
        self.limit_pub.publish(String(data=json.dumps({
            'max_forward_mps': self.p['maximum_forward_speed_mps'],
            'max_reverse_mps': self.p['maximum_reverse_speed_mps'],
            'navigation_speed_limit_mps': self.p['navigation_speed_limit_mps'],
            'max_curvature_1pm': self.p['maximum_curvature_1pm'],
            'minimum_turning_radius_m': 1. / self.p['maximum_curvature_1pm']})))
        self.create_timer(1. / self.p['control_rate_hz'], self._tick)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(f'Drive flight recorder: {self.drive_log_path}')

    def _load_runtime(self):
        try:
            with open(self.runtime_path) as stream:
                saved = yaml.safe_load(stream) or {}
            # Version 2 trims were learned while Collision Monitor fragmented
            # commands and reached their authority limits. They are not valid
            # priors for the continuous estimator, so newer models start from the
            # experiment feed-forward maps without requiring a manual file edit.
            version = int(saved.get('version', 0))
            if version >= 5:
                for key in self.trim:
                    self.trim[key] = float(
                        saved.get('trim_us', {}).get(key, 0.))
            saved_models = saved.get('steering_rls_models', {})
            for key, model in self.steering_models.items():
                candidate = saved_models.get(key, {})
                if int(candidate.get('observations', 0)) < 1:
                    continue
                model['gain'] = clamp(float(candidate.get('gain', 1.0)), .4, 1.6)
                model['bias'] = clamp(float(candidate.get('bias', 0.0)), -.35, .35)
                model['observations'] = int(candidate['observations'])
                model['residual_ema'] = max(0.0, float(candidate.get('residual_ema', 1.0)))
                covariance = candidate.get('covariance')
                if (isinstance(covariance, list) and len(covariance) == 2 and
                        all(isinstance(row, list) and len(row) == 2 for row in covariance)):
                    model['covariance'] = [[float(x) for x in row] for row in covariance]
            if version >= 4:
                self.delay_estimator = DelayEstimator(
                    self.delay_candidates,
                    self.p['steering_delay_forgetting_factor'],
                    self.p['steering_delay_min_observations'],
                    saved.get('steering_delay_estimator'))
            if version >= 6:
                for direction, model in self.breakaway_models.items():
                    candidate = saved.get('breakaway_models', {}).get(direction, {})
                    if int(candidate.get('observations', 0)) > 0:
                        model['pulse_us'] = float(candidate['pulse_us'])
                        model['observations'] = int(candidate['observations'])
            if version >= 7:
                for direction, model in self.floor_observer.items():
                    candidate = saved.get('floor_observer', {}).get(direction, {})
                    for key in model:
                        if key in candidate:
                            model[key] = (int(candidate[key])
                                          if key.endswith('observations')
                                          else float(candidate[key]))
                if 'effort_scale' in saved:
                    self.effort_scale = clamp(
                        float(saved['effort_scale']), 0.7, 1.3)
            self.estimated_steering_delay_s = self.delay_estimator.estimate
        except FileNotFoundError:
            pass
        except Exception as exc:
            self.get_logger().warning(f'Ignoring runtime model: {exc}')

    def _save_runtime(self):
        directory = os.path.dirname(self.runtime_path)
        os.makedirs(directory, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=directory, prefix='.adaptive-', text=True)
        try:
            with os.fdopen(fd, 'w') as stream:
                yaml.safe_dump({
                    'version': 7, 'trim_us': self.trim,
                    'steering_rls_models': self.steering_models,
                    'estimated_steering_delay_s': self.estimated_steering_delay_s,
                    'steering_delay_estimator': self.delay_estimator.state(),
                    'breakaway_models': self.breakaway_models,
                    'floor_observer': self.floor_observer,
                    # Value persists; variance deliberately does not, so every
                    # boot starts uncertain and re-verifies against evidence.
                    'effort_scale': self.effort_scale,
                }, stream)
            os.replace(tmp, self.runtime_path)
        finally:
            if os.path.exists(tmp): os.unlink(tmp)

    def _cmd(self, msg):
        self.cmd = msg; self.cmd_time = time.monotonic()

    def _planner_cmd(self, msg):
        self.planner_cmd = msg; self.planner_cmd_time = time.monotonic()
        linear, angular = limit_ackermann_twist(
            msg.linear.x, msg.angular.z,
            min(self.p['maximum_forward_speed_mps'],
                self.p['navigation_speed_limit_mps']),
            min(self.p['maximum_reverse_speed_mps'],
                self.p['navigation_speed_limit_mps']),
            self.p['maximum_curvature_1pm'])
        # This is the sole executable-command boundary. Collision Monitor must
        # inspect the same nonzero speed and curvature the hardware will use.
        self.command_gate_reason = ''
        if abs(linear) >= .01:
            curvature = angular / linear
            linear = math.copysign(
                max(abs(linear), self.p['minimum_sustain_speed_mps']), linear)
            direction = 'forward' if linear > 0.0 else 'reverse'
            target_pulse = self._steering_pulse(direction, curvature)
            steering_error = abs(target_pulse - self.steering_us)
            if steering_error > self.p['steering_slowdown_error_us']:
                scale = clamp(1.0 - steering_error / 300.0, 0.35, 1.0)
                linear = math.copysign(
                    max(self.p['minimum_sustain_speed_mps'],
                        abs(linear) * scale), linear)
            angular = linear * curvature
        limited = Twist()
        limited.linear.x = linear
        limited.angular.z = angular
        self.limited_cmd = limited
        self.limited_cmd_pub.publish(limited)

    def _reset_path_commitment(self):
        self.path_segment_start = 0
        self.path_progress_index = 0
        self.command_gate_reason = ''
        self.cusp_min_distance = math.inf
        self.cusp_missed = False
        if self.path_geometry is None:
            self.path_segment_end = 0
            self.path_segment_direction = 0
            return
        self.path_segment_direction = self.path_geometry.direction_at(0)
        self.path_segment_end = self.path_geometry.segment_end_index(0)

    def _update_path_progress(self):
        if self.path_geometry is None or self.pose is None:
            return None
        lower = max(self.path_segment_start, self.path_progress_index - 2)
        nearest = self.path_geometry.nearest_index_between(
            self.pose[0], self.pose[1], lower, self.path_segment_end)
        # Path chronology is authoritative. Noise may move the nearest sample
        # slightly backward, but progress never jumps backward or across a cusp.
        self.path_progress_index = max(self.path_progress_index, nearest)
        return self.path_progress_index

    def _observe_path_direction(self, linear):
        """Advance telemetry commitment at a cusp without modifying commands."""
        self.command_gate_reason = ''
        if (abs(linear) < .01 or self.path_geometry is None or
                self.pose is None or self.path_segment_direction == 0):
            return
        self._update_path_progress()
        requested = 1 if linear > 0.0 else -1
        samples = self.path_geometry.samples
        if self.path_segment_end >= len(samples) - 1:
            return
        next_direction = self.path_geometry.direction_at(self.path_segment_end)
        cusp_x, cusp_y, _ = samples[self.path_segment_end]
        cusp_distance = math.hypot(self.pose[0] - cusp_x, self.pose[1] - cusp_y)
        if requested == self.path_segment_direction:
            self.cusp_min_distance, newly_missed = update_cusp_guard(
                self.cusp_min_distance, cusp_distance,
                self.p['cusp_guard_entry_m'],
                self.p['cusp_guard_departure_m'])
            self.cusp_missed = self.cusp_missed or newly_missed
            return
        stopped_at_cusp = (
            cusp_distance <= self.p['path_inversion_tolerance_m'] and
            abs(self.speed) <= self.p['breakaway_threshold_mps'])
        if requested == next_direction and stopped_at_cusp:
            self.path_segment_start = self.path_segment_end
            self.path_progress_index = self.path_segment_start
            self.path_segment_direction = next_direction
            self.path_segment_end = self.path_geometry.segment_end_index(
                self.path_segment_start)
            self.cusp_min_distance = math.inf
            self.cusp_missed = False

    def _empty_preview(self):
        return {
            'path_curvature_now_1pm': 0.0,
            'path_curvature_preview_1pm': 0.0,
            'rpp_feedback_curvature_1pm': 0.0,
            'preview_distance_m': 0.0,
            'preview_horizon_s': 0.0,
            'preview_blend': 0.0,
            'preview_curvature_1pm': 0.0,
            'steering_assist_active': 0,
            'steering_command_source': 'rpp',
            'cusp_distance_m': math.inf,
            'path_end_distance_m': math.inf,
            'cross_track_error_m': math.inf,
            'heading_error_rad': math.inf,
            'nearest_path_distance_m': math.inf,
            'path_direction': 0,
        }

    def _path(self, msg):
        if msg.header.frame_id.lstrip('/') != 'odom' or len(msg.poses) < 3:
            self.path_geometry = None
            self.path_time = None
            self._reset_path_commitment()
            return
        samples = [
            (pose.pose.position.x, pose.pose.position.y,
             yaw_from_quaternion(pose.pose.orientation))
            for pose in msg.poses]
        try:
            self.path_geometry = PathGeometry(samples, 'odom')
            self.path_time = time.monotonic()
            self._reset_path_commitment()
        except ValueError:
            self.path_geometry = None
            self.path_time = None
            self._reset_path_commitment()

    def _control_segment(self, msg):
        if msg.header.frame_id.lstrip('/') != 'odom' or len(msg.poses) < 2:
            self.control_segment_samples = None
            self.control_segment_length = 0.0
            self.control_segment_time = None
            return
        samples = [
            (pose.pose.position.x, pose.pose.position.y,
             yaw_from_quaternion(pose.pose.orientation))
            for pose in msg.poses]
        self.control_segment_samples = samples
        self.control_segment_length = sum(
            math.hypot(second[0] - first[0], second[1] - first[1])
            for first, second in zip(samples, samples[1:]))
        self.control_segment_time = time.monotonic()

    def _segment_remaining(self):
        if self.control_segment_samples is None or self.pose is None:
            return math.inf
        _, _, along = polyline_projection(
            self.control_segment_samples, *self.pose)
        return max(0.0, self.control_segment_length - along)

    def _scan(self, msg):
        valid = []; forward = []; reverse = []
        lidar_x = self.p['lidar_x_m']; lidar_y = self.p['lidar_y_m']
        lidar_yaw = self.p['lidar_yaw_rad']
        front_x = self.p['footprint_front_x_m']
        rear_x = self.p['footprint_rear_x_m']
        half_width = self.p['footprint_half_width_m']
        for index, distance in enumerate(msg.ranges):
            if not msg.range_min <= distance <= msg.range_max:
                continue
            angle = msg.angle_min + index * msg.angle_increment
            external, front_clearance, rear_clearance = scan_point_clearance(
                distance, angle, lidar_x, lidar_y, lidar_yaw,
                front_x, rear_x, half_width)
            if not external:
                continue
            valid.append(distance)
            if front_clearance is not None:
                forward.append(front_clearance)
            if rear_clearance is not None:
                reverse.append(rear_clearance)
        self.closest = min(valid, default=math.inf)
        self.closest_forward = min(forward, default=math.inf)
        self.closest_reverse = min(reverse, default=math.inf)
        self.scan_time = time.monotonic()

    def _tf(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id.lstrip('/') != 'odom' or transform.child_frame_id.lstrip('/') != 'base_link':
                continue
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
            x = transform.transform.translation.x; y = transform.transform.translation.y
            yaw = yaw_from_quaternion(transform.transform.rotation)
            if self.pose is not None and stamp > self.last_tf_stamp:
                dt = stamp - self.last_tf_stamp
                dx, dy = x - self.pose[0], y - self.pose[1]
                forward = math.cos(self.pose[2]) * dx + math.sin(self.pose[2]) * dy
                self.raw_speed = forward / dt
                self.raw_yaw_rate = angle_difference(yaw, self.pose[2]) / dt
                self.speed_history.append(self.raw_speed)
                self.yaw_rate_history.append(self.raw_yaw_rate)
                self.speed = sorted(self.speed_history)[len(self.speed_history) // 2]
                self.yaw_rate = sorted(self.yaw_rate_history)[len(self.yaw_rate_history) // 2]
                yaw_envelope = (
                    self.p['maximum_curvature_1pm'] *
                    self.p['maximum_measured_speed_mps'])
                self.odom_outlier = (
                    len(self.speed_history) == self.speed_history.maxlen and
                    (abs(self.raw_speed - self.speed) >
                     self.p['maximum_measured_speed_mps'] or
                     abs(self.raw_yaw_rate - self.yaw_rate) > yaw_envelope))
                if self.odom_outlier:
                    self.motion_history.clear()
                else:
                    self._learn_steering(x, y, yaw)
                self.odom_sequence += 1
            self.pose = (x, y, yaw); self.last_tf_stamp = stamp
            self.odom_time = time.monotonic()
            odom = Odometry(); odom.header = transform.header
            odom.child_frame_id = 'base_link'; odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y; odom.pose.pose.orientation = transform.transform.rotation
            odom.twist.twist.linear.x = self.speed; odom.twist.twist.angular.z = self.yaw_rate
            self.odom_pub.publish(odom)

    def _desired(self):
        now = time.monotonic()
        if self.cmd_time is None or now - self.cmd_time > self.p['command_timeout_s']:
            return 0., 0., 'command watchdog'
        if self.odom_time is None or now - self.odom_time > self.p['odom_timeout_s']:
            return 0., 0., 'odometry stale'
        if self.scan_time is None or now - self.scan_time > self.p['scan_timeout_s']:
            return 0., 0., 'scan stale'
        # Gate on the faster of measured and commanded speed. Gating on
        # measured speed alone chattered near obstacles: a stopped robot
        # needs almost no clearance, so it launched, the launch spike
        # inflated the requirement past the actual gap, and it slammed to a
        # stop — one launch every 6 s in the 2026-07-11 13:27 run. Commanded
        # speed is RPP-regulated, so goal approaches still creep legally.
        planning_speed = max(
            abs(self.speed),
            abs(clamp(self.cmd.linear.x,
                      -self.p['maximum_reverse_speed_mps'],
                      self.p['maximum_forward_speed_mps'])))
        self.required_stop_clearance = stopping_clearance(
            planning_speed, self.p['sensor_reaction_time_s'],
            self.p['minimum_braking_deceleration_mps2'],
            self.p['obstacle_stop_m'])
        if (self.cmd.linear.x > 0. and
                self.closest_forward < self.required_stop_clearance):
            return 0., 0., 'forward obstacle stop'
        if (self.cmd.linear.x < 0. and
                self.closest_reverse < self.required_stop_clearance):
            return 0., 0., 'reverse obstacle stop'
        requested_v = clamp(
            self.cmd.linear.x, -self.p['maximum_reverse_speed_mps'],
            self.p['maximum_forward_speed_mps'])
        if abs(requested_v) < .01: return 0., 0., 'zero command'
        curvature = clamp(
            self.cmd.angular.z / requested_v,
            -self.p['maximum_curvature_1pm'],
                          self.p['maximum_curvature_1pm'])
        # Minimum sustainable speed and steering-transition slowdown were
        # already applied before Collision Monitor. Execute the approved Twist
        # unchanged; downstream safety logic may only replace it with zero.
        return requested_v, curvature, ''

    def _steering_pulse(self, direction, curvature):
        side = 'negative' if curvature < 0.0 else 'positive'
        model = self.steering_models[f'{direction}_{side}']
        confidence = (self._model_confidence(model)
                      if self.p['apply_steering_models'] else 0.0)
        gain = max(.4, model['gain'])
        identified_input = (curvature - model['bias']) / gain
        corrected_curvature = curvature + confidence * (identified_input - curvature)
        corrected_curvature = clamp(
            corrected_curvature, -self.p['maximum_curvature_1pm'],
            self.p['maximum_curvature_1pm'])
        return clamp(
            interpolate(self.steering_maps[direction], corrected_curvature),
            1000.0, 2000.0)

    def _model_confidence(self, model):
        required = self.p['steering_rls_min_observations']
        sample_confidence = clamp(
            ((model['observations'] - required) /
             max(1, self.p['steering_rls_full_confidence_observations'] - required)),
            0.0, 1.0)
        residual_confidence = clamp(
            1.0 - model['residual_ema'] /
            self.p['steering_rls_residual_limit_1pm'], 0.0, 1.0)
        return sample_confidence * residual_confidence

    def _steering_lag(self, direction, speed):
        # Speed floor matches the minimum sustainable speed: the distance
        # term is never evaluated below a speed the robot can actually hold.
        return (self.p[f'steering_lag_time_{direction}_s'] +
                self.p[f'steering_lag_distance_{direction}_m'] /
                max(abs(speed), self.p['minimum_sustain_speed_mps']))

    def _preview_command(self, rpp_curvature, target_speed, direction):
        result = self._empty_preview()
        result['preview_curvature_1pm'] = rpp_curvature
        result['steering_command_source'] = 'rpp'
        if not self.p['enable_external_path_interpretation']:
            self.preview = result
            return rpp_curvature
        now = time.monotonic()
        path_stale = (
            self.p['path_timeout_s'] > 0.0 and
            self.path_time is not None and
            now - self.path_time > self.p['path_timeout_s'])
        if (self.path_geometry is None or self.path_time is None or
                path_stale or self.pose is None):
            self.preview = result
            return rpp_curvature
        geometry = self.path_geometry
        index = self._update_path_progress()
        if index is None:
            self.preview = result
            return rpp_curvature
        direction_sign = 1 if direction == 'forward' else -1
        path_direction = geometry.direction_at(index)
        lateral, heading, nearest = geometry.tracking_error(
            index, self.pose[0], self.pose[1], self.pose[2])
        result.update({
            'cross_track_error_m': lateral,
            'heading_error_rad': heading,
            'nearest_path_distance_m': nearest,
            'path_direction': path_direction,
            'cusp_distance_m': geometry.cusp_distance(index, path_direction),
            'path_end_distance_m': geometry.remaining_path(index),
        })
        if path_direction != direction_sign:
            self.preview = result
            return rpp_curvature
        lookahead = clamp(
            abs(target_speed) * self.p['rpp_lookahead_time_s'],
            self.p['rpp_min_lookahead_m'], self.p['rpp_max_lookahead_m'])
        current_path_curvature = geometry.pure_pursuit_curvature(
            index, direction_sign, lookahead)
        feedback = rpp_curvature - current_path_curvature
        # Measured per-direction lag (2026-07-11 step experiment) replaces the
        # online delay estimate; the estimator still runs for telemetry.
        delay = self._steering_lag(direction, target_speed)
        side = 'negative' if rpp_curvature < 0.0 else 'positive'
        branch_confidence = self._model_confidence(
            self.steering_models[f'{direction}_{side}'])
        blend = branch_confidence
        preview_curvature = current_path_curvature
        preview_distance = 0.0
        horizon = delay
        # Two fixed-point passes account for time needed by the configured
        # command slew without introducing a hand-chosen preview distance.
        candidate = rpp_curvature
        for _ in range(2):
            pulse = self._steering_pulse(direction, candidate)
            slew_time = (abs(pulse - self.steering_us) /
                         self.p['steering_slew_us_per_s'])
            horizon = delay + slew_time
            requested_distance = abs(target_speed) * horizon
            preview_index, preview_distance, _ = geometry.advance(
                index, requested_distance, direction_sign)
            preview_curvature = geometry.pure_pursuit_curvature(
                preview_index, direction_sign, lookahead)
            candidate = preview_curvature + feedback
        applied_blend = (
            blend if self.p['apply_path_preview_compensation'] else 0.0)
        compensated = compose_preview_curvature(
            rpp_curvature, current_path_curvature, preview_curvature,
            applied_blend, self.p['maximum_curvature_1pm'])
        assist_active = False
        result.update({
            'path_curvature_now_1pm': current_path_curvature,
            'path_curvature_preview_1pm': preview_curvature,
            'rpp_feedback_curvature_1pm': feedback,
            'preview_distance_m': preview_distance,
            'preview_horizon_s': horizon,
            'preview_blend': applied_blend,
            'preview_curvature_1pm': compensated,
            'steering_assist_active': int(assist_active),
            'steering_command_source': (
                'path_floor' if assist_active else 'rpp'),
        })
        self.preview = result
        return compensated

    def _baseline_curvature_from_pulse(self, direction, pulse_us):
        inverse = sorted((pulse, curvature)
                         for curvature, pulse in self.steering_maps[direction])
        return interpolate(inverse, pulse_us)

    def _mean_delayed_baseline(self, start, end, delay, direction):
        values = [
            self._baseline_curvature_from_pulse(direction, pulse)
            for stamp, pulse, pulse_direction in self.pulse_history
            if start - delay <= stamp <= end - delay and
            pulse_direction == direction]
        if len(values) < 3:
            return None
        return sum(values) / len(values)

    def _causal_throttle_pulse(self, stamp, direction):
        candidates = [item for item in self.throttle_history
                      if item[2] == direction]
        if not candidates:
            return None
        return min(candidates, key=lambda item: abs(item[0] - stamp))[1]

    def _learn_breakaway(self, now, direction):
        if math.isfinite(self.startup_kick_pulse_us):
            # Pulse held when raw motion first appeared: the direct causal
            # attribution when the drop-on-raw-evidence path already lowered
            # the throttle before the robust observer confirmed breakaway.
            causal = self.startup_kick_pulse_us
        else:
            causal = self._causal_throttle_pulse(
                now - self.p[f'steering_lag_time_{direction}_s'], direction)
        if causal is None:
            return
        if ((direction == 'forward' and not 1000. <= causal < 1500.) or
                (direction == 'reverse' and not 1500. < causal <= 2000.)):
            return
        model = self.breakaway_models[direction]
        forgetting = (
            self.p['session_warmup_forgetting']
            if self.session_launches[direction] <
            self.p['session_warmup_launches']
            else self.p['breakaway_forgetting_factor'])
        model['pulse_us'] = (forgetting * model['pulse_us'] +
                             (1.0 - forgetting) * causal)
        model['observations'] += 1
        self.session_launches[direction] += 1
        self.startup_causal_pulse_us = causal

    def _flush_held_streak(self, direction):
        if self.floor_streak >= 4 and math.isfinite(self.floor_streak_min):
            model = self.floor_observer[direction]
            model['held_ema_mps'] = (
                self.floor_streak_min if model['held_observations'] == 0 else
                .9 * model['held_ema_mps'] + .1 * self.floor_streak_min)
            model['held_observations'] += 1
        self.floor_streak = 0
        self.floor_streak_min = math.inf

    def _update_effort_scale(self, direction, now):
        # Live pulse vs the map at the measured speed. The launch-transient
        # leak (g diving after hot launches) is handled by the rolling-age
        # gate at the call site, NOT by removing the P-term: the trim-based
        # variant was tautological (ratio ~= 1 by construction) and froze the
        # estimator at 1.000 for an entire drive (21:34 run).
        neutral = self.p['throttle_neutral_us']
        map_offset = interpolate(self.throttle_maps[direction],
                                 abs(self.speed)) - neutral
        if abs(map_offset) < 20.0:
            return
        observed = clamp((self.throttle_us - neutral) / map_offset, 0.7, 1.3)
        dt = max(0.0, now - self.effort_scale_time)
        self.effort_scale_time = now
        self.effort_scale_var += self.p['effort_scale_process_per_s'] * dt
        noise = self.p['effort_scale_measurement_sd'] ** 2
        gain = self.effort_scale_var / (self.effort_scale_var + noise)
        self.effort_scale += gain * (observed - self.effort_scale)
        self.effort_scale_var *= (1.0 - gain)

    def _throttle_slope(self, direction, speed):
        probe = .01
        points = self.throttle_maps[direction]
        return abs((interpolate(points, speed + probe) -
                    interpolate(points, speed - probe)) / (2.0 * probe))

    def _floor_estimate(self, direction):
        model = self.floor_observer[direction]
        if model['died_observations'] > 0:
            return clamp(model['died_ema_mps'] + .02, .08, .18)
        if model['held_observations'] > 0:
            return clamp(model['held_ema_mps'] - .01, .08, .18)
        return self.p['minimum_sustain_speed_mps']

    def _tick(self):
        now = time.monotonic(); dt = max(.001, now - self.last_tick); self.last_tick = now
        fresh_odom = self.odom_sequence != self.controlled_odom_sequence
        odom_dt = 0.0
        if fresh_odom:
            if self.last_control_odom_stamp is not None:
                odom_dt = clamp(
                    self.last_tf_stamp - self.last_control_odom_stamp, .001, .5)
            self.last_control_odom_stamp = self.last_tf_stamp
            self.controlled_odom_sequence = self.odom_sequence
        target, curvature, stop_reason = self._desired()
        sign_now = 0 if target == 0. else (1 if target > 0. else -1)
        if sign_now != 0 and sign_now != self.last_cmd_sign:
            self.last_cmd_sign = sign_now
            self.cmd_sign_since = now
        if (not stop_reason and sign_now != 0 and
                now - self.cmd_sign_since <
                self.p['direction_flip_debounce_s'] and
                (self.direction is None or
                 (self.direction == 'forward') != (sign_now > 0))):
            stop_reason = 'direction settle'
        rpp_curvature = curvature
        if stop_reason or abs(self.speed) > self.p['maximum_measured_speed_mps']:
            self.fault = stop_reason or 'measured overspeed'; self.state = 'stopped'
            self.direction = None; self.low_samples = self.limit_dwell = 0
            self.gentle_motion = False
            self.gentle_coast_active = False
            self.gentle_pulse_active = False
            target_throttle = self.p['throttle_neutral_us']
            target_steering = self.p['steering_center_us']; self.learn_start = None
            self.preview = self._empty_preview()
            self.throttle_antiwindup_state = 'inactive'
        else:
            self.fault = ''; direction = 'forward' if target > 0 else 'reverse'
            sign = 1. if target > 0 else -1.
            if direction != self.direction:
                changing_while_moving = (
                    sign * self.speed < -self.p['motion_threshold_mps'])
                self.direction = direction
                self.state = ('direction_change' if changing_while_moving
                              else 'startup')
                self.low_samples = 0
                self.startup_last_effort_time = None
                self.startup_effort_attempts = 0
                self.startup_causal_pulse_us = math.nan
                self.startup_kick_pulse_us = math.nan
                self.gentle_coast_active = False
                self.gentle_pulse_active = False
                self.recent_rolling_speeds.clear()
                self.floor_streak = 0
                self.floor_streak_min = math.inf
                base = interpolate(self.throttle_maps[direction], abs(target))
                self.throttle_us = (
                    self.p['throttle_neutral_us'] if changing_while_moving else
                    (min(1425., base + 4.) if sign > 0 else max(1575., base - 4.)))
            curvature = self._preview_command(
                rpp_curvature, target, direction)
            target_steering = self._steering_pulse(direction, curvature)
            # Pre-monitor command shaping already accounted for this servo
            # traversal. Never mutate an approved nonzero Twist downstream.
            steering_error_us = abs(target_steering - self.steering_us)
            steering_transition = (
                steering_error_us > self.p['steering_slowdown_error_us'])
            directional_speed = sign * self.speed
            self.segment_remaining_m = self._segment_remaining()
            self.gentle_motion = gentle_motion_requested(
                self.planner_cmd.linear.x, self.segment_remaining_m,
                self.p['gentle_request_max_mps'],
                self.p['gentle_segment_length_m'],
                active_segment=self.control_segment_samples is not None)
            if not self.gentle_motion:
                self.gentle_coast_active = False
                self.gentle_pulse_active = False
            elif self.state == 'rolling':
                # A newly gentle request must shed rolling energy before any
                # additional launch effort is allowed.
                self.state = 'startup'
                self.gentle_coast_active = True
                self.gentle_coast_since = now
            # Stage-2 probe scheduler: alternate-sign steering taps during
            # boring cruising, gated so they never coincide with anything
            # that matters. Measure-only; estimators consume them in stage 3.
            self.probe_offset_us = 0.0
            if self.p['probe_enabled'] and self.state == 'rolling':
                if now < self.probe_until:
                    self.probe_offset_us = (
                        self.probe_sign * self.p['probe_amplitude_us'])
                else:
                    if self.probe_until > self.probe_last_end:
                        self.probe_last_end = self.probe_until
                    clearance = (self.closest_forward if sign > 0
                                 else self.closest_reverse)
                    stop_ahead = min(
                        self.preview.get('cusp_distance_m', math.inf),
                        self.preview.get('path_end_distance_m', math.inf))
                    lateral = self.preview.get('cross_track_error_m', math.inf)
                    if (not steering_transition and
                            abs(curvature) < self.p['probe_max_curvature_1pm'] and
                            abs(target) >= 0.18 and
                            directional_speed >=
                            self.p['minimum_sustain_speed_mps'] and
                            clearance > (self.required_stop_clearance +
                                         self.p['probe_clearance_margin_m']) and
                            stop_ahead > self.p['probe_min_stop_distance_m'] and
                            (not math.isfinite(lateral) or abs(lateral) < .05) and
                            now - self.probe_last_end >=
                            self.p['probe_interval_s']):
                        self.probe_sign = -self.probe_sign
                        self.probe_id += 1
                        self.probe_until = now + self.p['probe_duration_s']
                        self.probe_offset_us = (
                            self.probe_sign * self.p['probe_amplitude_us'])
            target_steering += self.probe_offset_us
            base = interpolate(self.throttle_maps[direction], abs(target))
            trim_key = f'throttle_{direction}'
            # The actuator is refreshed at 50 Hz, but adaptation advances only
            # when TF supplies a new velocity measurement (normally ~10 Hz).
            # Reusing a sample here caused false stalls and bursty recovery.
            if not fresh_odom:
                target_throttle = self.throttle_us
                self.throttle_antiwindup_state = 'hold_no_fresh_odom'
            elif self.state == 'direction_change':
                target_throttle = self.p['throttle_neutral_us']
                self.throttle_antiwindup_state = 'frozen_direction_change'
                if directional_speed >= -self.p['motion_threshold_mps']:
                    self.state = 'startup'
                    target_throttle = (min(1425., base + 4.) if sign > 0
                                       else max(1575., base - 4.))
            elif self.state == 'planned_stop':
                # Coast at neutral while steering keeps tracking, which
                # pre-positions the wheels for the next path segment. The
                # direction flip or a zero command exits this state; if the
                # same-direction command persists, fall back to recovery.
                target_throttle = self.p['throttle_neutral_us']
                self.throttle_antiwindup_state = 'planned_stop'
                if directional_speed >= self.p['breakaway_threshold_mps']:
                    self.state = 'rolling'
                    self.rolling_since = now
                elif (now - self.planned_stop_since >
                        self.p['planned_stop_wait_s']):
                    self.state = 'recovery'
                    self.low_samples = self.limit_dwell = 0
            elif self.state in ('startup', 'recovery'):
                self.throttle_antiwindup_state = f'frozen_{self.state}'
                raw_motion = (
                    not self.odom_outlier and
                    sign * self.raw_speed >= self.p['motion_threshold_mps'])
                if (self.gentle_motion and
                        steering_error_us >
                        self.p['gentle_steering_ready_us']):
                    # Steering first: a low-speed correction should not spend
                    # its launch impulse while the wheels are still traversing.
                    target_throttle = self.p['throttle_neutral_us']
                    self.throttle_antiwindup_state = 'gentle_presteer'
                elif self.gentle_motion and self.gentle_coast_active:
                    self.gentle_pulse_active = False
                    target_throttle = self.p['throttle_neutral_us']
                    self.throttle_antiwindup_state = 'gentle_coast'
                    if (now - self.gentle_coast_since >=
                            self.p['gentle_min_coast_s'] and
                            directional_speed <=
                            self.p['gentle_rearm_speed_mps']):
                        self.gentle_coast_active = False
                        self.startup_last_effort_time = None
                        self.startup_effort_attempts = 0
                elif (self.gentle_motion and
                      (directional_speed >=
                       self.p['breakaway_threshold_mps'] or raw_motion)):
                    if not math.isfinite(self.startup_kick_pulse_us):
                        self.startup_kick_pulse_us = self.throttle_us
                    if self.state == 'startup':
                        self._learn_breakaway(now, direction)
                    self.gentle_coast_active = True
                    self.gentle_coast_since = now
                    self.gentle_pulse_active = False
                    target_throttle = self.p['throttle_neutral_us']
                    self.throttle_antiwindup_state = 'gentle_motion_detected'
                elif directional_speed >= self.p['breakaway_threshold_mps']:
                    if self.state == 'startup':
                        self._learn_breakaway(now, direction)
                    self.state = 'rolling'; self.low_samples = self.limit_dwell = 0
                    self.rolling_since = now
                    target_throttle = base + self.trim[trim_key]
                else:
                    # Weaker than sustain during the confirmation window: the
                    # ESC is still unwinding breakaway effort, and dropping
                    # only to the sustain pulse left a 0.5-0.65 m/s tail.
                    # Specified in speed units and converted via the local map
                    # slope: a fixed microsecond offset was worth ~2x more
                    # speed in reverse (shallower slope) and caused post-launch
                    # reverse stalls in the 15:27 drive.
                    drop_us = (self.p['startup_drop_through_mps'] *
                               self._throttle_slope(direction, abs(target)))
                    drop_pulse = base + self.trim[trim_key] + (
                        drop_us if sign > 0 else -drop_us)
                    if self.state == 'startup':
                        learned = self.breakaway_models[direction]['pulse_us']
                        if (self.session_launches[direction] <
                                self.p['session_warmup_launches']):
                            # Re-feel the pedal: approach the remembered
                            # threshold from the weak side this session.
                            learned += (
                                self.p['session_warmup_weaken_us'] if sign > 0
                                else -self.p['session_warmup_weaken_us'])
                        response_time = max(
                            1.0 / self.p['control_rate_hz'], odom_dt,
                            self.p[f'steering_lag_time_{direction}_s'])
                        weaker_than_learned = (
                            self.throttle_us > learned if sign > 0 else
                            self.throttle_us < learned)
                        if raw_motion:
                            # Motion is visible in the newest derivative before
                            # the robust observer confirms it. Breakaway effort
                            # held through the confirmation window is what
                            # produced 0.41-0.7 m/s launches at a 0.3 target:
                            # drop to the rolling feedforward immediately and
                            # let the ramp branch re-arm if this was noise.
                            if not math.isfinite(self.startup_kick_pulse_us):
                                self.startup_kick_pulse_us = self.throttle_us
                            target_throttle = drop_pulse
                            self.throttle_antiwindup_state = 'startup_raw_drop'
                        elif weaker_than_learned:
                            gap = abs(learned - self.throttle_us)
                            step = max(TICK_US, gap * odom_dt / response_time)
                            target_throttle = self.throttle_us + (-step if sign > 0 else step)
                            if ((sign > 0 and target_throttle <= learned) or
                                    (sign < 0 and target_throttle >= learned)):
                                target_throttle = learned
                                self.startup_last_effort_time = now
                        elif (self.startup_last_effort_time is None or
                              now - self.startup_last_effort_time >= response_time):
                            self.startup_effort_attempts += 1
                            step = min(
                                self.p['recovery_step_us'],
                                TICK_US * self.startup_effort_attempts)
                            target_throttle = self.throttle_us + (-step if sign > 0 else step)
                            self.startup_last_effort_time = now
                        else:
                            target_throttle = self.throttle_us
                    elif raw_motion:
                        target_throttle = drop_pulse
                        self.throttle_antiwindup_state = 'recovery_raw_drop'
                    else:
                        target_throttle = self.throttle_us + (
                            -self.p['recovery_step_us'] if sign > 0 else
                            self.p['recovery_step_us'])
                    if self.gentle_motion:
                        learned = self.breakaway_models[direction]['pulse_us']
                        target_throttle = limit_gentle_launch_pulse(
                            target_throttle, learned, sign > 0,
                            self.p['gentle_launch_max_extra_us'])
                        if not self.gentle_pulse_active:
                            self.gentle_pulse_active = True
                            self.gentle_pulse_since = now
                        elif (now - self.gentle_pulse_since >=
                              self.p['gentle_max_pulse_s']):
                            # Even without a velocity observation, a gentle
                            # request gets bounded impulse, never a held kick.
                            self.gentle_pulse_active = False
                            self.gentle_coast_active = True
                            self.gentle_coast_since = now
                            target_throttle = self.p['throttle_neutral_us']
                            self.throttle_antiwindup_state = (
                                'gentle_pulse_timeout')
                    limit = self.p['forward_recovery_limit_us'] if sign > 0 else self.p['reverse_recovery_limit_us']
                    target_throttle = max(limit, target_throttle) if sign > 0 else min(limit, target_throttle)
                    at_limit = target_throttle == limit
                    self.limit_dwell = self.limit_dwell + 1 if at_limit else 0
                    if self.limit_dwell >= self.p['recovery_limit_dwell_samples']:
                        self.fault = 'traction authority exhausted'; target_throttle = self.p['throttle_neutral_us']; self.state = 'stopped'
            else:
                self.recent_rolling_speeds.append(directional_speed)
                # Only converged samples measure the plant; transients measure
                # the controller chasing it. The 1.5 s rolling-age gate keeps
                # launch/drop transients out.
                if (directional_speed > self.p['motion_threshold_mps'] and
                        abs(directional_speed - abs(target)) < 0.06 and
                        now - self.rolling_since > 1.5):
                    self._update_effort_scale(direction, now)
                if .05 <= directional_speed <= .22:
                    self.floor_streak += 1
                    self.floor_streak_min = min(self.floor_streak_min,
                                                directional_speed)
                elif directional_speed > .22:
                    self._flush_held_streak(direction)
                error = target - self.speed
                kp = self.p['throttle_kp_us_per_mps']
                if kp <= 0.0:
                    kp = self._throttle_slope(direction, abs(target))
                ki = self.p['throttle_ki_us_per_m']
                low, high = ((self.p['forward_rolling_min_us'], 1425.)
                             if sign > 0 else
                             (1575., self.p['reverse_rolling_max_us']))
                unsaturated = base + self.trim[trim_key] - kp * error
                saturated = clamp(unsaturated, low, high)
                if steering_transition:
                    self.throttle_antiwindup_state = 'frozen_steering_transition'
                elif directional_speed < self.p['motion_threshold_mps']:
                    self.throttle_antiwindup_state = 'frozen_low_motion'
                elif saturated != unsaturated and kp > 0.0:
                    # Standard back calculation. Ki/Kp supplies the inverse-time
                    # gain from the existing controller rather than another
                    # independently tuned constant.
                    self.trim[trim_key] += (
                        (ki / kp) * (saturated - unsaturated) * odom_dt)
                    self.throttle_antiwindup_state = 'back_calculation'
                else:
                    self.trim[trim_key] -= ki * error * odom_dt
                    self.throttle_antiwindup_state = 'integrating'
                self.trim[trim_key] = clamp(
                    self.trim[trim_key],
                    -self.p['maximum_throttle_trim_us'],
                    self.p['maximum_throttle_trim_us'])
                desired = base + self.trim[trim_key] - kp * error
                desired = clamp(desired, low, high)
                slew = self.p['throttle_rolling_slew_us_per_odom']
                target_throttle = self.throttle_us + clamp(
                    desired - self.throttle_us, -slew, slew)
                self.low_samples = self.low_samples + 1 if directional_speed < self.p['motion_threshold_mps'] else 0
                # Breakaway threshold, not motion threshold: median-sign
                # noise at near-zero speed tripped 8 full-stop faults in the
                # 221 s run; genuine backward rolling crosses 0.045 quickly.
                if directional_speed < -self.p['breakaway_threshold_mps']:
                    self.fault = 'direction mismatch'; target_throttle = self.p['throttle_neutral_us']; self.state = 'stopped'
                elif self.low_samples >= 3:
                    stop_ahead = min(
                        self.preview.get('cusp_distance_m', math.inf),
                        self.preview.get('path_end_distance_m', math.inf))
                    if stop_ahead < self.p['planned_stop_cusp_m']:
                        # Stalled within reach of an intended stop: no kick,
                        # no died-evidence — coast and wait for the reversal.
                        self.state = 'planned_stop'
                        self.planned_stop_since = now
                        self.low_samples = 0
                        self.recent_rolling_speeds.clear()
                        self.floor_streak = 0
                        self.floor_streak_min = math.inf
                    else:
                        # Died while commanded: the strongest floor evidence.
                        # Discard the active held-streak (its tail is decay).
                        if self.recent_rolling_speeds:
                            died_at = max(self.recent_rolling_speeds)
                            floor_model = self.floor_observer[direction]
                            floor_model['died_ema_mps'] = (
                                died_at
                                if floor_model['died_observations'] == 0
                                else .8 * floor_model['died_ema_mps'] +
                                .2 * died_at)
                            floor_model['died_observations'] += 1
                            self.recent_rolling_speeds.clear()
                        self.floor_streak = 0
                        self.floor_streak_min = math.inf
                        self.state = 'recovery'; self.low_samples = self.limit_dwell = 0; self.learn_start = None
        max_step = self.p['steering_slew_us_per_s'] * dt
        self.steering_us += clamp(target_steering - self.steering_us, -max_step, max_step)
        self.throttle_us = target_throttle
        self.pulse_history.append(
            (now, self.steering_us, self.direction or ''))
        self.throttle_history.append(
            (now, self.throttle_us, self.direction or ''))
        if self.hardware:
            self.hardware.pulse(self.p['steering_channel'], self.steering_us)
            self.hardware.pulse(self.p['throttle_channel'], self.throttle_us)
        if now - self.last_save > 10.:
            self._save_runtime(); self.last_save = now
        self.drive_writer.writerow({
            'monotonic_s': f'{now:.6f}', 'fresh_odom': int(fresh_odom),
            'state': self.state, 'fault': self.fault,
            'direction': self.direction or '',
            'planner_v_mps': self.planner_cmd.linear.x,
            'planner_w_radps': self.planner_cmd.angular.z,
            'limited_v_mps': self.limited_cmd.linear.x,
            'limited_w_radps': self.limited_cmd.angular.z,
            'command_gate_reason': self.command_gate_reason,
            'path_progress_index': self.path_progress_index,
            'path_segment_end_index': self.path_segment_end,
            'post_collision_v_mps': self.cmd.linear.x,
            'post_collision_w_radps': self.cmd.angular.z,
            'planner_command_age_s': (
                now - self.planner_cmd_time
                if self.planner_cmd_time is not None else math.inf),
            'post_collision_command_age_s': (
                now - self.cmd_time if self.cmd_time is not None else math.inf),
            'accepted_v_mps': target,
            'rpp_curvature_1pm': rpp_curvature,
            'accepted_curvature_1pm': curvature,
            'measured_v_mps': self.speed, 'measured_w_radps': self.yaw_rate,
            'raw_v_mps': self.raw_speed, 'raw_w_radps': self.raw_yaw_rate,
            'odom_outlier': int(self.odom_outlier),
            'throttle_us': self.throttle_us, 'steering_us': self.steering_us,
            'closest_obstacle_m': self.closest,
            'front_clearance_m': self.closest_forward,
            'rear_clearance_m': self.closest_reverse,
            'low_samples': self.low_samples, 'limit_dwell': self.limit_dwell,
            'curvature_error_ema': self.curvature_error_ema,
            'steering_rls_models_json': json.dumps(
                self.steering_models, separators=(',', ':')),
            'estimated_steering_delay_s': self.estimated_steering_delay_s,
            'steering_delay_confidence': self.delay_estimator.confidence,
            'steering_delay_weights_json': json.dumps(
                self.delay_estimator.state()['weights'], separators=(',', ':')),
            'throttle_antiwindup_state': self.throttle_antiwindup_state,
            'required_stop_clearance_m': self.required_stop_clearance,
            'breakaway_models_json': json.dumps(
                self.breakaway_models, separators=(',', ':')),
            'startup_effort_attempts': self.startup_effort_attempts,
            'startup_causal_pulse_us': self.startup_causal_pulse_us,
            'floor_estimate_forward_mps': (
                f'{self._floor_estimate("forward"):.4f}'),
            'floor_estimate_reverse_mps': (
                f'{self._floor_estimate("reverse"):.4f}'),
            'floor_observer_json': json.dumps(
                self.floor_observer, separators=(',', ':')),
            'probe_id': self.probe_id,
            'probe_offset_us': f'{self.probe_offset_us:.1f}',
            'effort_scale': f'{self.effort_scale:.4f}',
            'effort_scale_sd': f'{math.sqrt(self.effort_scale_var):.4f}',
            'segment_remaining_m': self.segment_remaining_m,
            'gentle_motion': int(self.gentle_motion),
            'gentle_coast_active': int(self.gentle_coast_active),
            'gentle_pulse_active': int(self.gentle_pulse_active),
            **self.preview,
            **{f'{key}_trim_us': value for key, value in self.trim.items()}})

    def _learn_steering(self, x, y, yaw):
        now = time.monotonic()
        if (self.direction is None or self.state != 'rolling' or self.fault or
                abs(self.speed) < self.p['steering_learning_min_speed_mps']):
            self.motion_history.clear()
            return
        self.motion_history.append((now, x, y, yaw, self.direction))
        window_s = self.p['steering_identification_window_s']
        while (len(self.motion_history) > 2 and
               now - self.motion_history[0][0] > window_s + .20):
            self.motion_history.popleft()
        if now - self.last_identification_time < .25:
            return
        candidates = [sample for sample in self.motion_history
                      if now - sample[0] <= window_s]
        if len(candidates) < 4 or now - candidates[0][0] < .35:
            return
        direction = self.direction
        if any(sample[4] != direction for sample in candidates):
            self.motion_history.clear()
            return
        signed_distance = 0.0
        yaw_change = 0.0
        for previous, current in zip(candidates, candidates[1:]):
            _, px, py, pyaw, _ = previous
            _, cx, cy, cyaw, _ = current
            signed_distance += (math.cos(pyaw) * (cx - px) +
                                math.sin(pyaw) * (cy - py))
            yaw_change += angle_difference(cyaw, pyaw)
        if abs(signed_distance) < self.p['steering_identification_min_distance_m']:
            return
        measured = yaw_change / signed_distance
        if not math.isfinite(measured) or abs(measured) > 2.5:
            return
        start, end = candidates[0][0], candidates[-1][0]
        available = {}
        squared_errors = {}
        for delay in self.delay_candidates:
            baseline = self._mean_delayed_baseline(
                start, end, delay, direction)
            if baseline is None:
                continue
            available[delay] = baseline
            candidate_side = 'negative' if baseline < 0.0 else 'positive'
            candidate_model = self.steering_models[
                f'{direction}_{candidate_side}']
            residual = measured - (
                candidate_model['gain'] * baseline +
                candidate_model['bias'])
            squared_errors[delay] = residual * residual
        if not available:
            return
        self.delay_estimator.update(squared_errors)
        self.estimated_steering_delay_s = self.delay_estimator.estimate
        baseline = self._mean_delayed_baseline(
            start, end, self.estimated_steering_delay_s, direction)
        if baseline is None:
            weights = self.delay_estimator.weights()
            usable_weight = sum(weights.get(delay, 0.0) for delay in available)
            if usable_weight <= 0.0:
                return
            baseline = sum(available[delay] * weights.get(delay, 0.0)
                           for delay in available) / usable_weight
        if abs(baseline) < .10:
            return
        side = 'negative' if baseline < 0.0 else 'positive'
        model = self.steering_models[f'{direction}_{side}']
        phi0, phi1 = baseline, 1.0
        p = model['covariance']
        p_phi0 = p[0][0] * phi0 + p[0][1] * phi1
        p_phi1 = p[1][0] * phi0 + p[1][1] * phi1
        forgetting = self.p['steering_rls_forgetting_factor']
        denominator = forgetting + phi0 * p_phi0 + phi1 * p_phi1
        k0, k1 = p_phi0 / denominator, p_phi1 / denominator
        prediction = model['gain'] * phi0 + model['bias']
        innovation = clamp(measured - prediction, -.60, .60)
        model['gain'] = clamp(model['gain'] + k0 * innovation, .40, 1.60)
        model['bias'] = clamp(model['bias'] + k1 * innovation, -.35, .35)
        row0 = [p[0][0] - k0 * (phi0 * p[0][0] + phi1 * p[1][0]),
                p[0][1] - k0 * (phi0 * p[0][1] + phi1 * p[1][1])]
        row1 = [p[1][0] - k1 * (phi0 * p[0][0] + phi1 * p[1][0]),
                p[1][1] - k1 * (phi0 * p[0][1] + phi1 * p[1][1])]
        model['covariance'] = [[value / forgetting for value in row0],
                               [value / forgetting for value in row1]]
        model['observations'] += 1
        absolute_error = abs(measured - prediction)
        model['residual_ema'] = .90 * model['residual_ema'] + .10 * absolute_error
        self.curvature_error_ema = (
            absolute_error if self.curvature_observations == 0 else
            .90 * self.curvature_error_ema + .10 * absolute_error)
        self.curvature_observations += 1
        self.last_identification_time = now

    def _publish_status(self):
        data = {'armed': bool(self.hardware), 'state': self.state, 'fault': self.fault,
                'raw_nav_v': self.planner_cmd.linear.x,
                'raw_nav_w': self.planner_cmd.angular.z,
                'limited_nav_v': self.limited_cmd.linear.x,
                'limited_nav_w': self.limited_cmd.angular.z,
                'command_gate_reason': self.command_gate_reason,
                'path_progress_index': self.path_progress_index,
                'path_segment_end_index': self.path_segment_end,
                'path_segment_direction': self.path_segment_direction,
                'target_v': self.cmd.linear.x, 'target_w': self.cmd.angular.z,
                'speed': self.speed, 'yaw_rate': self.yaw_rate,
                'raw_speed': self.raw_speed, 'raw_yaw_rate': self.raw_yaw_rate,
                'odom_outlier': self.odom_outlier,
                'throttle_us': self.throttle_us, 'steering_us': self.steering_us,
                'closest_obstacle_m': self.closest, 'trim_us': self.trim,
                'front_clearance_m': self.closest_forward,
                'rear_clearance_m': self.closest_reverse,
                'steering_rls_models': self.steering_models,
                'steering_model_confidence': {
                    key: self._model_confidence(model)
                    for key, model in self.steering_models.items()},
                'estimated_steering_delay_s': self.estimated_steering_delay_s,
                'steering_delay_estimator': self.delay_estimator.state(),
                'preview': self.preview,
                'throttle_antiwindup_state': self.throttle_antiwindup_state,
                'required_stop_clearance_m': self.required_stop_clearance,
                'breakaway_models': self.breakaway_models,
                'startup_effort_attempts': self.startup_effort_attempts,
                'startup_causal_pulse_us': self.startup_causal_pulse_us,
                'floor_observer': self.floor_observer,
                'floor_estimates_mps': {
                    direction: self._floor_estimate(direction)
                    for direction in ('forward', 'reverse')},
                'segment_remaining_m': self.segment_remaining_m,
                'gentle_motion': self.gentle_motion,
                'gentle_coast_active': self.gentle_coast_active,
                'gentle_pulse_active': self.gentle_pulse_active,
                'curvature_error_ema': self.curvature_error_ema}
        self.debug_pub.publish(String(data=json.dumps(data)))
        # Keep the Nav2 limit deterministic while identification is active.
        # Feeding estimator residual back into speed confounds whether model or
        # speed caused an apparent improvement. Steering transition slowdown
        # remains local and directly observable in the flight recorder.
        limit = min(self.p['maximum_forward_speed_mps'],
                    self.p['navigation_speed_limit_mps'])
        speed_limit = SpeedLimit(); speed_limit.header.stamp = self.get_clock().now().to_msg()
        speed_limit.percentage = False; speed_limit.speed_limit = max(.08, limit)
        self.speed_limit_pub.publish(speed_limit)
        status = DiagnosticStatus(); status.name = 'adaptive_ackermann_controller'
        status.hardware_id = 'pca9685'; status.level = DiagnosticStatus.ERROR if self.fault else DiagnosticStatus.OK
        status.message = self.fault or self.state
        status.values = [KeyValue(key=k, value=str(v)) for k, v in data.items() if k != 'trim_us']
        array = DiagnosticArray(); array.header.stamp = self.get_clock().now().to_msg(); array.status = [status]
        self.diag_pub.publish(array)

    def destroy_node(self):
        try:
            self._save_runtime()
            if self.hardware:
                self.hardware.pulse(self.p['throttle_channel'], self.p['throttle_neutral_us'])
                self.hardware.release(self.p['steering_channel']); self.hardware.close()
        finally:
            self.drive_log.close()
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args); node = AdaptiveAckermannController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
