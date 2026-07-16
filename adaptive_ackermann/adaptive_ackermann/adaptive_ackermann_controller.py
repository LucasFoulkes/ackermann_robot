#!/usr/bin/env python3
"""Self-learning bridge from Twist commands to normalized actuator efforts."""

import csv
from collections import deque
import json
import math
import os
import tempfile
import time

import rclpy
import yaml
from adaptive_ackermann.adaptive_model import (
    DelayEstimator, PathGeometry,
    gentle_motion_requested, isotonic_points, limit_ackermann_twist,
    limit_gentle_launch_effort, polyline_projection, scan_point_clearance,
    stopping_clearance)
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from nav2_msgs.msg import SpeedLimit
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from adaptive_ackermann_msgs.msg import EffortCommand

# Efforts are normalized to [-1, 1]; the hardware driver maps them onto the
# birth-certificate pulse endpoints. 0.004 effort is ~one PWM tick on the
# reference 197 Hz PCA9685 -- the smallest command increment worth taking.
MINIMUM_EFFORT_STEP = 0.004


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
    # Duplicate x knots (e.g. an inverted map with pooled values) must
    # degrade to a step, never divide by zero: this exact division
    # killed the controller process mid-drive on 2026-07-15.
    span = b[0] - a[0]
    if span <= 0.0:
        return b[1]
    fraction = (x - a[0]) / span
    return a[1] + fraction * (b[1] - a[1])


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_difference(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class AdaptiveAckermannController(Node):
    def __init__(self):
        super().__init__('adaptive_ackermann_controller')
        defaults = {
            'control_rate_hz': 50.0,
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
            'throttle_kp_effort_per_mps': 0.0, 'throttle_ki_effort_per_m': 0.094,
            'throttle_rolling_slew_per_odom': 0.019,
            'forward_breakaway_prior_effort': -0.303,
            'reverse_breakaway_prior_effort': 0.347,
            'breakaway_forgetting_factor': .90,
            # Session warm-up: battery state changes between sessions, so the
            # persisted breakaway is stale at boot (fresh charge after a
            # sagged evening produced 0.5-0.8 m/s launches for ~3 min on
            # 2026-07-11 19:59). First launches per direction start the ramp
            # weaker and re-learn the threshold with faster forgetting.
            'session_warmup_launches': 5,
            'session_warmup_weaken_effort': 0.019,
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
            # Throttle-target ramp rate after breakaway (m/s per s); the
            # sustain floor is the starting point, cruise is reached in
            # ~0.6 s at 0.8. Zero disables.
            'launch_target_ramp_mps2': 0.8,
            # Steering-loop speed budget: v_max = fraction * L_min / tau.
            # fraction 0.5 is the 2*v*tau pure-pursuit stability margin;
            # sensing lag covers MOLA + odometry filtering (measured ~0.15).
            'steering_budget_fraction': 0.5,
            'steering_budget_sensing_lag_s': 0.15,
            # EW-variance reference for breakaway alpha damping: variance
            # equal to this (~0.1 effort of scatter) halves the rate.
            'breakaway_variance_reference': 0.01,
            # Per-side executable curvature ceilings (0 = use the
            # symmetric maximum_curvature_1pm). Measured full lock
            # 2026-07-14: kappa 1.99 LEFT(+) / 1.40 RIGHT(-); the
            # symmetric clamp wastes the strong side (ODAAC steal #5).
            'maximum_curvature_positive_1pm': 0.0,
            'maximum_curvature_negative_1pm': 0.0,
            # Steering drag raises breakaway when launching INTO a turn
            # (2026-07-14 follow sessions: 24-37% of curve-commanded time
            # spent stalled; the throttle model is 1-D by measurement, but
            # that measurement was at |k| <= 0.8). Scale launch efforts by
            # (1 + boost * |curvature|); drop-on-raw-evidence still bounds
            # any overshoot.
            'launch_curvature_boost': 0.15,
            # Set the wheels BEFORE moving: a 3-point-turn leg that launches
            # mid-servo-traverse drives its first half-meter nearly straight
            # (observed 2026-07-14). Startup holds neutral until steering is
            # within this effort error of its target.
            'launch_presteer_ready_effort': 0.12,
            # The ESC cannot roll continuously at arbitrarily low speed. For
            # gentle requests and short committed segments, pre-steer, apply a
            # bounded learned breakaway pulse, then coast before re-arming.
            'gentle_request_max_mps': .10,
            # Disabled after armed evidence showed a fixed remaining-distance
            # transition defeated continuous progress near segment endpoints.
            # Nav2's goal checker owns stopping distance; normal learned
            # closed-loop throttle remains active until it succeeds.
            'gentle_segment_length_m': 0.0,
            'gentle_steering_ready_effort': 0.12,
            'gentle_min_coast_s': .35,
            'gentle_max_pulse_s': .45,
            'gentle_rearm_speed_mps': .025,
            'gentle_launch_max_extra_effort': 0.019,
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
            # straight cruising, for the identification estimators. A 0.016
            # effort tap (~8 us) is ~0.024 1/m for half a second: ~2 mm of
            # lateral deviation, well inside tracking noise. Measure-only:
            # taps are logged (probe_id / probe_offset_effort) but nothing
            # learns from them yet.
            # Stage F throttle probe: OPTIONAL diagnostic gain-feeler (user
            # decision 2026-07-13: throttle is learned from ordinary-driving
            # equilibria and failures, not probing; pipeline stays available
            # off the critical path).
            'throttle_probe_enabled': False,
            'throttle_probe_freq_hz': 0.9,
            # Rung order is pragmatic-first: gated cruise measured only
            # ~5 s/session on a weak pack in a small room, so the completable
            # short-loud rung leads; stealth rungs are for healthy packs and
            # bigger rooms (bench: 8us/5cyc=5.8%sd, 4/10=11%, 2/20=13%).
            'throttle_probe_ladder_effort': [0.025, 0.0125, 0.00625],
            'throttle_probe_ladder_cycles': [5, 10, 20],
            'throttle_probe_snr_min': 2.0,
            'throttle_probe_cooldown_s': 12.0,
            'throttle_probe_esc_tau_s': 0.25,
            'probe_enabled': False,
            'probe_amplitude_effort': 0.016,
            'probe_duration_s': 0.5,
            'probe_interval_s': 4.0,
            'probe_max_curvature_1pm': 0.15,
            'probe_clearance_margin_m': 0.30,
            'probe_min_stop_distance_m': 1.0,
            'maximum_throttle_trim_effort': 0.125, 'forward_rolling_min_effort': -0.375,
            'reverse_rolling_max_effort': 0.4375, 'forward_recovery_limit_effort': -1.0,
            'reverse_recovery_limit_effort': 1.0, 'recovery_step_effort': 0.031,
            'recovery_limit_dwell_samples': 10, 'steering_slew_per_s': 8.0,
            'steering_slowdown_error_effort': 0.06,
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
            # Max curvature RPP's error-feedback may add on top of the
            # path's own demand. Sized to the MEASURED ~0.4 s loop delay:
            # larger corrections limit-cycle instead of converging.
            'steering_feedback_cap_1pm': 0.40,
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
            'forward_throttle_map': [.2, -0.2519, .25, -0.2571],
            'reverse_throttle_map': [.2, 0.2891, .25, 0.2923],
            'forward_steering_map': [-1.15, -0.8337, 0., -0.0248, 1.15, 0.5605],
            'reverse_steering_map': [-1.15, -0.8488, 0., 0.0233, 1.15, 0.6127],
            # Stage D: learned steering map challenger. 'shadow' logs what the
            # learned map would command without changing behavior; 'blend'
            # applies it inside the trust region with the dense map as
            # automatic rollback champion.
            'learned_steering_mode': 'shadow',
            'learned_steering_map_path': '~/.robot/learned_steering_map.yaml',
            'learned_steering_trust_kappa_1pm': 0.6,
            'learned_steering_bootstrap_trust_kappa_1pm': 0.35,
            'learned_steering_min_knot_samples': 30,
            'learned_steering_rollback_error_1pm': 0.30,
            'learned_steering_rollback_grace_s': 20.0,
            # Fraction of battery-coupled learned state (effort scale, session
            # trims) retained across a process restart. No voltage sensor
            # means off-time is unmeasurable; 0.5 halves stale drift each
            # start while keeping same-session restarts cheap.
            'battery_state_retention': 0.5,
            # Learned two-point throttle feedforward (breakaway -> cruise
            # anchor), the ArduPilot-style minimal line. 'shadow' logs what
            # it would command; 'applied' uses it as the feedforward base
            # with the config map as fallback whenever anchors are thin.
            'learned_throttle_mode': 'shadow',
            'trim_deadzone_mps': 0.04,
            # Stage C: gradual effective-steering-state challenger (shadow).
            'learned_dynamics_mode': 'shadow',
            'learned_dynamics_path': '~/.robot/learned_steering_dynamics.yaml',
            'birth_certificate_path':
                '~/ros2_ws/src/ackermann_robot/ackermann_robot/config/birth_certificate.yaml',
            # Weak-plant adaptation (odom-only, no battery gauge by design):
            # a slow achievability ratio lowers the published Nav2 speed
            # limit toward what the pack/surface can actually sustain, and
            # repeated traction-authority give-ups latch a clean behavioral
            # stop instead of a fault->relaunch loop.
            'weak_plant_min_limit_mps': 0.15,
            'exhaustion_events_to_latch': 3,
            'exhaustion_window_s': 120.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        # Live-tunable speed envelope (ros2 param set, no restart): the
        # 1 Hz /speed_limit publication propagates the change to Nav2.
        self.dynamic_parameters = {
            'navigation_speed_limit_mps', 'maximum_forward_speed_mps',
            'maximum_reverse_speed_mps', 'maximum_measured_speed_mps',
            'maximum_curvature_1pm', 'maximum_curvature_positive_1pm',
            'maximum_curvature_negative_1pm'}
        self.add_on_set_parameters_callback(self._on_parameters)
        self._apply_birth_certificate()
        self.throttle_maps = {d: pairs(self.p[f'{d}_throttle_map'])
                              for d in ('forward', 'reverse')}
        self.steering_maps = {d: pairs(self.p[f'{d}_steering_map'])
                              for d in ('forward', 'reverse')}
        for d, mp in list(self.steering_maps.items()):
            projected, adjusted = isotonic_points(mp)
            if adjusted:
                self.get_logger().warning(
                    f'{d} champion steering map was non-monotonic — '
                    'isotonic projection applied')
                self.steering_maps[d] = projected
        self._load_learned_steering()
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
            'odom_outlier', 'throttle_effort', 'steering_effort',
            'closest_obstacle_m', 'front_clearance_m', 'rear_clearance_m',
            'low_samples', 'limit_dwell',
            'throttle_forward_trim', 'throttle_reverse_trim',
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
            'startup_causal_effort', 'floor_estimate_forward_mps',
            'floor_estimate_reverse_mps', 'floor_observer_json',
            'probe_id', 'probe_offset_effort',
            'effort_scale', 'effort_scale_sd',
            'segment_remaining_m', 'gentle_motion',
            'gentle_coast_active', 'gentle_pulse_active',
            'learned_steering_effort', 'learned_steering_state',
            'tracking_error_ema_1pm', 'applied_error_ema_1pm',
            'effective_kappa_1pm', 'dynamics_error_ema_1pm',
            'fixedlag_error_ema_1pm', 'achievable_speed_ratio',
            'cruise_anchor_json', 'learned_line_effort',
            'learned_throttle_state', 'trim_rate_fwd', 'trim_rate_rev',
            'steering_polarity', 'steering_sign_evidence',
            'throttle_probe_offset_effort', 'tprobe_phase_s'])
        self.drive_writer.writeheader()
        self.trim = {'throttle_forward': 0., 'throttle_reverse': 0.}
        self.curvature_error_ema = 0.0
        self.curvature_observations = 0
        self.tracking_error_ema = 0.0
        self.tracking_error_samples = 0
        self.applied_error_ema = 0.0
        self.applied_error_samples = 0
        self.achievable_ratio = 1.0
        self.exhaustion_events = deque(maxlen=8)
        self.pack_exhausted = False
        # Polarity is LEARNED, never supplied (birth-certificate rule): the
        # sign of the first motion/curvature responses identifies it. +1
        # means the generic convention (forward pulse below ESC neutral,
        # higher steering pulse = positive curvature) matches the plant.
        self.steering_polarity = 1.0
        self.throttle_polarity = 1.0
        self.steering_sign_evidence = 0
        self.wrong_direction_launches = 0
        self.tpol_wrong_streak = 0
        self.tpol_episode_counted = False
        self.cruise_anchor = {
            direction: {'effort': 0.0, 'speed_mps': 0.0,
                        'spread': 0.0, 'samples': 0}
            for direction in ('forward', 'reverse')}
        self.trim_rate = {'throttle_forward': 1.0, 'throttle_reverse': 1.0}
        self.trim_err_sign = {'throttle_forward': 0, 'throttle_reverse': 0}
        self.trim_err_streak = {'throttle_forward': 0, 'throttle_reverse': 0}
        self.learned_line_effort = 0.0
        self.learned_throttle_state = 'off'
        self.breakaway_models = {
            'forward': {'effort': self.p['forward_breakaway_prior_effort'],
                        'observations': 0},
            'reverse': {'effort': self.p['reverse_breakaway_prior_effort'],
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
        self.steering_history = deque(maxlen=600)
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
        self.speed = 0.; self.yaw_rate = 0.; self.last_odom_stamp = None
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
        self.limit_dwell = 0; self.throttle_effort = 0.0
        self.steering_effort = 0.0; self.last_tick = time.monotonic()
        self.learn_start = None; self.learn_target = None; self.last_save = time.monotonic()
        self.startup_last_effort_time = None
        self.startup_effort_attempts = 0
        self.startup_causal_effort = math.nan
        self.startup_kick_effort = math.nan
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
        self.rolling_entry_speed = 0.0
        self.last_commanded_curvature = 0.0
        self.feedback_ema = 0.0
        # ODAAC steal #4: every gated-out learning sample is counted by
        # reason; published at 1 Hz in /controller/debug. Rejection
        # statistics are themselves a diagnostic (a spike in one reason
        # is a fault signature, not noise).
        self.learning_rejections = {}
        self.probe_id = 0
        self.probe_sign = 1.0
        self.probe_until = 0.0
        self.probe_last_end = 0.0
        self.probe_offset = 0.0
        self.tprobe_active = False
        self.tprobe_phase_t = 0.0
        self.tprobe_paused_t = 0.0
        self.tprobe_level = 0
        self.tprobe_last_end = 0.0
        self.tprobe_offset = 0.0
        self.tprobe_direction = ''
        self.tprobe_samples = []
        self.tprobe_effort_sum = 0.0
        self.tprobe_effort_n = 0
        self.throttle_probe_results = []
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
        self.create_subscription(Odometry, '/odom', self._odom, 20)
        self.create_subscription(LaserScan, '/scan', self._scan, 10)
        self.effort_pub = self.create_publisher(
            EffortCommand, '/actuator_effort', 10)
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

    def _on_parameters(self, parameters):
        applied = []
        for parameter in parameters:
            if parameter.name not in self.dynamic_parameters:
                return SetParametersResult(
                    successful=False,
                    reason=f'{parameter.name} is not live-tunable; '
                           'edit config and restart')
            self.p[parameter.name] = float(parameter.value)
            applied.append(f'{parameter.name}={parameter.value}')
        if applied and hasattr(self, 'limit_pub'):
            self.get_logger().info('live speed update: ' + ', '.join(applied))
            self.limit_pub.publish(String(data=json.dumps({
                'max_forward_mps': self.p['maximum_forward_speed_mps'],
                'max_reverse_mps': self.p['maximum_reverse_speed_mps'],
                'navigation_speed_limit_mps':
                    self.p['navigation_speed_limit_mps'],
                'max_curvature_1pm': self.p['maximum_curvature_1pm'],
                'minimum_turning_radius_m':
                    1. / self.p['maximum_curvature_1pm']})))
        return SetParametersResult(successful=True)

    def _load_runtime(self):
        try:
            with open(self.runtime_path) as stream:
                saved = yaml.safe_load(stream) or {}
            # Version 2 trims were learned while Collision Monitor fragmented
            # commands and reached their authority limits. They are not valid
            # priors for the continuous estimator, so newer models start from the
            # experiment feed-forward maps without requiring a manual file edit.
            version = int(saved.get('version', 0))
            if version < 8:
                # Move the old file aside BEFORE the periodic save can
                # overwrite it (a 2026-07-13 smoke test clobbered real
                # memory exactly this way). The converter reads *.pre_effort.
                preserved = self.runtime_path + '.pre_effort'
                if not os.path.exists(preserved):
                    os.replace(self.runtime_path, preserved)
                self.get_logger().warning(
                    'runtime memory v%d is pulse-denominated; preserved as '
                    '%s — run tools/convert_memory_to_effort.py to keep it. '
                    'Starting from bootstrap.' % (version, preserved))
                return
            saved_fingerprint = saved.get('vehicle_fingerprint')
            if saved_fingerprint not in (None, 'unregistered',
                                         self.vehicle_fingerprint):
                # Learned memory belongs to a DIFFERENT chassis: refuse to
                # inherit it (second-vehicle exam rule) and start from the
                # cautious bootstrap instead.
                self.get_logger().warning(
                    'runtime memory fingerprint %r != vehicle %r; starting '
                    'from cautious bootstrap' % (
                        saved_fingerprint, self.vehicle_fingerprint))
                return
            saved_probes = saved.get('throttle_probe_observations')
            if isinstance(saved_probes, list):
                self.throttle_probe_results = saved_probes[-200:]
            saved_rate = saved.get('trim_rate')
            if isinstance(saved_rate, dict):
                for key in self.trim_rate:
                    if key in saved_rate:
                        self.trim_rate[key] = clamp(
                            float(saved_rate[key]), 0.25, 4.0)
            saved_anchor = saved.get('cruise_anchor')
            if isinstance(saved_anchor, dict):
                for direction, model in self.cruise_anchor.items():
                    candidate = saved_anchor.get(direction, {})
                    if int(candidate.get('samples', 0)) > 0:
                        model.update({
                            'effort': float(candidate['effort']),
                            'speed_mps': float(candidate['speed_mps']),
                            'spread': float(
                                candidate.get('spread', 0.0)),
                            'samples': int(candidate['samples'])})
            # Polarity is a hardware fact: retain it fully across restarts.
            self.steering_polarity = float(
                saved.get('steering_polarity', 1.0)) or 1.0
            self.throttle_polarity = float(
                saved.get('throttle_polarity', 1.0)) or 1.0
            # Battery/surface state cannot be trusted across a power cycle
            # (no voltage sensing; the 2026-07-12 sag session persisted
            # effort_scale 1.10, which would launch a fresh pack ~10% hot).
            # Relax battery-coupled state toward neutral on every process
            # start; driving evidence re-converges it within minutes.
            retention = clamp(
                float(self.p['battery_state_retention']), 0.0, 1.0)
            if version >= 5:
                for key in self.trim:
                    self.trim[key] = retention * float(
                        saved.get('trim', {}).get(key, 0.))
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
                        model['effort'] = float(candidate['effort'])
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
                    loaded_scale = clamp(
                        float(saved['effort_scale']), 0.7, 1.3)
                    self.effort_scale = 1.0 + retention * (loaded_scale - 1.0)
                    # A relaxed estimate is an UNCERTAIN estimate: re-inflate
                    # the Kalman variance to boot level so the first minutes
                    # of driving snap it to the actual pack state. Without
                    # this the filter is confidently wrong after every swap
                    # (2026-07-12 low-pack sluggish-launch session).
                    self.effort_scale_var = max(
                        self.effort_scale_var,
                        self.p['effort_scale_boot_sd'] ** 2)
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
                    'version': 8, 'trim': self.trim,
                    'vehicle_fingerprint': self.vehicle_fingerprint,
                    'steering_polarity': self.steering_polarity,
                    'throttle_polarity': self.throttle_polarity,
                    'throttle_probe_observations':
                        self.throttle_probe_results,
                    'cruise_anchor': self.cruise_anchor,
                    'trim_rate': self.trim_rate,
                    'learning_rejections': self.learning_rejections,
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
            max(self._curvature_caps()))
        # This is the sole executable-command boundary. Collision Monitor must
        # inspect the same nonzero speed and curvature the hardware will use.
        # (Per-side asymmetry is applied at execution in _desired; the
        # monitor may see the stronger side's envelope — conservative.)
        self.command_gate_reason = ''
        if abs(linear) >= .01:
            curvature = angular / linear
            linear = math.copysign(
                max(abs(linear), self.p['minimum_sustain_speed_mps']), linear)
            direction = 'forward' if linear > 0.0 else 'reverse'
            target_pulse = self._steering_effort(direction, curvature)
            steering_error = abs(target_pulse - self.steering_effort)
            if steering_error > self.p['steering_slowdown_error_effort']:
                scale = clamp(1.0 - steering_error / 0.6, 0.35, 1.0)
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
                front_x, rear_x, half_width,
                curvature=self.last_commanded_curvature)
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

    def _odom(self, msg):
        # Any planar Odometry source. Speed/yaw-rate are derived here from
        # consecutive stamped poses so the conditioning (median filter +
        # outlier gate) is identical no matter what produced the topic.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x; y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if self.pose is not None and stamp > self.last_odom_stamp:
            dt = stamp - self.last_odom_stamp
            dx, dy = x - self.pose[0], y - self.pose[1]
            forward = math.cos(self.pose[2]) * dx + math.sin(self.pose[2]) * dy
            self.raw_speed = forward / dt
            self.raw_yaw_rate = angle_difference(yaw, self.pose[2]) / dt
            self.speed_history.append(self.raw_speed)
            self.yaw_rate_history.append(self.raw_yaw_rate)
            self.speed = sorted(self.speed_history)[len(self.speed_history) // 2]
            self.yaw_rate = sorted(self.yaw_rate_history)[len(self.yaw_rate_history) // 2]
            yaw_envelope = (
                max(self._curvature_caps()) *
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
        self.pose = (x, y, yaw); self.last_odom_stamp = stamp
        self.odom_time = time.monotonic()

    def _desired(self):
        now = time.monotonic()
        if self.pack_exhausted:
            # Behavioral end-of-charge: maximum permitted authority failed to
            # move the robot repeatedly. This is the ONLY battery stop —
            # detected from odometry, latched until restart.
            return 0., 0., 'pack exhausted (behavioral)'
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
        # The clearance corridor must bend with the CURRENT request —
        # updating this after the gates deadlocked cusp departures: the
        # corridor stayed bent for the finished forward leg while the
        # reverse leg (opposite lock) was being judged, so the gate saw a
        # phantom obstacle for 6.4 s (2026-07-15 18:55 flip).
        if abs(self.cmd.linear.x) > .01:
            self.last_commanded_curvature = self._clamp_curvature(
                self.cmd.angular.z / self.cmd.linear.x)
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
        curvature = self._clamp_curvature(
            self.cmd.angular.z / requested_v)
        # Minimum sustainable speed and steering-transition slowdown were
        # already applied before Collision Monitor. Execute the approved Twist
        # unchanged; downstream safety logic may only replace it with zero.
        return requested_v, curvature, ''

    def _apply_birth_certificate(self):
        """Overlay the vehicle's physical identity onto parameters (§3).

        The birth certificate is the single human-supplied file for a new
        chassis. The controller consumes only geometry and sensor mounting
        (plus the vehicle fingerprint). The actuator sections (channels,
        neutrals, safe pulse bounds) belong to the hardware driver, which
        normalizes them to the [-1, 1] effort domain this node lives in.
        """
        self.vehicle_fingerprint = 'unregistered'
        path = os.path.expanduser(str(self.p.get(
            'birth_certificate_path',
            '~/ros2_ws/src/ackermann_robot/ackermann_robot/config/birth_certificate.yaml')))
        try:
            with open(path) as stream:
                certificate = yaml.safe_load(stream) or {}
            if int(certificate.get('schema_version', 0)) != 1:
                raise ValueError('unsupported schema_version')
        except FileNotFoundError:
            self.get_logger().warning(
                'no birth certificate; using plain parameters')
            return
        except Exception as error:
            self.get_logger().warning(
                f'birth certificate rejected ({error}); plain parameters')
            return
        vehicle = certificate.get('vehicle', {})
        footprint = vehicle.get('footprint', {})
        lidar = certificate.get('lidar', {})
        overlay = {
            'vehicle_wheelbase_m': vehicle.get('wheelbase_m'),
            'footprint_front_x_m': footprint.get('front_x_m'),
            'footprint_rear_x_m': footprint.get('rear_x_m'),
            'footprint_half_width_m': footprint.get('half_width_m'),
            'lidar_x_m': lidar.get('x_m'),
            'lidar_y_m': lidar.get('y_m'),
            'lidar_yaw_rad': lidar.get('yaw_rad'),
        }
        applied = {k: v for k, v in overlay.items() if v is not None}
        self.p.update(applied)
        self.vehicle_fingerprint = (
            f"{certificate.get('vehicle_name', 'unnamed')}"
            f"/wb{self.p['vehicle_wheelbase_m']:.4f}")
        self.get_logger().info(
            f'birth certificate applied: {self.vehicle_fingerprint} '
            f'({len(applied)} facts)')

    def _load_learned_steering(self):
        """Stage D: learned monotone map as shadow/blend challenger."""
        self.learned_steering = None
        self.learned_steering_state = 'off'
        self.learned_steering_effort = 0.0
        self.learned_blend_started = None
        self.learned_rolled_back = False
        if self.p['learned_steering_mode'] == 'off':
            return
        path = os.path.expanduser(self.p['learned_steering_map_path'])
        try:
            with open(path) as f:
                memory = yaml.safe_load(f)
            maps = {}
            for direction in ('forward', 'reverse'):
                entry = memory['directions'][direction]
                # The stored map is already the evidence-weighted posterior
                # (thin bins carry the geometric prior), so EVERY knot is
                # usable. PAVA plateaus are spread by an epsilon ramp so the
                # inverse stays strictly increasing. Authority is decided by
                # the trust region, not by dropping knots: evidence-dense
                # kappa spans earn full trust; sparse/prior spans keep the
                # cautious bootstrap trust instead of silently handing those
                # curvatures back to the dense measured map (amnesia purity).
                min_n = self.p['learned_steering_min_knot_samples']
                points = []
                forward_points = []
                data_kappas = []
                for pulse, kappa, n in zip(entry['knots_effort'],
                                           entry['kappa_1pm'],
                                           entry['samples_per_knot']):
                    forward_points.append((pulse, kappa))
                    if n >= min_n:
                        data_kappas.append(kappa)
                    if points and kappa <= points[-1][0] + 1e-6:
                        kappa = points[-1][0] + 1e-4
                    points.append((kappa, pulse))
                if len(points) < 4:
                    raise ValueError(
                        f'{direction}: only {len(points)} usable knots')
                # ODAAC steal #2: a non-monotonic inverse steers the wrong
                # way inside the crossing. Project instead of trusting.
                points, adjusted = isotonic_points(points)
                if adjusted:
                    self.get_logger().warning(
                        f'{direction} learned inverse map was non-monotonic'
                        ' — isotonic projection applied')
                trust = self.p['learned_steering_trust_kappa_1pm']
                boot_trust = \
                    self.p['learned_steering_bootstrap_trust_kappa_1pm']
                data_lo = min(data_kappas) if data_kappas else 0.0
                data_hi = max(data_kappas) if data_kappas else 0.0
                maps[direction] = {
                    'inverse': points,
                    'effort_to_kappa': forward_points,
                    'kappa_min': max(points[0][0],
                                     -trust, min(data_lo, -boot_trust)),
                    'kappa_max': min(points[-1][0],
                                     trust, max(data_hi, boot_trust)),
                }
            self.learned_steering = maps
            self.learned_steering_state = 'shadow'
            self.get_logger().info(
                'learned steering map loaded (%s): forward %.2f..%.2f, '
                'reverse %.2f..%.2f 1/m' % (
                    self.p['learned_steering_mode'],
                    maps['forward']['kappa_min'], maps['forward']['kappa_max'],
                    maps['reverse']['kappa_min'], maps['reverse']['kappa_max']))
        except Exception as error:
            self.get_logger().warning(
                f'learned steering map unavailable ({error}); champion only')
        # Stage C shadow: gradual effective-steering state. Prediction only —
        # logs its error next to the fixed-lag champion's; never steers.
        self.learned_dynamics = None
        self.effective_kappa = 0.0
        self.dynamics_last_t = None
        self.dynamics_error_ema = 0.0
        self.fixedlag_error_ema = 0.0
        self.dynamics_error_samples = 0
        if (self.p['learned_dynamics_mode'] != 'off'
                and self.learned_steering is not None):
            try:
                with open(os.path.expanduser(
                        self.p['learned_dynamics_path'])) as f:
                    memory = yaml.safe_load(f)
                self.learned_dynamics = {
                    d: {'tau_s': float(memory['directions'][d]['tau_s']),
                        'dist_m': float(memory['directions'][d]['dist_m'])}
                    for d in ('forward', 'reverse')}
                self.get_logger().info(
                    'learned steering dynamics loaded (shadow): '
                    f"fwd tau {self.learned_dynamics['forward']['tau_s']} s, "
                    f"rev tau {self.learned_dynamics['reverse']['tau_s']} s")
            except Exception as error:
                self.get_logger().warning(
                    f'learned dynamics unavailable ({error})')

    def _steering_effort(self, direction, curvature):
        side = 'negative' if curvature < 0.0 else 'positive'
        model = self.steering_models[f'{direction}_{side}']
        confidence = (self._model_confidence(model)
                      if self.p['apply_steering_models'] else 0.0)
        gain = max(.4, model['gain'])
        identified_input = (curvature - model['bias']) / gain
        corrected_curvature = curvature + confidence * (identified_input - curvature)
        corrected_curvature = self._clamp_curvature(corrected_curvature)
        champion = clamp(
            interpolate(self.steering_maps[direction], corrected_curvature),
            -1.0, 1.0)
        if self.learned_steering is None:
            return champion
        learned = self.learned_steering[direction]
        # The learned inverse consumes the RAW requested curvature: the map
        # was identified from measured response, so the RLS correction that
        # compensates the champion's dense map must not be double-applied.
        # Learned polarity mirrors the lookup when discovery flipped it.
        adjusted = curvature * self.steering_polarity
        in_region = learned['kappa_min'] <= adjusted <= learned['kappa_max']
        self.learned_steering_effort = clamp(
            interpolate(learned['inverse'], adjusted), -1.0, 1.0)
        if (self.p['learned_steering_mode'] != 'blend'
                or self.learned_rolled_back or not in_region):
            self.learned_steering_state = \
                'rolled_back' if self.learned_rolled_back else (
                    'shadow' if self.p['learned_steering_mode'] != 'blend'
                    else 'out_of_region')
            return champion
        now = time.monotonic()
        if self.learned_blend_started is None:
            self.learned_blend_started = now
        elif (now - self.learned_blend_started >
                self.p['learned_steering_rollback_grace_s']
                and self.applied_error_samples >= 20
                and self.applied_error_ema >
                self.p['learned_steering_rollback_error_1pm']):
            self.learned_rolled_back = True
            self.learned_steering_state = 'rolled_back'
            self.get_logger().warning(
                'learned steering ROLLED BACK: applied_error_ema %.3f > %.3f'
                % (self.applied_error_ema,
                   self.p['learned_steering_rollback_error_1pm']))
            return champion
        self.learned_steering_state = 'applied'
        return self.learned_steering_effort

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

    def _steering_lag(self, direction, speed, champion=False):
        # Speed floor matches the minimum sustainable speed: the distance
        # term is never evaluated below a speed the robot can actually hold.
        # Stage C applied mode: the learned first-order time constant replaces
        # the fixed measured delay for the preview horizon (its mean lag
        # equals tau). champion=True always returns the fixed model — the
        # shadow comparison must keep scoring against the original champion.
        if (not champion and self.learned_dynamics is not None
                and self.p['learned_dynamics_mode'] == 'applied'):
            dyn = self.learned_dynamics[direction]
            return (dyn['tau_s'] + dyn['dist_m'] /
                    max(abs(speed), self.p['minimum_sustain_speed_mps']))
        time_term = self.p[f'steering_lag_time_{direction}_s']
        if not champion:
            # The lag is NOT a constant: it moves with battery voltage,
            # servo load/friction, and the slew fix changed it outright
            # (the fixed values were measured THROUGH the old 2.0/s
            # limiter). Blend toward the online delay estimator as its
            # confidence grows; the fixed measurement anchors when the
            # estimator is unsure. champion=True keeps the frozen value
            # so shadow scoring stays against the original champion.
            confidence = clamp(self.delay_estimator.confidence, 0.0, 1.0)
            time_term = ((1.0 - confidence) * time_term +
                         confidence * self.estimated_steering_delay_s)
        return (time_term +
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
            self.feedback_ema = 0.0
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
            self.feedback_ema = 0.0
            self.preview = result
            return rpp_curvature
        lookahead = clamp(
            abs(target_speed) * self.p['rpp_lookahead_time_s'],
            self.p['rpp_min_lookahead_m'], self.p['rpp_max_lookahead_m'])
        current_path_curvature = geometry.pure_pursuit_curvature(
            index, direction_sign, lookahead)
        raw_feedback = rpp_curvature - current_path_curvature
        cap = self.p['steering_feedback_cap_1pm']
        feedback = clamp(raw_feedback, -cap, cap)
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
            pulse = self._steering_effort(direction, candidate)
            slew_time = (abs(pulse - self.steering_effort) /
                         self.p['steering_slew_per_s'])
            horizon = delay + slew_time
            requested_distance = abs(target_speed) * horizon
            preview_index, preview_distance, _ = geometry.advance(
                index, requested_distance, direction_sign)
            preview_curvature = geometry.pure_pursuit_curvature(
                preview_index, direction_sign, lookahead)
            candidate = preview_curvature + feedback
        # Preview is a TRACKING aid: while not rolling the projection is
        # least reliable (stationary at a segment start, often at a merge
        # kink) and composing from it flip-flopped the steering target at
        # ~3 Hz against the geometry guard — every flip re-armed the
        # pre-steer hold and reset the launch throttle: the robot could
        # not START (141 s, zero motion, 18:58 session). Launches steer
        # to raw RPP (stable); preview engages once rolling.
        if self.state != 'rolling':
            self.preview = result
            return rpp_curvature
        # Degenerate-projection guard: chord math at kinks/segment ends
        # can read path curvature beyond anything Smac can plan (observed:
        # rpp -0.5 vs path_now +2.22 -> commanded +1.31 = a hard
        # WRONG-DIRECTION jerk). A geometrically impossible path signal is
        # an artifact — use raw RPP for this tick.
        geometry_cap = max(self._curvature_caps()) + 0.10
        if (abs(current_path_curvature) > geometry_cap or
                abs(preview_curvature) > geometry_cap):
            result['steering_command_source'] = 'rpp_geometry_guard'
            result['path_curvature_now_1pm'] = current_path_curvature
            result['path_curvature_preview_1pm'] = preview_curvature
            self.preview = result
            return rpp_curvature
        # Continuous confidence blend ON PURPOSE: the (1-blend) share of
        # raw RPP is the guardrail when the controller's own path
        # projection is wrong (merged-segment kinks, index snaps). The
        # 2026-07-15 all-or-nothing experiment removed it and the robot
        # got WORSE (couldn't hold a line, abort churn) — reverted.
        applied_blend = (
            blend if self.p['apply_path_preview_compensation'] else 0.0)
        future_command = preview_curvature + feedback
        compensated = self._clamp_curvature(
            rpp_curvature + applied_blend * (future_command - rpp_curvature))
        assist_active = False
        result.update({
            'path_curvature_now_1pm': current_path_curvature,
            'path_curvature_preview_1pm': preview_curvature,
            'rpp_feedback_curvature_1pm': raw_feedback,
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

    def _baseline_curvature_from_effort(self, direction, effort):
        inverse = sorted((effort, curvature)
                         for curvature, effort in self.steering_maps[direction])
        return interpolate(inverse, effort)

    def _mean_delayed_baseline(self, start, end, delay, direction):
        values = [
            self._baseline_curvature_from_effort(direction, pulse)
            for stamp, pulse, pulse_direction in self.steering_history
            if start - delay <= stamp <= end - delay and
            pulse_direction == direction]
        if len(values) < 3:
            return None
        return sum(values) / len(values)

    def _causal_throttle_effort(self, stamp, direction):
        candidates = [item for item in self.throttle_history
                      if item[2] == direction]
        if not candidates:
            return None
        return min(candidates, key=lambda item: abs(item[0] - stamp))[1]

    def _curvature_caps(self):
        base = self.p['maximum_curvature_1pm']
        positive = self.p['maximum_curvature_positive_1pm'] or base
        negative = self.p['maximum_curvature_negative_1pm'] or base
        return positive, negative

    def _clamp_curvature(self, curvature):
        positive, negative = self._curvature_caps()
        return clamp(curvature, -negative, positive)

    def _reject_sample(self, reason):
        self.learning_rejections[reason] = (
            self.learning_rejections.get(reason, 0) + 1)

    def _learn_breakaway(self, now, direction):
        if math.isfinite(self.startup_kick_effort):
            # Pulse held when raw motion first appeared: the direct causal
            # attribution when the drop-on-raw-evidence path already lowered
            # the throttle before the robust observer confirmed breakaway.
            causal = self.startup_kick_effort
        else:
            causal = self._causal_throttle_effort(
                now - self.p[f'steering_lag_time_{direction}_s'], direction)
        if causal is None:
            self._reject_sample('breakaway_no_causal_command')
            return
        if ((direction == 'forward' and not -1.05 <= causal < 0.0) or
                (direction == 'reverse' and not 0.0 < causal <= 1.05)):
            self._reject_sample('breakaway_implausible_effort')
            return
        model = self.breakaway_models[direction]
        forgetting = (
            self.p['session_warmup_forgetting']
            if self.session_launches[direction] <
            self.p['session_warmup_launches']
            else self.p['breakaway_forgetting_factor'])
        # ODAAC steal #3: variance-damped learning rate. Inconsistent
        # evidence (EW variance of the innovation) SLOWS the update
        # instead of being averaged in; bounded so a genuine shift
        # (pack swap) still converges at >=1/4 speed.
        error = causal - model['effort']
        model['variance'] = (.9 * model.get('variance', 0.0) +
                             .1 * error * error)
        damp = max(.25, 1.0 / (
            1.0 + model['variance'] /
            self.p['breakaway_variance_reference']))
        model['effort'] += (1.0 - forgetting) * damp * error
        model['observations'] += 1
        self.session_launches[direction] += 1
        self.startup_causal_effort = causal

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
        neutral = 0.0
        map_offset = interpolate(self.throttle_maps[direction],
                                 abs(self.speed)) - neutral
        if abs(map_offset) < 0.0625:
            self._reject_sample('effort_scale_weak_map_signal')
            return
        observed = clamp((self.throttle_effort - neutral) / map_offset, 0.7, 1.3)
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
                    self.last_odom_stamp - self.last_control_odom_stamp, .001, .5)
            self.last_control_odom_stamp = self.last_odom_stamp
            self.controlled_odom_sequence = self.odom_sequence
        target, curvature, stop_reason = self._desired()
        if self.state not in ('startup', 'recovery'):
            # Each launch episode contributes at most one polarity vote.
            self.tpol_episode_counted = False
            self.tpol_wrong_streak = 0
        # Rollback monitor input: measured-vs-commanded curvature error,
        # updated on every fresh odometry sample while rolling, REGARDLESS of
        # learning state. The RLS residual EMA cannot serve here: it freezes
        # while the learned map is applied (identification guard), which the
        # 2026-07-12 rollback drill proved blinds the rollback trigger.
        if (fresh_odom and self.state == 'rolling'
                and self.odom_outlier and abs(target) > .01):
            self._reject_sample('odom_outlier')
        if (fresh_odom and self.state == 'rolling'
                and not self.odom_outlier and abs(target) > .01):
            # Weak-plant achievability: how much of the commanded speed the
            # plant actually delivers, ~20 s time constant. Odometry-only
            # battery/surface adaptation — deliberately slow and bounded so
            # it cannot confound identification.
            if abs(target) >= 0.8 * self.p['navigation_speed_limit_mps']:
                ratio = clamp(abs(self.speed) / abs(target), 0.0, 1.2)
                self.achievable_ratio += .005 * (ratio - self.achievable_ratio)
                # Cruise-anchor learner (failure/equilibrium learning, user
                # decision 2026-07-13): the running answer to "what pulse is
                # holding what speed lately" — the second point of the
                # feedforward line, learned from nothing but ordinary
                # settled cruising. Continuously adaptive by construction;
                # it follows battery/surface at its own time constant.
                if (0.75 < ratio < 1.25 and not self.tprobe_active
                        and self.state == 'rolling'):
                    anchor = self.cruise_anchor[
                        'forward' if target > 0 else 'reverse']
                    if anchor['samples'] == 0:
                        anchor['effort'] = self.throttle_effort
                        anchor['speed_mps'] = abs(self.speed)
                    deviation = abs(self.throttle_effort - anchor['effort'])
                    anchor['effort'] += .01 * (
                        self.throttle_effort - anchor['effort'])
                    anchor['speed_mps'] += .01 * (
                        abs(self.speed) - anchor['speed_mps'])
                    anchor['spread'] += .01 * (
                        deviation - anchor['spread'])
                    anchor['samples'] += 1
        if (fresh_odom and self.state == 'rolling'
                and not self.odom_outlier
                and abs(self.speed) >= self.p['minimum_sustain_speed_mps']
                and abs(target) > .01
                and now - self.rolling_since > 1.5):
            # The 1.5 s rolling-age gate is the rollback-attribution fix
            # (2026-07-14): launch transients, direction flips, and fresh-
            # pack surges fed applied_error_ema, which benched a HEALTHY
            # map at every speed era (71k/13k/21k rolled_back ticks) — the
            # EMA was measuring plant chaos, not map error. Converged
            # samples measure the plant; transients measure the controller.
            measured_kappa = self.yaw_rate / self.speed
            # Steering-polarity evidence: compare measured curvature against
            # the LEARNED MAP'S OWN PREDICTION for the pulse actually sent
            # (effective_kappa, polarity-adjusted). The pulse is known no
            # matter which authority steered, so champion-driven samples are
            # evidence too — the first drill showed command-based evidence
            # gets diluted by the champion and frozen by rollback. Flip
            # once, loudly, and un-bench the map: a rollback caused by
            # mirrored polarity is cured by the flip.
            if (self.learned_steering is not None
                    and abs(self.effective_kappa) > 0.15
                    and abs(measured_kappa) > 0.10):
                agree = (measured_kappa * self.effective_kappa) > 0.0
                self.steering_sign_evidence = clamp(
                    self.steering_sign_evidence + (1 if agree else -1),
                    -30, 30)
                if self.steering_sign_evidence <= -12:
                    self.steering_polarity = -self.steering_polarity
                    self.steering_sign_evidence = 0
                    self.effective_kappa = 0.0
                    self.learned_rolled_back = False
                    self.applied_error_ema = 0.0
                    self.applied_error_samples = 0
                    self.get_logger().error(
                        'STEERING POLARITY FLIPPED to %+d — measured '
                        'curvature consistently opposed the map prediction'
                        % int(self.steering_polarity))
            tracking_error = abs(measured_kappa - curvature)
            # Saturation attribution gate (ODAAC §8.1, user-caught
            # 2026-07-15): when the command is pinned at the executable
            # cap or the servo is at its endpoint, the plant cannot
            # deliver more no matter how good the map is — the error
            # belongs to the SATURATED COMMAND, not the learned map.
            # Today's command-side storms pinned curvature for whole
            # legs and benched a healthy map over and over (the
            # 'learning rolled back' flapping).
            positive_cap, negative_cap = self._curvature_caps()
            cap_here = positive_cap if curvature > 0 else negative_cap
            saturated = (abs(curvature) >= 0.98 * cap_here or
                         abs(self.steering_effort) >= 0.99)
            if saturated:
                self._reject_sample('error_ema_saturated_command')
            if tracking_error < 2.5 and not saturated:
                self.tracking_error_ema = (
                    tracking_error if self.tracking_error_samples == 0 else
                    .90 * self.tracking_error_ema + .10 * tracking_error)
                self.tracking_error_samples += 1
                # Rollback attribution (session-7 false positive): only error
                # measured while the learned map is actually steering may
                # count against it. Global chaos (launch surges, recovery
                # cycling, out-of-region cusps) stays in the global EMA.
                if self.learned_steering_state == 'applied':
                    self.applied_error_ema = (
                        tracking_error if self.applied_error_samples == 0
                        else .90 * self.applied_error_ema
                        + .10 * tracking_error)
                    self.applied_error_samples += 1
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
            target_throttle = 0.0
            target_steering = 0.0; self.learn_start = None
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
                self.startup_causal_effort = math.nan
                self.startup_kick_effort = math.nan
                self.gentle_coast_active = False
                self.gentle_pulse_active = False
                self.recent_rolling_speeds.clear()
                self.floor_streak = 0
                self.floor_streak_min = math.inf
                base = interpolate(self.throttle_maps[direction], abs(target))
                self.throttle_effort = (
                    0.0 if changing_while_moving else
                    (min(-0.234, base + 0.0125) if sign > 0
                     else max(0.234, base - 0.0125)))
            curvature = self._preview_command(
                rpp_curvature, target, direction)
            target_steering = self._steering_effort(direction, curvature)
            # Pre-monitor command shaping already accounted for this servo
            # traversal. Never mutate an approved nonzero Twist downstream.
            steering_error = abs(target_steering - self.steering_effort)
            steering_transition = (
                steering_error > self.p['steering_slowdown_error_effort'])
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
            self.probe_offset = 0.0
            if self.p['probe_enabled'] and self.state == 'rolling':
                if now < self.probe_until:
                    self.probe_offset = (
                        self.probe_sign * self.p['probe_amplitude_effort'])
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
                        self.probe_offset = (
                            self.probe_sign * self.p['probe_amplitude_effort'])
            target_steering += self.probe_offset
            # Stage F throttle probe: triangular pulse offset, lock-in
            # demodulated on completion. Same gate family as the steering
            # taps; a burst aborts (discarded) the moment any gate fails.
            self.tprobe_offset = 0.0
            if self.p['throttle_probe_enabled']:
                clearance_ok = (
                    (self.closest_forward if sign > 0
                     else self.closest_reverse) >
                    self.required_stop_clearance +
                    self.p['probe_clearance_margin_m'])
                stop_ahead = min(
                    self.preview.get('cusp_distance_m', math.inf),
                    self.preview.get('path_end_distance_m', math.inf))
                lateral = self.preview.get('cross_track_error_m', math.inf)
                gates_ok = (
                    not steering_transition and
                    abs(curvature) < self.p['probe_max_curvature_1pm'] and
                    abs(target) >= 0.18 and
                    directional_speed >=
                    self.p['minimum_sustain_speed_mps'] and
                    clearance_ok and
                    stop_ahead > self.p['probe_min_stop_distance_m'] and
                    (not math.isfinite(lateral) or abs(lateral) < .05))
                freq = self.p['throttle_probe_freq_hz']
                if self.tprobe_active:
                    level = min(self.tprobe_level,
                                len(self.p['throttle_probe_ladder_effort']) - 1)
                    amp = self.p['throttle_probe_ladder_effort'][level]
                    cycles = self.p['throttle_probe_ladder_cycles'][level]
                    if self.tprobe_phase_t >= cycles / freq:
                        # Completion needs no gates: samples are in hand.
                        # Gates-first ordering left a finished burst waiting
                        # forever when its goal ended (session 095651).
                        self._complete_throttle_probe(amp, cycles, freq)
                        self.tprobe_active = False
                        self.tprobe_last_end = now
                    elif not gates_ok or direction != self.tprobe_direction:
                        # PAUSE on gate loss AND on opposite-direction legs:
                        # gated cruise windows last ~1 s in small rooms
                        # (session 093005: 9 bursts, all cut <0.8 s). The
                        # phase clock freezes; the burst resumes coherently —
                        # lock-in demodulation is indifferent to wall-clock
                        # gaps in probe-time.
                        # The pause budget spends only while ROLLING with
                        # gates closed; launch/recovery time (where this
                        # block never runs) must not age the burst — on a
                        # weak pack that killed every burst (session 0957+).
                        self.tprobe_paused_t += dt
                        if self.tprobe_paused_t > 45.0:
                            self.tprobe_active = False
                            self.tprobe_last_end = now
                            self.tprobe_samples = []
                            self.get_logger().info(
                                'throttle probe discarded: paused too long')
                    else:
                        self.tprobe_paused_t = 0.0
                        self.tprobe_phase_t += dt
                        if int(self.tprobe_phase_t * 2) != int(
                                (self.tprobe_phase_t - dt) * 2):
                            self.get_logger().info(
                                'probe burst: phase %.1f/%.1f s amp %.0f'
                                % (self.tprobe_phase_t, cycles / freq, amp))
                        phase = (self.tprobe_phase_t * freq) % 1.0
                        self.tprobe_offset = amp * (
                            4.0 * abs(phase - 0.5) - 1.0)
                        if fresh_odom and not self.odom_outlier:
                            self.tprobe_samples.append(
                                (self.tprobe_phase_t, abs(self.speed)))
                        self.tprobe_effort_sum += self.throttle_effort
                        self.tprobe_effort_n += 1
                elif (gates_ok and now - self.tprobe_last_end >=
                        self.p['throttle_probe_cooldown_s']):
                    self.tprobe_active = True
                    self.tprobe_phase_t = 0.0
                    self.tprobe_paused_t = 0.0
                    self.tprobe_direction = direction
                    self.tprobe_samples = []
                    self.tprobe_effort_sum = 0.0
                    self.tprobe_effort_n = 0
            # Post-launch target ramp: commanding cruise the instant
            # breakaway confirms was the lunge amplifier — the feedforward
            # jumps to map(0.6) while the wheels barely roll. Ramp the
            # throttle target from the sustain floor as rolling matures;
            # steering and safety gates see the ORIGINAL command.
            ramp = self.p['launch_target_ramp_mps2']
            if ramp > 0.0:
                age = (now - self.rolling_since
                       if self.state == 'rolling' else 0.0)
                # Start the ramp at the speed the breakaway pop actually
                # delivered (~0.25-0.3), not the sustain floor: starting at
                # 0.12 while already moving at 0.3 commands an immediate
                # brake into the pop — the stall/re-pop 'lunge' cycle.
                start = max(self.p['minimum_sustain_speed_mps'],
                            self.rolling_entry_speed)
                allowed = start + ramp * age
                if abs(target) > allowed:
                    target = math.copysign(allowed, target)
            base = interpolate(self.throttle_maps[direction], abs(target))
            self.learned_throttle_state = 'off'
            self.learned_line_effort = 0.0
            if self.p['learned_throttle_mode'] != 'off':
                line_effort = self._learned_line_effort(direction, abs(target))
                if line_effort is None:
                    self.learned_throttle_state = 'insufficient'
                else:
                    self.learned_line_effort = line_effort
                    if self.p['learned_throttle_mode'] == 'applied':
                        base = line_effort
                        self.learned_throttle_state = 'applied'
                    else:
                        self.learned_throttle_state = 'shadow'
            trim_key = f'throttle_{direction}'
            # The actuator is refreshed at 50 Hz, but adaptation advances only
            # when TF supplies a new velocity measurement (normally ~10 Hz).
            # Reusing a sample here caused false stalls and bursty recovery.
            if not fresh_odom:
                target_throttle = self.throttle_effort
                self.throttle_antiwindup_state = 'hold_no_fresh_odom'
            elif self.state == 'direction_change':
                target_throttle = 0.0
                self.throttle_antiwindup_state = 'frozen_direction_change'
                if directional_speed >= -self.p['motion_threshold_mps']:
                    self.state = 'startup'
                    target_throttle = (
                        min(-0.234, base + 0.0125) if sign > 0
                        else max(0.234, base - 0.0125))
            elif self.state == 'planned_stop':
                # Coast at neutral while steering keeps tracking, which
                # pre-positions the wheels for the next path segment. The
                # direction flip or a zero command exits this state; if the
                # same-direction command persists, fall back to recovery.
                target_throttle = 0.0
                self.throttle_antiwindup_state = 'planned_stop'
                if directional_speed >= self.p['breakaway_threshold_mps']:
                    self.state = 'rolling'
                    self.rolling_since = now
                    self.rolling_entry_speed = abs(directional_speed)
                elif (now - self.planned_stop_since >
                        self.p['planned_stop_wait_s']):
                    self.state = 'recovery'
                    self.low_samples = self.limit_dwell = 0
            elif self.state in ('startup', 'recovery'):
                self.throttle_antiwindup_state = f'frozen_{self.state}'
                raw_motion = (
                    not self.odom_outlier and
                    sign * self.raw_speed >= self.p['motion_threshold_mps'])
                # Throttle-polarity evidence: SUSTAINED motion opposite to
                # the commanded direction (3 consecutive fresh samples),
                # counted at most once per launch episode. Two independent
                # wrong-way episodes flip the output mirror; a confirmed
                # correct launch clears the count. Per-tick counting false-
                # flipped during the 2026-07-13 steering-weave drill.
                if fresh_odom and not self.odom_outlier:
                    if (-sign * self.raw_speed >=
                            2.0 * self.p['motion_threshold_mps']):
                        self.tpol_wrong_streak += 1
                    else:
                        self.tpol_wrong_streak = 0
                    if (self.tpol_wrong_streak >= 3
                            and not self.tpol_episode_counted):
                        self.tpol_episode_counted = True
                        self.wrong_direction_launches += 1
                        if self.wrong_direction_launches >= 2:
                            self.throttle_polarity = -self.throttle_polarity
                            self.wrong_direction_launches = 0
                            self.get_logger().error(
                                'THROTTLE POLARITY FLIPPED to %+d — '
                                'sustained launches opposite to command'
                                % int(self.throttle_polarity))
                if raw_motion:
                    self.wrong_direction_launches = 0
                if (self.state == 'startup' and not raw_motion
                        and directional_speed
                        < self.p['breakaway_threshold_mps']
                        and steering_error
                        > self.p['launch_presteer_ready_effort']):
                    # Wheels first, motion second.
                    target_throttle = 0.0
                    self.throttle_antiwindup_state = 'launch_presteer'
                elif (self.gentle_motion and
                        steering_error >
                        self.p['gentle_steering_ready_effort']):
                    # Steering first: a low-speed correction should not spend
                    # its launch impulse while the wheels are still traversing.
                    target_throttle = 0.0
                    self.throttle_antiwindup_state = 'gentle_presteer'
                elif self.gentle_motion and self.gentle_coast_active:
                    self.gentle_pulse_active = False
                    target_throttle = 0.0
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
                    if not math.isfinite(self.startup_kick_effort):
                        self.startup_kick_effort = self.throttle_effort
                    if self.state == 'startup':
                        self._learn_breakaway(now, direction)
                    self.gentle_coast_active = True
                    self.gentle_coast_since = now
                    self.gentle_pulse_active = False
                    target_throttle = 0.0
                    self.throttle_antiwindup_state = 'gentle_motion_detected'
                elif directional_speed >= self.p['breakaway_threshold_mps']:
                    if self.state == 'startup':
                        self._learn_breakaway(now, direction)
                    self.state = 'rolling'; self.low_samples = self.limit_dwell = 0
                    self.rolling_since = now
                    self.rolling_entry_speed = abs(directional_speed)
                    target_throttle = base + self.trim[trim_key]
                else:
                    # Weaker than sustain during the confirmation window: the
                    # ESC is still unwinding breakaway effort, and dropping
                    # only to the sustain pulse left a 0.5-0.65 m/s tail.
                    # Specified in speed units and converted via the local map
                    # slope: a fixed microsecond offset was worth ~2x more
                    # speed in reverse (shallower slope) and caused post-launch
                    # reverse stalls in the 15:27 drive.
                    drop_effort = (self.p['startup_drop_through_mps'] *
                               self._throttle_slope(direction, abs(target)))
                    curve_boost = (1.0 + self.p['launch_curvature_boost']
                                   * abs(curvature))
                    # Boost ONLY the breakaway push. Scaling the post-
                    # breakaway landing level too made every high-curvature
                    # REVERSE launch land 30% hot (median 0.49 m/s peaks,
                    # worst 0.84 — the backward lunges, 2026-07-14 01:0x).
                    drop_pulse = base + self.trim[trim_key] + (
                        drop_effort if sign > 0 else -drop_effort)
                    if self.state == 'startup':
                        learned = (self.breakaway_models[direction]['effort']
                                   * curve_boost)
                        if (self.session_launches[direction] <
                                self.p['session_warmup_launches']):
                            # Re-feel the pedal: approach the remembered
                            # threshold from the weak side this session.
                            learned += (
                                self.p['session_warmup_weaken_effort'] if sign > 0
                                else -self.p['session_warmup_weaken_effort'])
                        response_time = max(
                            1.0 / self.p['control_rate_hz'], odom_dt,
                            self.p[f'steering_lag_time_{direction}_s'])
                        weaker_than_learned = (
                            self.throttle_effort > learned if sign > 0 else
                            self.throttle_effort < learned)
                        if raw_motion:
                            # Motion is visible in the newest derivative before
                            # the robust observer confirms it. Breakaway effort
                            # held through the confirmation window is what
                            # produced 0.41-0.7 m/s launches at a 0.3 target:
                            # drop to the rolling feedforward immediately and
                            # let the ramp branch re-arm if this was noise.
                            if not math.isfinite(self.startup_kick_effort):
                                self.startup_kick_effort = self.throttle_effort
                            target_throttle = drop_pulse
                            self.throttle_antiwindup_state = 'startup_raw_drop'
                        elif weaker_than_learned:
                            gap = abs(learned - self.throttle_effort)
                            step = max(MINIMUM_EFFORT_STEP, gap * odom_dt / response_time)
                            target_throttle = self.throttle_effort + (-step if sign > 0 else step)
                            if ((sign > 0 and target_throttle <= learned) or
                                    (sign < 0 and target_throttle >= learned)):
                                target_throttle = learned
                                self.startup_last_effort_time = now
                        elif (self.startup_last_effort_time is None or
                              now - self.startup_last_effort_time >= response_time):
                            self.startup_effort_attempts += 1
                            step = min(
                                self.p['recovery_step_effort'],
                                MINIMUM_EFFORT_STEP * self.startup_effort_attempts)
                            target_throttle = self.throttle_effort + (-step if sign > 0 else step)
                            self.startup_last_effort_time = now
                        else:
                            target_throttle = self.throttle_effort
                    elif raw_motion:
                        target_throttle = drop_pulse
                        self.throttle_antiwindup_state = 'recovery_raw_drop'
                    else:
                        target_throttle = self.throttle_effort + (
                            -self.p['recovery_step_effort'] if sign > 0 else
                            self.p['recovery_step_effort'])
                    if self.gentle_motion:
                        learned = self.breakaway_models[direction]['effort']
                        target_throttle = limit_gentle_launch_effort(
                            target_throttle, learned, sign > 0,
                            self.p['gentle_launch_max_extra_effort'])
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
                            target_throttle = 0.0
                            self.throttle_antiwindup_state = (
                                'gentle_pulse_timeout')
                    limit = self.p['forward_recovery_limit_effort'] if sign > 0 else self.p['reverse_recovery_limit_effort']
                    target_throttle = max(limit, target_throttle) if sign > 0 else min(limit, target_throttle)
                    at_limit = target_throttle == limit
                    self.limit_dwell = self.limit_dwell + 1 if at_limit else 0
                    if self.limit_dwell >= self.p['recovery_limit_dwell_samples']:
                        self.fault = 'traction authority exhausted'; target_throttle = 0.0; self.state = 'stopped'
                        self.exhaustion_events.append(now)
                        recent = [t for t in self.exhaustion_events
                                  if now - t < self.p['exhaustion_window_s']]
                        if (len(recent) >=
                                self.p['exhaustion_events_to_latch']):
                            self.pack_exhausted = True
                            self.get_logger().error(
                                'PACK EXHAUSTED (behavioral): %d traction '
                                'give-ups in %.0f s — commanding neutral '
                                'until restart' % (len(recent),
                                self.p['exhaustion_window_s']))
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
                kp = self.p['throttle_kp_effort_per_mps']
                if kp <= 0.0:
                    kp = self._throttle_slope(direction, abs(target))
                ki = self.p['throttle_ki_effort_per_m']
                low, high = ((self.p['forward_rolling_min_effort'], -0.234)
                             if sign > 0 else
                             (0.234, self.p['reverse_rolling_max_effort']))
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
                    # Rprop-style correction sizing (user design 2026-07-13):
                    # persistent same-sign error -> corrections were too
                    # timid, remember to ramp faster (x1.2); error sign flip
                    # -> they overshot, remember to step smaller (x0.5).
                    # Dead-zone below the odometry noise floor and a streak
                    # requirement longer than the feedback lag keep it from
                    # adapting on noise or chasing lag-phantoms.
                    if abs(error) > self.p['trim_deadzone_mps']:
                        err_sign = 1 if error > 0 else -1
                        if err_sign == self.trim_err_sign[trim_key]:
                            self.trim_err_streak[trim_key] += 1
                            if self.trim_err_streak[trim_key] >= 4:
                                self.trim_rate[trim_key] = min(
                                    4.0, self.trim_rate[trim_key] * 1.2)
                                self.trim_err_streak[trim_key] = 0
                        else:
                            if self.trim_err_sign[trim_key] != 0:
                                self.trim_rate[trim_key] = max(
                                    0.25, self.trim_rate[trim_key] * 0.5)
                            self.trim_err_sign[trim_key] = err_sign
                            self.trim_err_streak[trim_key] = 0
                        self.trim[trim_key] -= (
                            ki * self.trim_rate[trim_key] * error * odom_dt)
                    self.throttle_antiwindup_state = 'integrating'
                self.trim[trim_key] = clamp(
                    self.trim[trim_key],
                    -self.p['maximum_throttle_trim_effort'],
                    self.p['maximum_throttle_trim_effort'])
                desired = base + self.trim[trim_key] - kp * error
                desired = clamp(desired, low, high)
                slew = self.p['throttle_rolling_slew_per_odom']
                target_throttle = self.throttle_effort + clamp(
                    desired - self.throttle_effort, -slew, slew)
                self.low_samples = self.low_samples + 1 if directional_speed < self.p['motion_threshold_mps'] else 0
                # Breakaway threshold, not motion threshold: median-sign
                # noise at near-zero speed tripped 8 full-stop faults in the
                # 221 s run; genuine backward rolling crosses 0.045 quickly.
                if directional_speed < -self.p['breakaway_threshold_mps']:
                    self.fault = 'direction mismatch'; target_throttle = 0.0; self.state = 'stopped'
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
        max_step = self.p['steering_slew_per_s'] * dt
        self.steering_effort += clamp(target_steering - self.steering_effort, -max_step, max_step)
        if self.tprobe_active and self.state == 'rolling':
            # Exogenous probe rides on the final output (it is the r(t) of
            # closed-loop identification theory), inside the recovery bounds.
            target_throttle = clamp(
                target_throttle + self.tprobe_offset,
                self.p['forward_recovery_limit_effort'],
                self.p['reverse_recovery_limit_effort'])
        if self.throttle_polarity < 0:
            # Discovered mirrored ESC wiring: reflect the effort around
            # neutral. Magnitudes are approximate until the effort learners
            # re-converge; direction is correct immediately.
            target_throttle = -target_throttle
        self.throttle_effort = target_throttle
        self.steering_history.append(
            (now, self.steering_effort, self.direction or ''))
        self.throttle_history.append(
            (now, self.throttle_effort, self.direction or ''))
        self._shadow_dynamics(now, fresh_odom)
        command = EffortCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.steering_effort = float(self.steering_effort)
        command.drive_effort = float(self.throttle_effort)
        self.effort_pub.publish(command)
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
            'throttle_effort': self.throttle_effort, 'steering_effort': self.steering_effort,
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
            'startup_causal_effort': self.startup_causal_effort,
            'floor_estimate_forward_mps': (
                f'{self._floor_estimate("forward"):.4f}'),
            'floor_estimate_reverse_mps': (
                f'{self._floor_estimate("reverse"):.4f}'),
            'floor_observer_json': json.dumps(
                self.floor_observer, separators=(',', ':')),
            'probe_id': self.probe_id,
            'probe_offset_effort': f'{self.probe_offset:.1f}',
            'learned_steering_effort': f'{self.learned_steering_effort:.1f}',
            'learned_steering_state': self.learned_steering_state,
            'tracking_error_ema_1pm': f'{self.tracking_error_ema:.4f}',
            'applied_error_ema_1pm': f'{self.applied_error_ema:.4f}',
            'effective_kappa_1pm': f'{self.effective_kappa:.4f}',
            'dynamics_error_ema_1pm': f'{self.dynamics_error_ema:.4f}',
            'fixedlag_error_ema_1pm': f'{self.fixedlag_error_ema:.4f}',
            'achievable_speed_ratio': f'{self.achievable_ratio:.4f}',
            'cruise_anchor_json': json.dumps(
                self.cruise_anchor, separators=(',', ':')),
            'learned_line_effort': f'{self.learned_line_effort:.1f}',
            'learned_throttle_state': self.learned_throttle_state,
            'trim_rate_fwd': f'{self.trim_rate["throttle_forward"]:.2f}',
            'trim_rate_rev': f'{self.trim_rate["throttle_reverse"]:.2f}',
            'steering_polarity': int(self.steering_polarity),
            'steering_sign_evidence': self.steering_sign_evidence,
            'throttle_probe_offset_effort': f'{self.tprobe_offset:.2f}',
            'tprobe_phase_s': f'{self.tprobe_phase_t:.2f}',
            'effort_scale': f'{self.effort_scale:.4f}',
            'effort_scale_sd': f'{math.sqrt(self.effort_scale_var):.4f}',
            'segment_remaining_m': self.segment_remaining_m,
            'gentle_motion': int(self.gentle_motion),
            'gentle_coast_active': int(self.gentle_coast_active),
            'gentle_pulse_active': int(self.gentle_pulse_active),
            **self.preview,
            **{f'{key}_trim': value for key, value in self.trim.items()}})

    def _learn_steering(self, x, y, yaw):
        now = time.monotonic()
        # While the learned map is APPLIED the champion RLS/delay estimators
        # would identify against commands they did not generate; freeze them
        # rather than double-compensate (Stage C/D no-double-compensation).
        if self.learned_steering_state == 'applied':
            self.motion_history.clear()
            return
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
            self._reject_sample('identification_direction_mixed')
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
            self._reject_sample('identification_kappa_outlier')
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

    def _shadow_dynamics(self, now, fresh_odom):
        """Stage C shadow: evolve gradual effective steering state and score
        its curvature prediction against the fixed-lag champion. Never
        steers; both errors are logged for held-out-style live comparison."""
        if self.learned_steering is None:
            return
        direction = 'forward' if self.speed >= 0.0 else 'reverse'
        # Without learned dynamics (amnesia boot) the effective state still
        # evolves with a generic time constant so polarity discovery and
        # shadow prediction have a signal; error EMAs stay meaningful.
        dyn = (self.learned_dynamics[direction]
               if self.learned_dynamics is not None
               else {'tau_s': 0.25, 'dist_m': 0.0})
        target = self.steering_polarity * interpolate(
            self.learned_steering[direction]['effort_to_kappa'],
            self.steering_effort)
        dt = 0.0 if self.dynamics_last_t is None \
            else clamp(now - self.dynamics_last_t, 0.0, 0.5)
        self.dynamics_last_t = now
        speed = max(abs(self.speed), self.p['minimum_sustain_speed_mps'])
        tau_eff = max(dyn['tau_s'] + dyn['dist_m'] / speed, 1e-3)
        self.effective_kappa += (target - self.effective_kappa) * (
            1.0 - math.exp(-dt / tau_eff))
        if (not fresh_odom or self.state != 'rolling' or self.odom_outlier
                or abs(self.speed) < self.p['minimum_sustain_speed_mps']):
            return
        measured = self.yaw_rate / self.speed
        if abs(measured) > 2.5:
            return
        lag = self._steering_lag(direction, self.speed, champion=True)
        past_pulse = self.steering_effort
        for stamp, pulse, _ in reversed(self.steering_history):
            past_pulse = pulse
            if stamp <= now - lag:
                break
        fixed_prediction = self.steering_polarity * interpolate(
            self.learned_steering[direction]['effort_to_kappa'], past_pulse)
        dynamic_error = abs(measured - self.effective_kappa)
        fixed_error = abs(measured - fixed_prediction)
        if self.dynamics_error_samples == 0:
            self.dynamics_error_ema = dynamic_error
            self.fixedlag_error_ema = fixed_error
        else:
            self.dynamics_error_ema = (
                .95 * self.dynamics_error_ema + .05 * dynamic_error)
            self.fixedlag_error_ema = (
                .95 * self.fixedlag_error_ema + .05 * fixed_error)
        self.dynamics_error_samples += 1

    def _learned_line_effort(self, direction, speed):
        """Two-point learned feedforward: breakaway -> cruise anchor.

        Returns None while either anchor is too thin — the config map keeps
        authority until the robot has actually earned both points from its
        own driving (launches + settled cruising)."""
        anchor = self.cruise_anchor[direction]
        breakaway = self.breakaway_models[direction]
        if anchor['samples'] < 200 or breakaway['observations'] < 3:
            return None
        p0 = breakaway['effort']
        v0 = max(self._floor_estimate(direction), 0.05)
        p1, v1 = anchor['effort'], anchor['speed_mps']
        if abs(v1 - v0) < 0.05:
            return None
        return p0 + (speed - v0) * (p1 - p0) / (v1 - v0)

    def _complete_throttle_probe(self, amplitude, cycles, freq):
        """Quadrature lock-in over the finished burst -> local slope."""
        samples = self.tprobe_samples
        self.tprobe_samples = []
        if len(samples) < 0.6 * cycles / freq * 10.0:
            self.get_logger().info(
                'throttle probe discarded: only %d/%d odometry samples'
                % (len(samples), int(cycles / freq * 10.0)))
            return

        def demod(f):
            i_sum = q_sum = 0.0
            for t, v in samples:
                i_sum += v * math.sin(2.0 * math.pi * f * t)
                q_sum += v * math.cos(2.0 * math.pi * f * t)
            n = len(samples)
            return 2.0 * math.hypot(i_sum / n, q_sum / n)

        amp_out = demod(freq)
        amp_noise = max(demod(1.7 * freq), 1e-6)
        snr = amp_out / amp_noise
        amp_in = 8.0 * amplitude / (math.pi ** 2)   # triangle fundamental
        lag_gain = 1.0 / math.hypot(
            1.0, 2.0 * math.pi * freq
            * self.p['throttle_probe_esc_tau_s'])
        slope = amp_out / (amp_in * lag_gain)
        accepted = snr >= self.p['throttle_probe_snr_min']
        center = (self.tprobe_effort_sum / self.tprobe_effort_n
                        if self.tprobe_effort_n else 0.0)
        self.throttle_probe_results.append({
            'direction': self.tprobe_direction,
            'center_effort': round(center, 4),
            'amplitude_effort': amplitude,
            'cycles': cycles,
            'slope_mps_per_effort': round(slope, 4),
            'snr': round(snr, 2),
            'accepted': bool(accepted),
        })
        del self.throttle_probe_results[:-200]
        # Ladder (loud-first order): weak SNR steps toward the louder rung
        # (lower index); a strong burst earns a step toward stealth.
        if not accepted:
            self.tprobe_level = max(0, self.tprobe_level - 1)
        elif snr >= 2.0 * self.p['throttle_probe_snr_min']:
            self.tprobe_level = min(
                self.tprobe_level + 1,
                len(self.p['throttle_probe_ladder_effort']) - 1)
        self.get_logger().info(
            'throttle probe %s: dir=%s center=%.3f amp=%.3f slope=%.3f '
            'mps/effort snr=%.1f' % (
                'accepted' if accepted else 'weak', self.tprobe_direction,
                center, amplitude, slope, snr))

    def _publish_status(self):
        data = {'state': self.state, 'fault': self.fault,
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
                'throttle_effort': self.throttle_effort, 'steering_effort': self.steering_effort,
                'closest_obstacle_m': self.closest, 'trim': self.trim,
                'front_clearance_m': self.closest_forward,
                'rear_clearance_m': self.closest_reverse,
                'learning_rejections': self.learning_rejections,
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
                'startup_causal_effort': self.startup_causal_effort,
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
        # Weak-plant reduction: ask Nav2 only for what the pack/surface has
        # recently delivered (with margin). Lowers only — never raises above
        # the configured limit — and recovers as achievability recovers.
        if self.achievable_ratio < 0.85:
            limit = max(self.p['weak_plant_min_limit_mps'],
                        limit * clamp(self.achievable_ratio * 1.15, 0.4, 1.0))
        # Steering-loop speed budget (2026-07-15, the week's lesson made
        # policy): every command-side pathology appeared when cruise
        # outran the loop. Pure-pursuit stability needs the lookahead
        # floor to cover ~2 * v * total loop delay, so invert it:
        #   v_max = fraction * L_min / (tau_hat + sensing)
        # tau_hat is the ONLINE delay estimate (follows battery, servo
        # load, the slew fix), sensing covers odometry latency + filter.
        # Self-lowers when the loop degrades, self-raises as it improves
        # — cruise is EARNED from measurement, not configured ambition.
        if self.delay_estimator.confidence >= 0.5:
            loop_tau = (self.estimated_steering_delay_s +
                        self.p['steering_budget_sensing_lag_s'])
            budget = (self.p['steering_budget_fraction'] *
                      self.p['rpp_min_lookahead_m'] / max(loop_tau, 0.05))
            limit = min(limit, max(budget,
                                   self.p['weak_plant_min_limit_mps']))
        speed_limit = SpeedLimit(); speed_limit.header.stamp = self.get_clock().now().to_msg()
        speed_limit.percentage = False; speed_limit.speed_limit = max(.08, limit)
        self.speed_limit_pub.publish(speed_limit)
        status = DiagnosticStatus(); status.name = 'adaptive_ackermann_controller'
        status.hardware_id = ''; status.level = DiagnosticStatus.ERROR if self.fault else DiagnosticStatus.OK
        status.message = self.fault or self.state
        status.values = [KeyValue(key=k, value=str(v)) for k, v in data.items() if k != 'trim']
        array = DiagnosticArray(); array.header.stamp = self.get_clock().now().to_msg(); array.status = [status]
        self.diag_pub.publish(array)

    def destroy_node(self):
        try:
            self._save_runtime()
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
