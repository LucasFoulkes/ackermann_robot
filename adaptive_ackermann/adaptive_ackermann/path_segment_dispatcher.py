#!/usr/bin/env python3
"""Cusp-aware FollowPath proxy for one-direction-at-a-time controllers."""

import math
import json
import os
import tempfile
import threading
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import FollowPath
from nav2_msgs.srv import IsPathValid
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from adaptive_ackermann.adaptive_model import (
    TrackabilityEstimator, learned_planner_curvature, path_direction_runs,
    polyline_projection, segment_abort_reason, segment_goal_checker)


# Sentinel abort reason meaning "segment complete at its cusp": the goal
# proceeds to the next segment instead of aborting for fresh planning.
CUSP_HANDOVER = 'cusp handover'


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def split_path_at_cusps(path):
    """Return inclusive chronological slices containing one direction run."""
    if len(path.poses) < 2:
        return []
    samples = [
        (item.pose.position.x, item.pose.position.y,
         yaw_from_quaternion(item.pose.orientation))
        for item in path.poses]
    runs = path_direction_runs(samples)
    segments = []
    for start, end, direction in runs:
        segment = Path()
        segment.header = path.header
        segment.poses = list(path.poses[start:end + 1])
        segments.append((segment, direction))
    return segments


def segment_properties(path):
    samples = [
        (item.pose.position.x, item.pose.position.y,
         yaw_from_quaternion(item.pose.orientation))
        for item in path.poses]
    length = 0.0
    yaw_change = 0.0
    for first, second in zip(samples, samples[1:]):
        length += math.hypot(second[0] - first[0], second[1] - first[1])
        yaw_change += math.atan2(
            math.sin(second[2] - first[2]),
            math.cos(second[2] - first[2]))
    runs = path_direction_runs(samples)
    direction = runs[0][2] if runs else 1
    curvature = yaw_change / (direction * length) if length > 1e-6 else 0.0
    return samples, direction, length, curvature


def polyline_tracking_error(samples, x, y, yaw):
    """Return closest (distance, heading error) on a pose polyline."""
    distance, heading, _ = polyline_projection(samples, x, y, yaw)
    return distance, heading


def percentile(values, fraction):
    if not values:
        return math.inf
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1,
                       max(0, round((len(ordered) - 1) * fraction)))]


def finite_or_none(value):
    value = float(value)
    return value if math.isfinite(value) else None


class PathSegmentDispatcher(Node):
    """Expose FollowPath and execute Reeds-Shepp direction runs in order."""

    def __init__(self):
        super().__init__('path_segment_dispatcher')
        self.declare_parameter('frontend_action_name', 'follow_path')
        self.declare_parameter('backend_action_name', 'follow_path_backend')
        self.declare_parameter('cusp_settle_time_s', 0.60)
        self.declare_parameter('cusp_goal_checker_id', 'cusp_goal_checker')
        self.declare_parameter('minimum_executable_segment_m', 0.18)
        self.declare_parameter(
            'trackability_state_path', '~/.robot/planner_trackability.yaml')
        self.declare_parameter('trackability_prior_radius_m', 1.30)
        self.declare_parameter('trackability_physical_limit_1pm', 0.92)
        self.declare_parameter('trackability_min_segment_m', 0.50)
        self.declare_parameter('trackability_max_entry_xy_m', 0.06)
        self.declare_parameter('trackability_max_entry_yaw_rad', 0.15)
        self.declare_parameter('trackability_max_xte_p90_m', 0.05)
        self.declare_parameter('trackability_max_heading_p90_rad', 0.15)
        self.declare_parameter('trackability_max_blocked_fraction', 0.05)
        self.declare_parameter('trackability_min_samples', 10)
        self.declare_parameter('segment_no_progress_timeout_s', 6.0)
        self.declare_parameter('segment_progress_epsilon_m', 0.05)
        self.declare_parameter('segment_wrong_direction_timeout_s', 0.75)
        self.declare_parameter('segment_watch_min_length_m', 0.18)
        self.declare_parameter('segment_path_check_period_s', 0.50)
        self.declare_parameter('segment_path_check_horizon_m', 1.50)
        self.declare_parameter('segment_blocked_path_timeout_s', 0.75)
        # A "reversal" this close to a committed segment's end, commanding
        # the NEXT segment's direction, is RPP already driving the next leg
        # (its lookahead crossed the cusp before the goal checker fired) —
        # hand the segment over instead of aborting the goal for fresh
        # planning. 2026-07-14: this signature caused 6 aborts and 1 failed
        # goal in one session, with endpoint errors of ~8 cm at abort time.
        self.declare_parameter('cusp_handover_window_m', 0.25)
        frontend = self.get_parameter('frontend_action_name').value
        backend = self.get_parameter('backend_action_name').value
        self.settle_time = float(self.get_parameter('cusp_settle_time_s').value)
        self.cusp_goal_checker = str(
            self.get_parameter('cusp_goal_checker_id').value)
        self.minimum_segment = float(
            self.get_parameter('minimum_executable_segment_m').value)
        self.trackability_path = os.path.expanduser(str(
            self.get_parameter('trackability_state_path').value))
        self.trackability_min_segment = float(
            self.get_parameter('trackability_min_segment_m').value)
        self.entry_xy_limit = float(
            self.get_parameter('trackability_max_entry_xy_m').value)
        self.entry_yaw_limit = float(
            self.get_parameter('trackability_max_entry_yaw_rad').value)
        self.xte_limit = float(
            self.get_parameter('trackability_max_xte_p90_m').value)
        self.heading_limit = float(
            self.get_parameter('trackability_max_heading_p90_rad').value)
        self.blocked_limit = float(
            self.get_parameter('trackability_max_blocked_fraction').value)
        self.minimum_samples = int(
            self.get_parameter('trackability_min_samples').value)
        self.declare_parameter('launch_patience_factor', 3.0)
        self.launch_patience_factor = float(
            self.get_parameter('launch_patience_factor').value)
        self.controller_state = ''
        self.no_progress_timeout = float(
            self.get_parameter('segment_no_progress_timeout_s').value)
        self.progress_epsilon = float(
            self.get_parameter('segment_progress_epsilon_m').value)
        self.wrong_direction_timeout = float(
            self.get_parameter('segment_wrong_direction_timeout_s').value)
        self.watch_min_length = float(
            self.get_parameter('segment_watch_min_length_m').value)
        self.path_check_period = float(
            self.get_parameter('segment_path_check_period_s').value)
        self.path_check_horizon = float(
            self.get_parameter('segment_path_check_horizon_m').value)
        self.blocked_path_timeout = float(
            self.get_parameter('segment_blocked_path_timeout_s').value)
        self.cusp_handover_window = float(
            self.get_parameter('cusp_handover_window_m').value)
        prior = 1.0 / float(
            self.get_parameter('trackability_prior_radius_m').value)
        physical = float(
            self.get_parameter('trackability_physical_limit_1pm').value)
        saved = self._load_trackability_state()
        self.trackability = TrackabilityEstimator(
            prior, physical, state=saved)
        # Stage E: apply qualified trackability evidence to the NEXT goal in
        # the same process instead of waiting for a relaunch (doc §11.3).
        # NOTE: trackability_physical_limit_1pm arrives from launch with the
        # planning utilization ALREADY applied — pass utilization 1.0 here or
        # the envelope would be discounted twice.
        self.declare_parameter('planner_update_min_change', 0.02)
        self.planner_update_min_change = float(
            self.get_parameter('planner_update_min_change').value)
        self.trackability_physical = physical
        self.trackability_prior_radius = float(
            self.get_parameter('trackability_prior_radius_m').value)
        self.applied_planner_radius = 1.0 / learned_planner_curvature(
            physical, 1.0, self.trackability_prior_radius, saved)
        self.planner_param_client = self.create_client(
            SetParameters, '/planner_server/set_parameters')
        self.controller_param_client = self.create_client(
            SetParameters, '/controller_server/set_parameters')
        self.metric_lock = threading.Lock()
        self.pose = None
        self.pre_monitor_command = (0.0, 0.0)
        self.post_monitor_command = (0.0, 0.0)
        self.active_metric = None
        self.execution_lock = threading.Lock()
        self.generation_lock = threading.Lock()
        self.latest_generation = 0
        self.request_generations = {}
        self.active_execution = False
        self.backend_goals = {}
        self.cancel_futures = set()
        self.group = ReentrantCallbackGroup()
        self.client = ActionClient(
            self, FollowPath, backend, callback_group=self.group)
        self.path_valid_client = self.create_client(
            IsPathValid, '/is_path_valid', callback_group=self.group)
        self.server = ActionServer(
            self, FollowPath, frontend,
            execute_callback=self.execute,
            goal_callback=self.goal,
            cancel_callback=self.cancel,
            callback_group=self.group)
        self.segment_pub = self.create_publisher(
            Path, '/controller_segment_plan', 10)
        self.trackability_pub = self.create_publisher(
            String, '/planner_trackability', 10)
        self.create_subscription(
            Odometry, '/odom', self._odom, 20,
            callback_group=self.group)
        self.create_subscription(
            Twist, '/cmd_vel_nav', self._pre_monitor, 20,
            callback_group=self.group)
        self.create_subscription(
            Twist, '/cmd_vel', self._post_monitor, 20,
            callback_group=self.group)
        self.create_subscription(
            String, '/controller/debug', self._debug, 20,
            callback_group=self.group)
        self.create_timer(
            0.10, self._watch_segment, callback_group=self.group)
        self.create_timer(
            self.path_check_period, self._check_remaining_path,
            callback_group=self.group)
        self.get_logger().info(
            f'Cusp dispatcher ready: {frontend} -> {backend}; '
            f'planner radius {1.0 / self.trackability.curvature_limit:.3f} m '
            f'({self.trackability.source}, '
            f'confidence={self.trackability.confidence:.2f})')

    def _load_trackability_state(self):
        try:
            with open(self.trackability_path) as stream:
                state = yaml.safe_load(stream) or {}
            return state if isinstance(state, dict) else {}
        except (OSError, yaml.YAMLError, TypeError, ValueError):
            return {}

    def _apply_planner_envelope(self):
        """Push the learned radius to Smac/RPP between goals (Stage E)."""
        curvature = learned_planner_curvature(
            self.trackability_physical, 1.0,
            self.trackability_prior_radius, self.trackability.state())
        radius = 1.0 / max(curvature, 1e-3)
        change = abs(radius - self.applied_planner_radius) \
            / max(self.applied_planner_radius, 1e-3)
        if change < self.planner_update_min_change:
            return
        targets = (
            (self.planner_param_client, 'GridBased.minimum_turning_radius'),
            (self.controller_param_client,
             'FollowPath.regulated_linear_scaling_min_radius'))
        if not all(client.service_is_ready() for client, _ in targets):
            self.get_logger().warning(
                'planner envelope update deferred: parameter services not '
                'ready')
            return
        for client, name in targets:
            value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=radius)
            client.call_async(SetParameters.Request(
                parameters=[ParameterMsg(name=name, value=value)]))
        self.get_logger().info(
            'planner envelope updated between goals: radius %.3f -> %.3f m '
            '(source %s, confidence %.2f)' % (
                self.applied_planner_radius, radius,
                self.trackability.source, self.trackability.confidence))
        self.applied_planner_radius = radius

    def _save_trackability_state(self):
        directory = os.path.dirname(self.trackability_path)
        try:
            os.makedirs(directory, exist_ok=True)
            fd, temporary = tempfile.mkstemp(
                prefix='.planner_trackability.', suffix='.yaml', dir=directory)
            try:
                with os.fdopen(fd, 'w') as stream:
                    yaml.safe_dump(self.trackability.state(), stream,
                                   sort_keys=True)
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temporary, self.trackability_path)
            finally:
                if os.path.exists(temporary):
                    os.unlink(temporary)
        except OSError as error:
            self.get_logger().warning(
                f'Could not persist planner trackability: {error}')

    def _pre_monitor(self, message):
        with self.metric_lock:
            self.pre_monitor_command = (
                float(message.linear.x), float(message.angular.z))
            metric = self.active_metric
            if metric is None:
                return
            now = time.monotonic()
            if abs(message.linear.x) < 0.03:
                metric['wrong_direction_since'] = None
            elif float(message.linear.x) * metric['direction'] < 0.0:
                if metric['wrong_direction_since'] is None:
                    metric['wrong_direction_since'] = now
            else:
                metric['wrong_direction_since'] = None

    def _post_monitor(self, message):
        with self.metric_lock:
            self.post_monitor_command = (
                float(message.linear.x), float(message.angular.z))

    def _debug(self, message):
        try:
            data = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        fault = str(data.get('fault', '')).lower()
        state = str(data.get('state', '')).lower()
        with self.metric_lock:
            self.controller_state = state
            if self.active_metric is not None and (
                    'obstacle' in fault or 'overspeed' in fault or
                    state == 'recovery'):
                self.active_metric['contaminated'] = True

    def _odom(self, message):
        pose = message.pose.pose
        current = (float(pose.position.x), float(pose.position.y),
                   yaw_from_quaternion(pose.orientation))
        measured_speed = float(message.twist.twist.linear.x)
        with self.metric_lock:
            self.pose = current
            metric = self.active_metric
            if metric is None:
                return
            xte, heading, along = polyline_projection(
                metric['samples'], *current)
            metric['current_progress_m'] = along
            if along >= metric['best_progress_m'] + self.progress_epsilon:
                metric['best_progress_m'] = along
                metric['last_progress_at'] = time.monotonic()
            if abs(measured_speed) < 0.03:
                return
            pre_v = self.pre_monitor_command[0]
            if pre_v * metric['direction'] <= 0.01:
                return
            if math.isfinite(xte) and math.isfinite(heading):
                metric['xte'].append(xte)
                metric['heading'].append(heading)
                metric['commanded_samples'] += 1
                if abs(self.post_monitor_command[0]) < 0.01:
                    metric['blocked_samples'] += 1

    @staticmethod
    def _pose_error(current, target):
        if current is None:
            return math.inf, math.inf
        return (math.hypot(current[0] - target[0], current[1] - target[1]),
                abs(math.atan2(math.sin(current[2] - target[2]),
                               math.cos(current[2] - target[2]))))

    def _begin_metric(self, segment, direction, backend_goal,
                      next_direction=None):
        samples, _, length, curvature = segment_properties(segment)
        with self.metric_lock:
            entry_xy, entry_yaw = self._pose_error(self.pose, samples[0])
            initial_progress = (polyline_projection(
                samples, *self.pose)[2] if self.pose is not None else 0.0)
            self.active_metric = {
                'samples': samples, 'direction': direction, 'length': length,
                'next_direction': next_direction,
                'curvature': curvature, 'entry_xy': entry_xy,
                'entry_yaw': entry_yaw, 'xte': [], 'heading': [],
                'commanded_samples': 0, 'blocked_samples': 0,
                'contaminated': False,
                'best_progress_m': initial_progress,
                'current_progress_m': initial_progress,
                'last_progress_at': time.monotonic(),
                'wrong_direction_since': None,
                'blocked_path_since': None,
                'path_check_pending': False,
                'abort_reason': '', 'cancel_requested': False,
                'backend_goal': backend_goal,
                'segment_path': segment}

    def _check_remaining_path(self):
        if not self.path_valid_client.service_is_ready():
            return
        with self.metric_lock:
            metric = self.active_metric
            if (metric is None or metric['cancel_requested'] or
                    metric['path_check_pending'] or
                    metric['length'] < self.watch_min_length):
                return
            progress = metric['current_progress_m']
            samples = metric['samples']
            source = metric['segment_path']
            token = id(metric)
            metric['path_check_pending'] = True
        cumulative = [0.0]
        for first, second in zip(samples, samples[1:]):
            cumulative.append(cumulative[-1] + math.hypot(
                second[0] - first[0], second[1] - first[1]))
        start = max(0.0, progress - 0.10)
        end = progress + self.path_check_horizon
        indices = [index for index, distance in enumerate(cumulative)
                   if start <= distance <= end]
        if indices and indices[0] > 0:
            indices.insert(0, indices[0] - 1)
        if len(indices) < 2:
            with self.metric_lock:
                if self.active_metric is not None and id(
                        self.active_metric) == token:
                    self.active_metric['path_check_pending'] = False
            return
        path = Path()
        path.header = source.header
        path.poses = [source.poses[index] for index in indices]
        request = IsPathValid.Request()
        request.path = path
        future = self.path_valid_client.call_async(request)
        future.add_done_callback(
            lambda completed, identity=token:
            self._path_check_complete(completed, identity))

    def _path_check_complete(self, future, token):
        try:
            invalid = not bool(future.result().is_valid)
        except Exception as error:
            self.get_logger().warning(f'Path validity check failed: {error}')
            invalid = False
        now = time.monotonic()
        with self.metric_lock:
            metric = self.active_metric
            if metric is None or id(metric) != token:
                return
            metric['path_check_pending'] = False
            if invalid:
                if metric['blocked_path_since'] is None:
                    metric['blocked_path_since'] = now
            else:
                metric['blocked_path_since'] = None

    def _watch_segment(self):
        backend_goal = None
        reason = ''
        now = time.monotonic()
        with self.metric_lock:
            metric = self.active_metric
            if metric is None or metric['cancel_requested']:
                return
            # Launch patience (odom-only weak-plant adaptation): while the
            # actuator layer reports it is actively escalating a launch or
            # traction recovery, progress legitimately takes longer — stretch
            # the watchdog instead of cancelling a segment the weak pack is
            # still winning. The controller's behavioral pack-exhaustion
            # latch remains the terminal backstop.
            patience = (self.launch_patience_factor
                        if self.controller_state in ('startup', 'recovery')
                        else 1.0)
            reason = segment_abort_reason(
                now, metric['length'], metric['last_progress_at'],
                metric['wrong_direction_since'],
                metric['blocked_path_since'],
                no_progress_timeout=self.no_progress_timeout * patience,
                wrong_direction_timeout=self.wrong_direction_timeout,
                blocked_path_timeout=self.blocked_path_timeout,
                minimum_watched_length=self.watch_min_length)
            handover = False
            if reason == 'controller reversed inside a committed segment':
                remaining = metric['length'] - metric['current_progress_m']
                next_direction = metric.get('next_direction')
                commanded = self.pre_monitor_command[0]
                handover = (
                    next_direction is not None and
                    next_direction != metric['direction'] and
                    remaining <= self.cusp_handover_window and
                    commanded * next_direction > 0.0)
            if reason:
                metric['abort_reason'] = CUSP_HANDOVER if handover else reason
                metric['contaminated'] = not handover
                metric['cancel_requested'] = True
                backend_goal = metric.get('backend_goal')
                if handover:
                    reason = (f'cusp handover: controller already pulling '
                              f'into the next segment with '
                              f'{max(remaining, 0.0):.2f} m remaining')
        if backend_goal is not None:
            if 'handover' in reason:
                self.get_logger().info(f'Completing segment early: {reason}')
            else:
                self.get_logger().error(
                    f'Aborting segment for fresh planning: {reason}')
            future = backend_goal.cancel_goal_async()
            self.cancel_futures.add(future)
            future.add_done_callback(self._cancel_complete)

    def _cancel_complete(self, future):
        self.cancel_futures.discard(future)
        try:
            response = future.result()
            accepted = bool(response.goals_canceling)
        except Exception as error:
            accepted = False
            self.get_logger().error(
                f'Backend segment cancel request failed: {error}')
        if accepted:
            self.get_logger().warning(
                'Backend accepted segment cancellation')
            return
        self.get_logger().error('Backend rejected segment cancellation')
        # Allow the timer to retry rather than leaving a live backend action
        # behind an already-detected invalid segment.
        with self.metric_lock:
            if self.active_metric is not None:
                self.active_metric['cancel_requested'] = False

    def _finish_metric(self, backend_succeeded):
        with self.metric_lock:
            metric = self.active_metric
            self.active_metric = None
        # An empty authoritative segment explicitly releases ownership. This
        # prevents later Nav2 recovery behaviors (especially BackUp) from
        # inheriting the completed FollowPath segment's terminal semantics.
        self.segment_pub.publish(Path())
        if metric is None:
            return ''
        with self.metric_lock:
            endpoint_xy, endpoint_yaw = self._pose_error(
                self.pose, metric['samples'][-1])
        xte_p90 = percentile(metric['xte'], .90)
        heading_p90 = percentile(metric['heading'], .90)
        count = metric['commanded_samples']
        blocked = (metric['blocked_samples'] / count if count else 1.0)
        demand = abs(metric['curvature'])
        eligible = (
            metric['length'] >= self.trackability_min_segment and
            demand >= 0.10 and count >= self.minimum_samples and
            metric['entry_xy'] <= self.entry_xy_limit and
            metric['entry_yaw'] <= self.entry_yaw_limit and
            blocked <= self.blocked_limit and
            not metric['contaminated'])
        # A cusp handover IS reaching the segment end (within the handover
        # window, tracking clean) — the backend result is CANCELED only
        # because we canceled it ourselves.
        completed = (backend_succeeded or
                     metric['abort_reason'] == CUSP_HANDOVER)
        passed = (completed and xte_p90 <= self.xte_limit and
                  heading_p90 <= self.heading_limit)
        branch = ('forward' if metric['direction'] > 0 else 'reverse')
        branch += '_positive' if metric['curvature'] >= 0.0 else '_negative'
        changed = self.trackability.observe(
            branch, demand, passed, eligible=eligible)
        if eligible:
            self._save_trackability_state()
        evidence = {
            'branch': branch, 'length_m': metric['length'],
            'demand_curvature_1pm': demand,
            'entry_xy_m': finite_or_none(metric['entry_xy']),
            'entry_yaw_rad': finite_or_none(metric['entry_yaw']),
            'xte_p90_m': finite_or_none(xte_p90),
            'heading_p90_rad': finite_or_none(heading_p90),
            'endpoint_xy_m': finite_or_none(endpoint_xy),
            'endpoint_yaw_rad': finite_or_none(endpoint_yaw),
            'blocked_fraction': blocked, 'samples': count,
            'contaminated': metric['contaminated'], 'eligible': eligible,
            'passed': passed, 'estimate_updated': changed,
            'branch_estimate_1pm': (
                self.trackability.branches[branch]['estimate_1pm']),
            'planner_radius_m': 1.0 / self.trackability.curvature_limit,
            'best_progress_m': metric['best_progress_m'],
            'abort_reason': metric['abort_reason'],
        }
        self.trackability_pub.publish(String(data=json.dumps(
            evidence, allow_nan=False)))
        self.get_logger().info(
            f'Trackability {branch}: length={metric["length"]:.2f} m, '
            f'demand={demand:.3f} 1/m, entry={metric["entry_xy"]:.3f} m/'
            f'{metric["entry_yaw"]:.3f} rad, xte_p90={xte_p90:.3f} m, '
            f'heading_p90={heading_p90:.3f} rad, endpoint={endpoint_xy:.3f} m/'
            f'{endpoint_yaw:.3f} rad, blocked={blocked:.1%}, samples={count}, '
            f'eligible={eligible}, pass={passed}, '
            f'estimate={self.trackability.branches[branch]["estimate_1pm"]:.3f} '
            f'1/m{(" UPDATED" if changed else "")}')
        return metric['abort_reason']

    def goal(self, request):
        if len(request.path.poses) < 2:
            return GoalResponse.REJECT
        with self.generation_lock:
            self.latest_generation += 1
            self.request_generations[id(request)] = self.latest_generation
            superseding = self.active_execution
            backend_goals = list(self.backend_goals.values())
        if superseding:
            self.get_logger().warning(
                'New FollowPath goal supersedes active dispatch; '
                'canceling old backend before starting it')
            for backend_goal in backend_goals:
                future = backend_goal.cancel_goal_async()
                self.cancel_futures.add(future)
                future.add_done_callback(self._cancel_complete)
        return GoalResponse.ACCEPT

    def cancel(self, goal_handle):
        backend_goal = self.backend_goals.get(bytes(goal_handle.goal_id.uuid))
        if backend_goal is not None:
            backend_goal.cancel_goal_async()
        return CancelResponse.ACCEPT

    def _is_superseded(self, generation):
        with self.generation_lock:
            return generation < self.latest_generation

    async def execute(self, goal_handle):
        generation = self.request_generations.pop(
            id(goal_handle.request), self.latest_generation)
        self.execution_lock.acquire()
        with self.generation_lock:
            self.active_execution = True
        try:
            if self._is_superseded(generation):
                result = FollowPath.Result()
                result.error_code = FollowPath.Result.UNKNOWN
                result.error_msg = 'superseded before dispatch began'
                goal_handle.abort()
                return result
            return await self._execute_serial(goal_handle, generation)
        finally:
            with self.generation_lock:
                self.active_execution = False
            self.execution_lock.release()
            # Between-goals is the safe boundary for envelope promotion:
            # the goal just ended, the next global plan has not been
            # computed yet. Failures are non-fatal (next goal simply plans
            # with the previous envelope).
            try:
                self._apply_planner_envelope()
            except Exception as error:
                self.get_logger().warning(
                    f'planner envelope update failed: {error}')

    async def _execute_serial(self, goal_handle, generation):
        result = FollowPath.Result()
        raw_segments = split_path_at_cusps(goal_handle.request.path)
        segments = []
        for index, item in enumerate(raw_segments):
            _, _, length, _ = segment_properties(item[0])
            if index + 1 < len(raw_segments) and length < self.minimum_segment:
                self.get_logger().warning(
                    f'Skipping non-final cusp fragment {index + 1}: '
                    f'{length:.3f} m < {self.minimum_segment:.3f} m')
                continue
            segments.append(item)
        # Skipping a fragment leaves its neighbors pointing the SAME way.
        # Executed separately they cost a full stop + settle + relaunch
        # between two legs of identical direction (measured 3-4 s each,
        # nine in one maneuver, 2026-07-15). Merge them into one leg: the
        # skipped fragment's ~0.17 m offset becomes an in-path kink the
        # capped steering feedback absorbs while rolling.
        merged = []
        for segment, direction in segments:
            if merged and merged[-1][1] == direction:
                combined = Path()
                combined.header = merged[-1][0].header
                combined.poses = (list(merged[-1][0].poses) +
                                  list(segment.poses))
                merged[-1] = (combined, direction)
                self.get_logger().info(
                    'Merged consecutive same-direction segments around a '
                    'skipped fragment')
            else:
                merged.append((segment, direction))
        segments = merged
        if not segments:
            result.error_code = FollowPath.Result.INVALID_PATH
            result.error_msg = 'path has no executable segment'
            goal_handle.abort()
            return result
        if not self.client.wait_for_server(timeout_sec=3.0):
            result.error_code = FollowPath.Result.UNKNOWN
            result.error_msg = 'backend FollowPath action unavailable'
            goal_handle.abort()
            return result
        key = bytes(goal_handle.goal_id.uuid)
        self.get_logger().info(
            f'Executing {len(segments)} chronological direction segment(s)')
        try:
            for index, (segment, direction) in enumerate(segments):
                if (goal_handle.is_cancel_requested or
                        self._is_superseded(generation)):
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()
                    return result
                request = FollowPath.Goal()
                request.path = segment
                request.controller_id = goal_handle.request.controller_id
                request.goal_checker_id = segment_goal_checker(
                    goal_handle.request.goal_checker_id,
                    final_segment=(index + 1 == len(segments)),
                    cusp_goal_checker=self.cusp_goal_checker)
                request.progress_checker_id = goal_handle.request.progress_checker_id

                def feedback(message, frontend=goal_handle):
                    frontend.publish_feedback(message.feedback)

                backend_goal = await self.client.send_goal_async(
                    request, feedback_callback=feedback)
                if not backend_goal.accepted:
                    result.error_code = FollowPath.Result.INVALID_CONTROLLER
                    result.error_msg = f'backend rejected segment {index + 1}'
                    goal_handle.abort()
                    return result
                self.backend_goals[key] = backend_goal
                self.segment_pub.publish(segment)
                self._begin_metric(
                    segment, direction, backend_goal,
                    next_direction=(segments[index + 1][1]
                                    if index + 1 < len(segments) else None))
                self.get_logger().info(
                    f'Segment {index + 1}/{len(segments)}: '
                    f'{"forward" if direction > 0 else "reverse"}, '
                    f'{len(segment.poses)} poses')
                wrapped = await backend_goal.get_result_async()
                succeeded = wrapped.status == GoalStatus.STATUS_SUCCEEDED
                abort_reason = self._finish_metric(succeeded)
                self.backend_goals.pop(key, None)
                if (goal_handle.is_cancel_requested or
                        self._is_superseded(generation)):
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()
                    return result
                if abort_reason == CUSP_HANDOVER:
                    # Segment completed at its cusp; the backend CANCELED
                    # status is our own doing. Fall through to the settle
                    # dwell and dispatch the next segment.
                    self.get_logger().info(
                        f'Segment {index + 1}/{len(segments)} handed over '
                        f'at cusp')
                elif abort_reason:
                    result.error_code = FollowPath.Result.FAILED_TO_MAKE_PROGRESS
                    result.error_msg = abort_reason
                    goal_handle.abort()
                    return result
                elif wrapped.status != GoalStatus.STATUS_SUCCEEDED:
                    result = wrapped.result
                    if wrapped.status == GoalStatus.STATUS_CANCELED:
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()
                    return result
                if (index + 1 < len(segments) and
                        segments[index + 1][1] != direction):
                    deadline = (self.get_clock().now().nanoseconds * 1e-9 +
                                self.settle_time)
                    while self.get_clock().now().nanoseconds * 1e-9 < deadline:
                        if (goal_handle.is_cancel_requested or
                                self._is_superseded(generation)):
                            if goal_handle.is_cancel_requested:
                                goal_handle.canceled()
                            else:
                                goal_handle.abort()
                            return result
                        # rclpy coroutine execution does not install an asyncio
                        # event loop. The multithreaded executor keeps action
                        # callbacks live while this one goal thread dwells.
                        time.sleep(0.05)
            goal_handle.succeed()
            return result
        finally:
            with self.metric_lock:
                self.active_metric = None
            backend_goal = self.backend_goals.pop(key, None)
            if backend_goal is not None and goal_handle.is_cancel_requested:
                await backend_goal.cancel_goal_async()

    def destroy_node(self):
        self.server.destroy()
        self.client.destroy()
        super().destroy_node()


def main():
    rclpy.init()
    node = PathSegmentDispatcher()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
