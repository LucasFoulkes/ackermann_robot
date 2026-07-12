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
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from ackermann_robot.adaptive_model import (
    TrackabilityEstimator, path_direction_runs, segment_goal_checker)


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
    best = None
    for first, second in zip(samples, samples[1:]):
        vx, vy = second[0] - first[0], second[1] - first[1]
        squared = vx * vx + vy * vy
        fraction = (0.0 if squared < 1e-9 else max(0.0, min(
            1.0, ((x - first[0]) * vx + (y - first[1]) * vy) / squared)))
        px, py = first[0] + fraction * vx, first[1] + fraction * vy
        delta_yaw = math.atan2(
            math.sin(second[2] - first[2]),
            math.cos(second[2] - first[2]))
        path_yaw = first[2] + fraction * delta_yaw
        distance = math.hypot(x - px, y - py)
        heading = abs(math.atan2(
            math.sin(yaw - path_yaw), math.cos(yaw - path_yaw)))
        if best is None or distance < best[0]:
            best = (distance, heading)
    return best if best is not None else (math.inf, math.inf)


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
        prior = 1.0 / float(
            self.get_parameter('trackability_prior_radius_m').value)
        physical = float(
            self.get_parameter('trackability_physical_limit_1pm').value)
        saved = self._load_trackability_state()
        self.trackability = TrackabilityEstimator(
            prior, physical, state=saved)
        self.metric_lock = threading.Lock()
        self.pose = None
        self.pre_monitor_command = (0.0, 0.0)
        self.post_monitor_command = (0.0, 0.0)
        self.active_metric = None
        self.group = ReentrantCallbackGroup()
        self.client = ActionClient(
            self, FollowPath, backend, callback_group=self.group)
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
        self.backend_goals = {}
        self.get_logger().info(
            f'Cusp dispatcher ready: {frontend} -> {backend}; learned '
            f'planner radius {1.0 / self.trackability.curvature_limit:.3f} m')

    def _load_trackability_state(self):
        try:
            with open(self.trackability_path) as stream:
                state = yaml.safe_load(stream) or {}
            return state if isinstance(state, dict) else {}
        except (OSError, yaml.YAMLError, TypeError, ValueError):
            return {}

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
            if metric is None or abs(measured_speed) < 0.03:
                return
            pre_v = self.pre_monitor_command[0]
            if pre_v * metric['direction'] <= 0.01:
                return
            xte, heading = polyline_tracking_error(
                metric['samples'], *current)
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

    def _begin_metric(self, segment, direction):
        samples, _, length, curvature = segment_properties(segment)
        with self.metric_lock:
            entry_xy, entry_yaw = self._pose_error(self.pose, samples[0])
            self.active_metric = {
                'samples': samples, 'direction': direction, 'length': length,
                'curvature': curvature, 'entry_xy': entry_xy,
                'entry_yaw': entry_yaw, 'xte': [], 'heading': [],
                'commanded_samples': 0, 'blocked_samples': 0,
                'contaminated': False}

    def _finish_metric(self, backend_succeeded):
        with self.metric_lock:
            metric = self.active_metric
            self.active_metric = None
            if metric is None:
                return
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
        passed = (backend_succeeded and xte_p90 <= self.xte_limit and
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

    def goal(self, request):
        return (GoalResponse.ACCEPT if len(request.path.poses) >= 2
                else GoalResponse.REJECT)

    def cancel(self, goal_handle):
        backend_goal = self.backend_goals.get(bytes(goal_handle.goal_id.uuid))
        if backend_goal is not None:
            backend_goal.cancel_goal_async()
        return CancelResponse.ACCEPT

    async def execute(self, goal_handle):
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
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
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
                self._begin_metric(segment, direction)
                self.get_logger().info(
                    f'Segment {index + 1}/{len(segments)}: '
                    f'{"forward" if direction > 0 else "reverse"}, '
                    f'{len(segment.poses)} poses')
                wrapped = await backend_goal.get_result_async()
                succeeded = wrapped.status == GoalStatus.STATUS_SUCCEEDED
                self._finish_metric(succeeded)
                self.backend_goals.pop(key, None)
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return result
                if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
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
                        if goal_handle.is_cancel_requested:
                            goal_handle.canceled()
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
