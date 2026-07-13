#!/usr/bin/env python3
"""Autonomous branch-coverage drive using ordinary Nav2 goals.

Sends a fixed sequence of goals relative to the live robot pose, chosen to
exercise forward/reverse and left/right steering branches plus Reeds-Shepp
cusps. Every candidate goal is checked against the latest laser scan for a
clear corridor before it is sent; Nav2 planning, the Collision Monitor, and
the controller scan guard remain the safety authorities during motion.

Writes a per-goal outcome summary to ~/.robot/experiment_sessions/.
"""

import json
import math
import os
import sys
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

# Measured lidar mount from the birth-certificate facts.
LASER_X = 0.237
LASER_YAW = math.pi

# Footprint rectangle in base_link, with margin, to reject self-returns.
FOOT_X_MIN, FOOT_X_MAX = -0.14, 0.44
FOOT_Y_ABS = 0.20

CORRIDOR_HALF_WIDTH = 0.35
CORRIDOR_END_MARGIN = 0.50
GOAL_CLEAR_RADIUS = 0.55
GOAL_TIMEOUT_S = 150.0
SETTLE_S = 3.0
MAX_CONSECUTIVE_FAILURES = 3
# Keep the whole session within WiFi/supervision range of the start pose:
# no goal may target beyond this radius, and drifting past it triggers a
# return-toward-start relocation (2026-07-12: a session walked out of WiFi).
TETHER_RADIUS_M = 3.0

# (name, dx, dy, dyaw) in the robot frame at send time.
TEMPLATES = [
    ('fwd_straight_launch', 1.8, 0.0, 0.0),
    ('fwd_left_arc', 1.5, 0.9, math.radians(55)),
    ('fwd_right_arc', 1.5, -0.9, math.radians(-55)),
    ('heading_flip_center', 1.2, 0.0, math.pi),
    ('pure_reverse', -1.5, 0.0, 0.0),
    ('fwd_left_arc_2', 1.5, 0.9, math.radians(55)),
    ('fwd_right_arc_2', 1.5, -0.9, math.radians(-55)),
    ('flip_reverse_right', 1.0, -0.6, math.pi),
    ('flip_reverse_left', 1.0, 0.6, math.pi),
    ('fwd_straight_finish', 1.8, 0.0, 0.0),
]


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class CoverageDrive(Node):

    def __init__(self):
        super().__init__('auto_coverage_drive')
        self._scan = None
        self._scan_stamp = 0.0
        self._scan_lock = threading.Lock()
        scan_qos = QoSProfile(depth=10)
        scan_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, '/scan', self._on_scan, scan_qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def _on_scan(self, msg):
        with self._scan_lock:
            self._scan = msg
            self._scan_stamp = time.monotonic()

    def scan_points_base(self):
        """Latest scan as (x, y) points in base_link, self-returns removed."""
        with self._scan_lock:
            scan = self._scan
        if scan is None:
            return None
        points = []
        angle = scan.angle_min
        cos_yaw = math.cos(LASER_YAW)
        sin_yaw = math.sin(LASER_YAW)
        for r in scan.ranges:
            a = angle
            angle += scan.angle_increment
            if not math.isfinite(r) or r <= scan.range_min or r >= scan.range_max:
                continue
            lx, ly = r * math.cos(a), r * math.sin(a)
            x = LASER_X + cos_yaw * lx - sin_yaw * ly
            y = sin_yaw * lx + cos_yaw * ly
            if FOOT_X_MIN < x < FOOT_X_MAX and abs(y) < FOOT_Y_ABS:
                continue
            points.append((x, y))
        return points

    def scan_fresh(self):
        return self._scan is not None and time.monotonic() - self._scan_stamp < 1.0

    def robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        return t.x, t.y, yaw_from_quat(tf.transform.rotation)

    def tf_fresh(self):
        """MOLA can wedge with a stale odom->base_link transform while other
        topics keep flowing; require the TF stamp to be recent."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time())
        except Exception:
            return False
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        # MOLA's stamps legitimately trail wall clock by a few seconds of
        # pipeline latency; the wedge we guard against freezes them for
        # minutes. 4 s separates the two cleanly.
        return abs(now - stamp) < 4.0

    def clearance_survey(self):
        """Nearest obstacle distance per 15-degree bearing sector."""
        points = self.scan_points_base()
        if points is None:
            return None
        sectors = {}
        for px, py in points:
            bearing = math.degrees(math.atan2(py, px))
            sector = int(math.floor(bearing / 15.0)) * 15
            d = math.hypot(px, py)
            if d < sectors.get(sector, 99.0):
                sectors[sector] = round(d, 2)
        return sectors

    def corridor_clear(self, dx, dy):
        """True if the straight corridor to (dx, dy) in base_link is free."""
        points = self.scan_points_base()
        if points is None:
            return False, 'no scan'
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            return False, 'degenerate goal'
        ux, uy = dx / dist, dy / dist
        for px, py in points:
            along = px * ux + py * uy
            cross = ux * py - uy * px
            if 0.05 < along < dist + CORRIDOR_END_MARGIN \
                    and abs(cross) < CORRIDOR_HALF_WIDTH:
                return False, f'corridor blocked at {along:.2f} m'
            if math.hypot(px - dx, py - dy) < GOAL_CLEAR_RADIUS:
                return False, 'goal point obstructed'
        return True, ''

    def build_goal(self, pose, dx, dy, dyaw):
        x, y, yaw = pose
        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x + dx * math.cos(yaw) - dy * math.sin(yaw)
        goal.pose.position.y = y + dx * math.sin(yaw) + dy * math.cos(yaw)
        gyaw = yaw + dyaw
        goal.pose.orientation.z = math.sin(gyaw / 2.0)
        goal.pose.orientation.w = math.cos(gyaw / 2.0)
        return goal

    def drive_goal(self, goal):
        """Send one goal, wait for terminal status. Returns (status, seconds)."""
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal
        start = time.monotonic()
        send_future = self.nav_client.send_goal_async(nav_goal)
        while not send_future.done():
            if time.monotonic() - start > 10.0:
                return 'send_timeout', time.monotonic() - start
            time.sleep(0.05)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return 'rejected', time.monotonic() - start
        result_future = handle.get_result_async()
        while not result_future.done():
            if time.monotonic() - start > GOAL_TIMEOUT_S:
                handle.cancel_goal_async()
                time.sleep(3.0)
                return 'timeout_cancelled', time.monotonic() - start
            time.sleep(0.2)
        status = result_future.result().status
        names = {
            GoalStatus.STATUS_SUCCEEDED: 'succeeded',
            GoalStatus.STATUS_ABORTED: 'aborted',
            GoalStatus.STATUS_CANCELED: 'canceled',
        }
        return names.get(status, f'status_{status}'), time.monotonic() - start


def main():
    rclpy.init()
    node = CoverageDrive()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    session = {
        'started_at': time.strftime('%Y-%m-%d %H:%M:%S'),
        'goals': [],
    }
    out_dir = os.path.expanduser('~/.robot/experiment_sessions')
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(
        out_dir, time.strftime('session_%Y%m%d_%H%M%S.json'))

    def save():
        tmp = out_path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(session, f, indent=2)
        os.replace(tmp, out_path)

    print('waiting for scan, odometry TF, and Nav2 action server...',
          flush=True)
    deadline = time.monotonic() + 180.0
    while time.monotonic() < deadline:
        if node.scan_fresh() and node.robot_pose() is not None \
                and node.tf_fresh() and node.nav_client.server_is_ready():
            break
        print(f'waiting: scan={node.scan_fresh()} '
              f'pose={node.robot_pose() is not None} '
              f'tf_fresh={node.tf_fresh()} '
              f'nav={node.nav_client.server_is_ready()}', flush=True)
        time.sleep(1.0)
    else:
        session['error'] = 'stack never became healthy'
        save()
        print('ERROR: stack never became healthy', flush=True)
        rclpy.shutdown()
        return

    print('stack healthy; settling 10 s before first goal', flush=True)
    time.sleep(10.0)

    # Arena check: refuse to drive unless some direction has real room.
    survey = node.clearance_survey()
    session['initial_clearance_by_sector'] = survey
    save()
    best = max(survey.values()) if survey else 0.0
    print(f'clearance survey (m by 15-deg sector): {survey}', flush=True)
    if '--survey-only' in sys.argv:
        session['error'] = 'survey-only run, no goals sent'
        save()
        print('survey-only mode: exiting before any goal', flush=True)
        rclpy.shutdown()
        return
    if best < 2.5:
        session['error'] = f'arena too small: best clearance {best:.2f} m'
        save()
        print(f'REFUSING TO DRIVE: best clearance only {best:.2f} m',
              flush=True)
        rclpy.shutdown()
        return

    # Scale the whole template set to the room: full size needs ~3.5 m of
    # open space; smaller rooms get proportionally shorter maneuvers so the
    # robot stops driving itself into corners (session-12 lesson).
    room_scale = max(0.6, min(1.0, best / 3.5))
    if room_scale < 1.0:
        print(f'room scale {room_scale:.2f} (best clearance {best:.1f} m)',
              flush=True)
    scaled_templates = [(n, dx * room_scale, dy * room_scale, dyaw)
                        for n, dx, dy, dyaw in TEMPLATES]

    def maybe_relocate():
        """If the forward wedge is cramped, drive to the most open sector.

        Session 2 lesson: rotating templates into a shrinking wedge walks the
        robot into a corner (front clearance reached 0.11 m). Recentering to
        open space keeps every subsequent maneuver clean.
        """
        s = node.clearance_survey()
        if not s:
            return None
        fwd_best = max((v for k, v in s.items() if abs(k) <= 60), default=0.0)
        if fwd_best >= 2.2:
            return None
        if max(s.values()) < 2.3:
            return 'boxed_in'
        pose = node.robot_pose()
        if pose is None:
            return None
        # Try the most open sectors in order; a rearward sector becomes a
        # REVERSE escape (session-16 lesson: backing straight out worked
        # where every forward corridor was blocked).
        chosen = None
        reason = 'no open sector'
        for best_sector in sorted(s, key=s.get, reverse=True)[:4]:
            best = s[best_sector]
            if best < 2.3:
                break
            bearing = math.radians(best_sector + 7.5)
            d = min(1.8, best - 1.3)
            rdx, rdy = d * math.cos(bearing), d * math.sin(bearing)
            if start_pose is not None:
                gx = pose[0] + rdx * math.cos(pose[2]) - rdy * math.sin(pose[2])
                gy = pose[1] + rdx * math.sin(pose[2]) + rdy * math.cos(pose[2])
                if math.hypot(gx - start_pose[0],
                              gy - start_pose[1]) > TETHER_RADIUS_M:
                    reason = 'relocation beyond tether'
                    continue
            clear, reason = node.corridor_clear(rdx, rdy)
            if clear:
                chosen = (rdx, rdy, bearing, d, best_sector)
                break
        if chosen is None:
            return f'relocation blocked: {reason}'
        rdx, rdy, bearing, d, best_sector = chosen
        rearward = abs(best_sector + 7.5) > 100.0
        # Rearward escape keeps the current heading so Smac plans a straight
        # reverse instead of a turn there is no room for.
        goal = node.build_goal(pose, rdx, rdy, 0.0 if rearward else bearing)
        print(f'relocating {d:.1f} m toward {best_sector} deg'
              f'{" (reverse escape)" if rearward else ""} '
              f'(fwd wedge only {fwd_best:.1f} m)', flush=True)
        outcome, seconds = node.drive_goal(goal)
        session['goals'].append({
            'name': 'relocate_to_open_space', 'outcome': outcome,
            'duration_s': round(seconds, 1),
            'bearing_deg': best_sector, 'distance_m': round(d, 2)})
        save()
        time.sleep(SETTLE_S)
        return 'relocated'

    start_pose = node.robot_pose()

    def maybe_return_home():
        """Drive back toward the start pose when past the tether radius."""
        pose = node.robot_pose()
        if pose is None or start_pose is None:
            return None
        dist = math.hypot(start_pose[0] - pose[0], start_pose[1] - pose[1])
        if dist <= TETHER_RADIUS_M * 0.8:
            return None
        home_bearing = math.atan2(start_pose[1] - pose[1],
                                  start_pose[0] - pose[0]) - pose[2]
        for offset_deg in (0, 25, -25, 50, -50):
            bearing = home_bearing + math.radians(offset_deg)
            d = min(1.8, dist)
            rdx, rdy = d * math.cos(bearing), d * math.sin(bearing)
            clear, _ = node.corridor_clear(rdx, rdy)
            if not clear:
                continue
            print(f'returning toward start: {dist:.1f} m out '
                  f'(tether {TETHER_RADIUS_M} m)', flush=True)
            goal = node.build_goal(pose, rdx, rdy, bearing)
            outcome, seconds = node.drive_goal(goal)
            session['goals'].append({
                'name': 'return_toward_start', 'outcome': outcome,
                'duration_s': round(seconds, 1),
                'distance_from_start_m': round(dist, 2)})
            save()
            time.sleep(SETTLE_S)
            return 'returned'
        return None

    consecutive_failures = 0
    relocations = 0
    returns = 0
    for name, dx, dy, dyaw in scaled_templates:
        if returns < 4 and maybe_return_home() == 'returned':
            returns += 1
        if relocations < 4:
            note = maybe_relocate()
            if note == 'boxed_in':
                session['error'] = 'boxed in: no sector with >=2.3 m clearance'
                save()
                print('ABORT: boxed in everywhere', flush=True)
                break
            if note == 'relocated':
                relocations += 1
            elif note is not None and note != 'boxed_in':
                print(f'relocation unavailable: {note}', flush=True)
        pose = node.robot_pose()
        if pose is None or not node.scan_fresh():
            session['goals'].append({'name': name, 'outcome': 'skipped',
                                     'reason': 'unhealthy inputs'})
            save()
            continue

        # Try the template as-is, then rotate the whole maneuver toward open
        # space (preserving its shape relative to the approach), then shrink.
        entry = {'name': name, 'requested': [dx, dy, dyaw]}
        chosen = None
        for scale in (1.0, 0.7):
            for rot_deg in (0, 25, -25, 50, -50, 75, -75):
                rot = math.radians(rot_deg)
                sdx = (dx * math.cos(rot) - dy * math.sin(rot)) * scale
                sdy = (dx * math.sin(rot) + dy * math.cos(rot)) * scale
                if math.hypot(sdx, sdy) < 0.8:
                    continue
                if start_pose is not None:
                    gx = pose[0] + sdx * math.cos(pose[2]) \
                        - sdy * math.sin(pose[2])
                    gy = pose[1] + sdx * math.sin(pose[2]) \
                        + sdy * math.cos(pose[2])
                    if math.hypot(gx - start_pose[0],
                                  gy - start_pose[1]) > TETHER_RADIUS_M:
                        entry['blocked_reason'] = 'beyond tether'
                        continue
                clear, reason = node.corridor_clear(sdx, sdy)
                if clear:
                    chosen = (sdx, sdy, dyaw + rot)
                    entry['scale'] = scale
                    entry['rotated_deg'] = rot_deg
                    break
                entry['blocked_reason'] = reason
            if chosen is not None:
                break
        if chosen is None:
            entry['outcome'] = 'skipped_no_clearance'
            session['goals'].append(entry)
            save()
            print(f'{name}: skipped ({entry.get("blocked_reason")})',
                  flush=True)
            continue

        goal = node.build_goal(pose, chosen[0], chosen[1], chosen[2])
        entry['goal_odom'] = [goal.pose.position.x, goal.pose.position.y]
        print(f'{name}: sending goal scale={entry["scale"]}', flush=True)
        outcome, seconds = node.drive_goal(goal)
        entry['outcome'] = outcome
        entry['duration_s'] = round(seconds, 1)
        end_pose = node.robot_pose()
        if end_pose is not None:
            entry['endpoint_error_m'] = round(math.hypot(
                end_pose[0] - goal.pose.position.x,
                end_pose[1] - goal.pose.position.y), 3)
        session['goals'].append(entry)
        save()
        print(f'{name}: {outcome} in {seconds:.0f} s '
              f'endpoint_err={entry.get("endpoint_error_m")}', flush=True)

        if outcome == 'succeeded':
            consecutive_failures = 0
        else:
            consecutive_failures += 1
            if consecutive_failures >= MAX_CONSECUTIVE_FAILURES:
                session['error'] = 'aborted: consecutive failures'
                save()
                print('ABORT: consecutive failures', flush=True)
                break
        time.sleep(SETTLE_S)

    session['finished_at'] = time.strftime('%Y-%m-%d %H:%M:%S')
    save()
    print(f'session complete -> {out_path}', flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
