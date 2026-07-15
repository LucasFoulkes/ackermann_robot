#!/usr/bin/env python3
"""Three-point-turn drill: near-rotation-in-place at MAX steering.

Commands alternating full-lock forward/reverse arcs directly through the
normal safety chain (/cmd_vel_nav_raw -> adaptive limits -> collision
monitor -> controller). No planner involved: this isolates and trains the
PLANT skill the maneuvers depend on. Correct steer timing comes from the
controller itself — its launch pre-steer holds throttle until the wheels
are at the commanded angle, so every phase starts fully cranked.

Phases alternate lock with drive direction so the nose rotates one way
while translation cancels; a phase ends when it has moved far enough
(keep-in-place), stalled (blocked -> switch direction now), or timed out.

Prints a per-phase report (heading gained, distance used, ACHIEVED
curvature vs the commanded max) and the total footprint — run it several
times in a row and watch the numbers improve as breakaway/anchors learn
this regime. Every full-lock launch is also prime training data for the
high-curvature steering-map refit.

Usage: three_point_drill.py [rotations]   (default 2, alternating sides)
Needs ~1.5 m of clear space around the robot. Ctrl-C stops cleanly.
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

MAX_KAPPA = 1.15          # the certified executable limit; ask for ALL of it
CRAWL = 0.16              # phase speed (m/s)
PHASE_MAX_S = 3.0
PHASE_MAX_DIST = 0.35     # keep-in-place: end phase after this much travel
BLOCKED_S = 1.5           # no motion this long -> wall; switch direction
TARGET_ROTATION = math.radians(170.0)


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Drill(Node):
    def __init__(self):
        super().__init__('three_point_drill')
        self.pose = None
        self.speed = 0.0
        self.yaw_rate = 0.0
        self.create_subscription(Odometry, '/odom', self._odom, 20)
        self.pub = self.create_publisher(Twist, '/cmd_vel_nav_raw', 10)

    def _odom(self, m):
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y,
                     yaw_from_quaternion(m.pose.pose.orientation))
        self.speed = m.twist.twist.linear.x
        self.yaw_rate = m.twist.twist.angular.z

    def command(self, v, kappa):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(v * kappa)
        self.pub.publish(t)

    def wait_pose(self):
        while self.pose is None:
            rclpy.spin_once(self, timeout_sec=0.2)

    def rotate(self, side):
        """One near-in-place rotation. side=+1 rotates nose left (CCW)."""
        self.wait_pose()
        origin = self.pose
        start_yaw = origin[2]
        rotated = 0.0
        last_yaw = start_yaw
        phase_sign = 1                     # +1 forward, -1 reverse
        phases = []
        max_footprint = 0.0
        print(f'--- rotation ({"CCW" if side > 0 else "CW"}), target '
              f'{math.degrees(TARGET_ROTATION):.0f} deg ---')
        while abs(rotated) < TARGET_ROTATION:
            p0 = self.pose
            t0 = time.monotonic()
            yaw0 = self.pose[2]
            last_motion = t0
            kappas = []
            # forward phase: lock TOWARD the turn; reverse: opposite lock.
            kappa = side * MAX_KAPPA * phase_sign
            reason = 'time'
            while True:
                rclpy.spin_once(self, timeout_sec=0.05)
                now = time.monotonic()
                self.command(phase_sign * CRAWL, kappa)
                dist = math.hypot(self.pose[0] - p0[0], self.pose[1] - p0[1])
                foot = math.hypot(self.pose[0] - origin[0],
                                  self.pose[1] - origin[1])
                max_footprint = max(max_footprint, foot)
                if abs(self.speed) > 0.04:
                    last_motion = now
                    if abs(self.speed) > 0.08:
                        kappas.append(abs(self.yaw_rate / self.speed))
                if dist >= PHASE_MAX_DIST:
                    reason = 'distance'
                    break
                if now - last_motion > BLOCKED_S and now - t0 > 1.0:
                    reason = 'BLOCKED'
                    break
                if now - t0 > PHASE_MAX_S:
                    break
                d_yaw = wrap(self.pose[2] - last_yaw)
                rotated += d_yaw
                last_yaw = self.pose[2]
                if abs(rotated) >= TARGET_ROTATION:
                    reason = 'target reached'
                    break
            gained = math.degrees(wrap(self.pose[2] - yaw0))
            achieved = (sorted(kappas)[len(kappas) // 2]
                        if kappas else 0.0)
            phases.append((phase_sign, gained, dist, achieved, reason))
            print(f"  phase {'FWD' if phase_sign > 0 else 'REV'}: "
                  f"{gained:+5.1f} deg in {dist:.2f} m, achieved "
                  f"|kappa| {achieved:.2f} / commanded {MAX_KAPPA} "
                  f"({reason})")
            self.command(0.0, 0.0)
            time.sleep(0.4)                # let it settle at the cusp
            phase_sign = -phase_sign
            if len(phases) > 14:
                print('  giving up after 14 phases')
                break
        self.command(0.0, 0.0)
        total = math.degrees(wrap(self.pose[2] - start_yaw))
        print(f'  => rotated {total:+.0f} deg in {len(phases)} phases, '
              f'footprint {max_footprint:.2f} m')
        return len(phases), max_footprint


def main():
    rotations = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    rclpy.init()
    node = Drill()
    try:
        for i in range(rotations):
            side = 1 if i % 2 == 0 else -1
            node.rotate(side)
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.command(0.0, 0.0)
        time.sleep(0.2)
        node.command(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()
        print('drill done (robot commanded to stop)')


if __name__ == '__main__':
    main()
