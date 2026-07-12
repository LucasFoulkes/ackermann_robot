#!/usr/bin/env python3
"""Measure forward and reverse ESC breakaway with slow neutral-centered ramps."""
import argparse, csv, math, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

ADDR, CHANNEL, PRESCALE = 0x40, 14, 30
US_PER_TICK = 1e6 / (25_000_000 / 4096 / (PRESCALE + 1)) / 4096
NEUTRAL = 1500.0


def open_bus():
    try:
        from smbus2 import SMBus
    except ImportError:
        from smbus import SMBus
    bus = SMBus(1)
    bus.write_byte_data(ADDR, 0x00, 0x10)
    bus.write_byte_data(ADDR, 0xFE, PRESCALE)
    bus.write_byte_data(ADDR, 0x00, 0x00)
    time.sleep(.005)
    bus.write_byte_data(ADDR, 0x00, 0xA0)
    return bus


def write_tick(bus, tick):
    tick = max(0, min(4095, int(tick)))
    bus.write_i2c_block_data(ADDR, 0x06 + 4*CHANNEL,
                             [0, 0, tick & 255, tick >> 8])


def neutral(bus):
    write_tick(bus, round(NEUTRAL / US_PER_TICK))


class Experiment(Node):
    def __init__(self, args, writer):
        super().__init__('esc_bidirectional_breakaway')
        self.args, self.writer = args, writer
        self.pose = self.previous = self.origin = None
        self.speed = self.signed_speed = 0.0
        self.odom_time = None
        self.front = self.rear = math.inf
        self.direction = 'forward'
        self.trial = 0
        self.odom_seq = 0
        self.velocity_history = []
        self.error, self.ticks = 0.0, []
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 20)

    def on_tf(self, msg):
        for tf in msg.transforms:
            if (tf.header.frame_id.lstrip('/') != 'odom' or
                    tf.child_frame_id.lstrip('/') != 'base_link'):
                continue
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec*1e-9
            p, q = tf.transform.translation, tf.transform.rotation
            heading = math.atan2(2*(q.w*q.z + q.x*q.y),
                                 1-2*(q.y*q.y + q.z*q.z))
            current = stamp, p.x, p.y, heading
            if self.previous and stamp > self.previous[0]:
                dt = stamp-self.previous[0]
                dx, dy = p.x-self.previous[1], p.y-self.previous[2]
                v = (dx*math.cos(heading) + dy*math.sin(heading))/dt
                self.signed_speed = .75*self.signed_speed + .25*v
                self.speed = abs(self.signed_speed)
                self.velocity_history.append(self.signed_speed)
                del self.velocity_history[:-500]
            self.previous = self.pose = current
            self.odom_seq += 1
            self.odom_time = time.monotonic()
            self.origin = self.origin or current

    def on_scan(self, msg):
        front, rear = [], []
        for i, distance in enumerate(msg.ranges):
            if not math.isfinite(distance) or not msg.range_min <= distance <= msg.range_max:
                continue
            angle = msg.angle_min + i*msg.angle_increment
            if abs(angle) <= math.radians(25): front.append(distance)
            if abs(abs(angle)-math.pi) <= math.radians(25): rear.append(distance)
        self.front, self.rear = min(front, default=math.inf), min(rear, default=math.inf)

    @property
    def distance(self):
        if not self.pose or not self.origin: return 0.0
        return math.hypot(self.pose[1]-self.origin[1], self.pose[2]-self.origin[2])

    def safe(self):
        if self.odom_time is None or time.monotonic()-self.odom_time > .5:
            return False, 'odometry missing/stale'
        clearance = self.front if self.direction == 'forward' else self.rear
        if clearance < self.args.stop_range:
            return False, f'{self.direction} obstacle at {clearance:.2f} m'
        if self.distance >= self.args.max_distance:
            return False, f'distance limit {self.distance:.2f} m'
        if self.speed >= self.args.max_speed:
            return False, f'overspeed {self.signed_speed:+.3f} m/s'
        return True, ''

    def spin_for(self, seconds):
        end = time.monotonic()+seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.03)

    def apply(self, bus, pulse, seconds, expected_sign):
        self.error, self.ticks = 0.0, []
        end = time.monotonic()+seconds
        while rclpy.ok() and time.monotonic() < end:
            value = pulse/US_PER_TICK + self.error
            tick = round(value)
            self.error = value-tick
            write_tick(bus, tick); self.ticks.append(tick)
            rclpy.spin_once(self, timeout_sec=.02)
            okay, reason = self.safe()
            if not okay: return reason
            if expected_sign*self.signed_speed >= self.args.breakaway_speed:
                return 'breakaway'
        return 'continue'

    def log(self, elapsed, pulse, result):
        avg_tick = sum(self.ticks)/len(self.ticks) if self.ticks else math.nan
        clearance = self.front if self.direction == 'forward' else self.rear
        self.writer.writerow([self.trial, self.direction, f'{elapsed:.3f}', f'{pulse:.2f}',
                              f'{avg_tick:.4f}', f'{self.signed_speed:.6f}',
                              f'{self.distance:.6f}', f'{clearance:.3f}', result])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--arm', action='store_true')
    p.add_argument('--step-us', type=float, default=.5)
    p.add_argument('--step-seconds', type=float, default=.30)
    p.add_argument('--breakaway-speed', type=float, default=.05)
    p.add_argument('--max-speed', type=float, default=.25)
    p.add_argument('--max-distance', type=float, default=.20)
    p.add_argument('--stop-range', type=float, default=.20)
    p.add_argument('--forward-start-us', type=float, default=1420.0)
    p.add_argument('--reverse-start-us', type=float, default=1580.0)
    p.add_argument('--cycles', type=int, default=3,
                   help='one forward and one reverse trial per cycle')
    p.add_argument('--output', default='esc_breakaway_repeated.csv')
    args, ros_args = p.parse_known_args()
    if not args.arm: p.error('refusing hardware output without --arm')
    if not .25 <= args.step_us <= 1.0: p.error('--step-us must be 0.25..1.0')
    if not 1 <= args.cycles <= 10: p.error('--cycles must be 1..10')
    if not 1350 <= args.forward_start_us < 1500 < args.reverse_start_us <= 1650:
        p.error('unsafe ramp bounds')
    rclpy.init(args=ros_args); bus = None
    with open(args.output, 'w', newline='') as output:
        writer = csv.writer(output)
        writer.writerow(['trial','direction','elapsed_s','pulse_us','average_tick',
                         'signed_speed_mps','distance_m','clearance_m','result'])
        node = Experiment(args, writer)
        try:
            bus = open_bus(); neutral(bus); node.spin_for(3)
            started = time.monotonic()
            definitions = {
                'forward': (args.forward_start_us, 1350., -args.step_us, 1.),
                'reverse': (args.reverse_start_us, 1650., args.step_us, -1.),
            }
            sequence = []
            for cycle in range(args.cycles):
                sequence.extend(('forward', 'reverse') if cycle % 2 == 0
                                else ('reverse', 'forward'))
            for trial, direction in enumerate(sequence, 1):
                first, limit, delta, sign = definitions[direction]
                neutral(bus); node.spin_for(2)
                node.trial = trial
                node.direction, node.origin, node.signed_speed = direction, node.pose, 0.0
                pulse, found = first, False
                while (pulse >= limit if delta < 0 else pulse <= limit):
                    result = node.apply(bus, pulse, args.step_seconds, sign)
                    node.log(time.monotonic()-started, pulse, result); output.flush()
                    print(f'trial {trial}/{len(sequence)} {direction} {pulse:.2f} us: '
                          f'{node.signed_speed:+.3f} m/s ({result})')
                    if result == 'breakaway': found = True; break
                    if result != 'continue': raise RuntimeError(result)
                    pulse += delta
                neutral(bus)
                if not found: raise RuntimeError(f'{direction} breakaway not found')
        except (KeyboardInterrupt, RuntimeError) as error:
            print(f'STOP: {error}')
        finally:
            if bus:
                for _ in range(5): neutral(bus); time.sleep(.02)
                bus.close()
            node.destroy_node(); rclpy.shutdown()
            print(f'ESC neutral. Log: {args.output}')


if __name__ == '__main__': main()
