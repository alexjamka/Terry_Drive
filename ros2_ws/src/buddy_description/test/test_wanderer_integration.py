#!/usr/bin/env python3
"""Integration test: wanderer -> scan_guard -> /cmd_vel, as launched on the robot."""
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

N = 360


def make_scan(front_m=None):
    s = LaserScan()
    s.header.frame_id = 'laser'
    s.angle_min = -math.pi
    s.angle_max = math.pi
    s.angle_increment = 2 * math.pi / N
    s.range_min = 0.10
    s.range_max = 10.0
    r = [8.0] * N
    if front_m is not None:
        for d in range(-25, 26):   # wall straight ahead of the ROBOT
            idx = int(round((math.radians(180 + d) - s.angle_min) / s.angle_increment)) % N
            r[idx] = front_m
    s.ranges = r
    return s


class H(Node):
    def __init__(self):
        super().__init__('itest_wander')
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.cmd = []
        self.state = []
        self.t0 = time.time()
        self.create_subscription(
            Twist, '/cmd_vel', lambda m: self.cmd.append((time.time() - self.t0, m)), 10)
        self.create_subscription(
            String, '/wander_status',
            lambda m: self.state.append(json.loads(m.data)['state']), 10)

    def run(self, seconds, front_m):
        self.t0 = time.time()
        self.cmd.clear()
        self.state.clear()
        end = time.time() + seconds
        while time.time() < end:
            s = make_scan(front_m)
            s.header.stamp = self.get_clock().now().to_msg()
            self.scan_pub.publish(s)
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.cmd, self.state


fails = []


def check(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}{'  ' + detail if detail else ''}")
    if not cond:
        fails.append(name)


def main():
    rclpy.init()
    h = H()
    time.sleep(2.0)

    print("1. Open room -> should cruise forward.")
    cmd, st = h.run(4.0, None)
    fwd = [c.linear.x for _, c in cmd if c.linear.x > 0.01]
    check("drives forward", len(fwd) > 5, f"{len(fwd)} forward cmds, max={max(fwd, default=0):.2f}")
    check("reaches DRIVE state", 'DRIVE' in st, f"states seen: {sorted(set(st))}")

    print("\n2. Wall 0.5 m ahead -> should turn, not push into it.")
    cmd, st = h.run(5.0, 0.5)
    turning = [c.angular.z for _, c in cmd if abs(c.angular.z) > 0.05]
    pushing = [(t, c.linear.x) for t, c in cmd if c.linear.x > 0.01]
    check("rotates to escape", len(turning) > 5, f"{len(turning)} turn cmds")
    # Commands in the first moments are either in flight from the previous
    # phase or the one cycle before the wanderer has seen the new scan. The
    # property that matters is that forward motion stops and STAYS stopped.
    settle = 0.5
    sustained = [(t, v) for t, v in pushing if t > settle]
    last = max((t for t, _ in pushing), default=0.0)
    check("no forward motion once the wall has been seen",
          not sustained,
          f"{len(sustained)} cmds after {settle}s"
          + (f", worst {max(v for _, v in sustained):.3f} m/s" if sustained else ""))
    check("forward ceases promptly", last < settle,
          f"{len(pushing)} transient cmds, last at t={last:.2f}s")
    check("enters TURN state", 'TURN' in st, f"states seen: {sorted(set(st))}")

    print("\n3. Wall clears -> should resume driving.")
    cmd, st = h.run(5.0, None)
    fwd = [c.linear.x for _, c in cmd if c.linear.x > 0.01]
    check("resumes forward", len(fwd) > 5, f"{len(fwd)} forward cmds")

    h.destroy_node()
    rclpy.shutdown()
    print("\n" + ("ALL PASS" if not fails else f"FAILURES: {fails}"))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
