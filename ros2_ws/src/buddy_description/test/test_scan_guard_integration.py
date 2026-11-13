#!/usr/bin/env python3
"""Integration test: does buddy_scan_guard actually govern /cmd_vel?"""
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

N = 360


def make_scan(front_m=None):
    """front_m: obstacle distance straight ahead of the ROBOT, or None for clear."""
    s = LaserScan()
    s.header.frame_id = 'laser'
    s.angle_min = -math.pi
    s.angle_max = math.pi
    s.angle_increment = 2 * math.pi / N
    s.range_min = 0.10
    s.range_max = 10.0
    r = [8.0] * N
    if front_m is not None:
        # robot-front == laser angle pi, because the X4 is mounted backwards
        for d in range(-20, 21):
            idx = int(round((math.radians(180 + d) - s.angle_min) / s.angle_increment)) % N
            r[idx] = front_m
    s.ranges = r
    return s


class Harness(Node):
    def __init__(self):
        super().__init__('itest_harness')
        # Match a real lidar driver: sensor data is BEST_EFFORT.
        self.scan_pub = self.create_publisher(
            LaserScan, '/scan', qos_profile_sensor_data)
        self.raw_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.got = []
        self.create_subscription(Twist, '/cmd_vel', lambda m: self.got.append(m), 10)

    def drive(self, seconds, front_m, lin, ang):
        """Publish a scan + raw command for a while, return the last /cmd_vel."""
        self.got.clear()
        t = Twist()
        t.linear.x = lin
        t.angular.z = ang
        end = time.time() + seconds
        while time.time() < end:
            s = make_scan(front_m)
            s.header.stamp = self.get_clock().now().to_msg()
            self.scan_pub.publish(s)
            self.raw_pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.got[-1] if self.got else None


fails = []


def check(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}{'  ' + detail if detail else ''}")
    if not cond:
        fails.append(name)


def main():
    rclpy.init()
    h = Harness()
    time.sleep(2.0)  # let the guard resolve TF and settle

    print("1. Clear ahead, commanded 0.20 m/s forward.")
    out = h.drive(2.0, None, 0.20, 0.0)
    check("command passes through", out is not None and abs(out.linear.x - 0.20) < 0.02,
          f"lin={out.linear.x:.3f}" if out else "no /cmd_vel")

    print("\n2. Obstacle 0.25 m ahead (inside stop_distance 0.35).")
    out = h.drive(2.0, 0.25, 0.20, 0.0)
    check("forward is cut", out is not None and abs(out.linear.x) < 1e-6,
          f"lin={out.linear.x:.3f}" if out else "no /cmd_vel")

    print("\n3. Same obstacle, but commanded to rotate.")
    out = h.drive(2.0, 0.25, 0.0, 0.6)
    check("rotation still allowed", out is not None and abs(out.angular.z - 0.6) < 0.02,
          f"ang={out.angular.z:.3f}" if out else "no /cmd_vel")

    print("\n4. Obstacle 0.55 m ahead (slow zone 0.35-0.75).")
    # Request ABOVE the achievable floor (min_linear 0.26): scaling is
    # meaningful, so the output must drop - but not below what the drivetrain
    # can sustain, because a scaled-but-unachievable command makes the base
    # judder rather than crawl. See the min_linear note in buddy_scan_guard.
    out = h.drive(2.0, 0.55, 0.40, 0.0)
    ok = out is not None and 0.26 - 1e-6 <= out.linear.x < 0.40
    check("forward is scaled down, not below the achievable floor", ok,
          f"lin={out.linear.x:.3f}" if out else "no /cmd_vel")

    print("\n4b. Same obstacle, request already BELOW the floor.")
    # 0.20 m/s is under min_linear, so it cannot be slowed at all. It must pass
    # through unchanged - and must NOT be lifted to 0.26, or the guard would
    # speed up the robot it exists to protect.
    out = h.drive(2.0, 0.55, 0.20, 0.0)
    ok = out is not None and abs(out.linear.x - 0.20) < 0.02
    check("sub-floor request passes through, never raised", ok,
          f"lin={out.linear.x:.3f}" if out else "no /cmd_vel")

    print("\n5. Stop publishing commands -> watchdog.")
    h.got.clear()
    end = time.time() + 2.0
    while time.time() < end:
        s = make_scan(None)
        s.header.stamp = h.get_clock().now().to_msg()
        h.scan_pub.publish(s)
        rclpy.spin_once(h, timeout_sec=0.05)
    out = h.got[-1] if h.got else None
    check("output zeroed on silent driver",
          out is not None and abs(out.linear.x) < 1e-6 and abs(out.angular.z) < 1e-6,
          f"lin={out.linear.x:.3f} ang={out.angular.z:.3f}" if out else "no /cmd_vel")

    print("\n6. Stop publishing scans -> fail closed.")
    h.got.clear()
    t = Twist()
    t.linear.x = 0.20
    end = time.time() + 3.0
    while time.time() < end:
        h.raw_pub.publish(t)
        rclpy.spin_once(h, timeout_sec=0.05)
    out = h.got[-1] if h.got else None
    check("output zeroed on stale scan",
          out is not None and abs(out.linear.x) < 1e-6,
          f"lin={out.linear.x:.3f}" if out else "no /cmd_vel")

    h.destroy_node()
    rclpy.shutdown()
    print("\n" + ("ALL PASS" if not fails else f"FAILURES: {fails}"))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
