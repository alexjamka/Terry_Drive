#!/usr/bin/env python3
"""
Lidar safety governor.

Sits between whatever is driving the robot and the motors. Behaviour nodes
publish to /cmd_vel_raw; this node applies obstacle limits from /scan and
publishes the result to /cmd_vel, which the serial bridge consumes.

    follow_controller ─┐
                       ├─> /cmd_vel_raw ─> [scan_guard] ─> /cmd_vel ─> ESP32
    wanderer ──────────┘                        ^
                                              /scan

Putting the limiting here rather than inside each behaviour means obstacle
avoidance can't be forgotten by a new behaviour, and there's one place to tune.

Rules:
  * Forward motion is scaled down inside slow_distance and cut at stop_distance.
  * Rotation is NEVER blocked - spinning in place is how the robot escapes, and
    a guard that freezes rotation deadlocks against a wall.
  * Reverse is limited by the rear sector (the X4 sees 360 deg).
  * Missing or stale scans, or a silent driver, mean stop. Fail closed.
"""

import json
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from buddy_scan_utils import resolve_scan_yaw, sector_distances


class ScanGuard(Node):

    def __init__(self):
        super().__init__('buddy_scan_guard')

        self.declare_parameter('stop_distance', 0.35)
        self.declare_parameter('slow_distance', 0.75)
        self.declare_parameter('rear_stop_distance', 0.30)
        self.declare_parameter('front_half_angle_deg', 35.0)
        self.declare_parameter('side_span_deg', 55.0)
        self.declare_parameter('max_linear', 0.30)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('scan_timeout', 1.0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('base_frame', 'BuddyBaseFootprint_link')
        self.declare_parameter('scan_frame', 'laser')
        # Fallback only - the real value is read from TF at startup.
        self.declare_parameter('scan_yaw_offset', math.pi)

        g = lambda n: self.get_parameter(n).value
        self.stop_distance = g('stop_distance')
        self.slow_distance = g('slow_distance')
        self.rear_stop_distance = g('rear_stop_distance')
        self.front_half = math.radians(g('front_half_angle_deg'))
        self.side_span = math.radians(g('side_span_deg'))
        self.max_linear = g('max_linear')
        self.max_angular = g('max_angular')
        self.cmd_timeout = g('cmd_timeout')
        self.scan_timeout = g('scan_timeout')
        self.base_frame = g('base_frame')
        self.scan_frame = g('scan_frame')

        if self.slow_distance <= self.stop_distance:
            self.get_logger().warn(
                f'slow_distance ({self.slow_distance}) <= stop_distance '
                f'({self.stop_distance}); forward motion will be on/off with no '
                f'slow zone.')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._last_cmd = Twist()
        self._last_cmd_time = None
        self._sectors = None
        self._last_scan_time = None

        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.create_subscription(Twist, '/cmd_vel_raw', self._on_cmd, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/scan_guard_status', 10)

        # Resolve the lidar mounting from TF; the X4 is mounted backwards, and
        # getting this wrong inverts every avoidance decision.
        self.scan_yaw = resolve_scan_yaw(self, self.base_frame, self.scan_frame)
        if self.scan_yaw is None:
            self.scan_yaw = self.get_parameter('scan_yaw_offset').value
            self.get_logger().warn(
                f'TF {self.base_frame} <- {self.scan_frame} unavailable; falling '
                f'back to scan_yaw_offset={self.scan_yaw:.3f} rad. Verify this '
                f'matches the physical mounting before trusting avoidance.')
        else:
            self.get_logger().info(
                f'Lidar mounting yaw from TF: {math.degrees(self.scan_yaw):.1f} deg')

        self.create_timer(1.0 / max(g('publish_rate'), 1.0), self._tick)
        self.get_logger().info(
            f'scan_guard up: stop<{self.stop_distance}m slow<{self.slow_distance}m '
            f'front=+/-{math.degrees(self.front_half):.0f}deg')

    def _on_scan(self, msg: LaserScan):
        self._sectors = sector_distances(
            msg, self.scan_yaw, self.front_half, self.side_span)
        self._last_scan_time = self.get_clock().now()

    def _on_cmd(self, msg: Twist):
        self._last_cmd = msg
        self._last_cmd_time = self.get_clock().now()

    def _age(self, stamp):
        if stamp is None:
            return math.inf
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def _tick(self):
        out = Twist()
        reason = 'ok'

        cmd_age = self._age(self._last_cmd_time)
        scan_age = self._age(self._last_scan_time)

        if cmd_age > self.cmd_timeout:
            reason = 'no_command'
        elif scan_age > self.scan_timeout or self._sectors is None:
            # No eyes: refuse to move rather than drive blind.
            reason = 'no_scan'
        else:
            s = self._sectors
            lin = max(-self.max_linear, min(self.max_linear, self._last_cmd.linear.x))
            ang = max(-self.max_angular, min(self.max_angular, self._last_cmd.angular.z))

            if lin > 0.0:
                front = s['front']
                if front <= self.stop_distance:
                    lin = 0.0
                    reason = 'front_blocked'
                elif front < self.slow_distance:
                    span = self.slow_distance - self.stop_distance
                    scale = (front - self.stop_distance) / span if span > 0 else 0.0
                    lin *= max(0.0, min(1.0, scale))
                    reason = 'slowing'
            elif lin < 0.0 and s['rear'] <= self.rear_stop_distance:
                lin = 0.0
                reason = 'rear_blocked'

            out.linear.x = lin
            out.angular.z = ang

        self.cmd_pub.publish(out)

        s = self._sectors or {}
        self.status_pub.publish(String(data=json.dumps({
            'reason': reason,
            'cmd_out': {'lin': round(out.linear.x, 3), 'ang': round(out.angular.z, 3)},
            'cmd_in': {'lin': round(self._last_cmd.linear.x, 3),
                       'ang': round(self._last_cmd.angular.z, 3)},
            'sectors': {k: (None if math.isinf(v) else round(v, 2))
                        for k, v in s.items()},
            'scan_age': round(scan_age, 2) if math.isfinite(scan_age) else None,
        })))


def main(args=None):
    rclpy.init(args=args)
    node = ScanGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())  # don't leave the base rolling
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
