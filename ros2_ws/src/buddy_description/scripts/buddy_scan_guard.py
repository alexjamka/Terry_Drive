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
from rclpy.qos import qos_profile_sensor_data
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
        # Must not clip the behaviours' turn commands. Measured 2026-11-11 by
        # sweeping commanded speed against /odom distance:
        #
        #   cmd 0.20 m/s (PWM 166) -> 43% achieved   (stalls intermittently)
        #   cmd 0.25 m/s (PWM 208) -> 100%
        #   cmd 0.30 m/s (PWM 249) -> 144%
        #
        # PWM 249 really produces ~0.43 m/s, so the firmware's v*833 scale is
        # wrong (true constant ~581) and MIN_PWM 150 is a real floor of
        # 0.26 m/s, not the 0.18 m/s previously documented. The slowest
        # non-stalling in-place turn is therefore 2*0.26/0.3302 = 1.57 rad/s.
        # A 1.5 clamp sat just under that, so every turn the wanderer asked
        # for was throttled into the stall zone. Must stay above 1.57.
        self.declare_parameter('max_angular', 2.0)
        # The base is effectively bang-bang: MIN_PWM 150 floors every nonzero
        # command, and under real load on the floor PWM 150 does NOT reliably
        # break away (the "149 = both wheels turn" bench figure was measured
        # with the wheels ELEVATED). So a scaled-down forward command is not a
        # slow robot, it is a juddering one.
        #
        # Measured on the 185 s mapping run of 2026-11-12: front clearance sat
        # between 0.35 and 0.75 m for the whole run, so the proportional zone
        # below scaled 0.28 m/s to ~0.1 m/s on 2566 of 3700 ticks. front_blocked
        # fired ZERO times - the robot was never stopped by the guard, just
        # throttled into stalling. It covered 4.65 m of wheel travel in 185 s
        # (0.99 m net) and the SLAM map stopped growing after 24 s.
        #
        # Fix: scale as before, but never emit a nonzero speed below what the
        # drivetrain can actually sustain. Either it moves or it stops.
        # Closing the velocity loop (PI) is what would buy back a real slow
        # zone; until then this keeps the guard honest.
        self.declare_parameter('min_linear', 0.26)
        # Minimum returns required in the rear sector before reverse is
        # trusted. An empty sector reports +inf, which reads as "clear" but
        # means "blind" - and astern is structurally blind on this robot: the
        # neck pole sits ~0.29 m behind the lidar, and a scan taken 2026-11-12
        # had 68% of astern rays returning nothing, with a 20 deg wedge at
        # zero returns. The robot was reversed into a wall that the guard was
        # reporting as 0.98 m clear. Rotation is never blocked, so refusing to
        # reverse blind costs nothing: the robot can always turn and drive out
        # forwards, where the lidar can actually see.
        self.declare_parameter('min_rear_points', 8)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('scan_timeout', 1.0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('base_frame', 'BuddyBaseFootprint_link')
        self.declare_parameter('scan_frame', 'laser')
        # Fallback only - the real value is read from TF at startup.
        self.declare_parameter('scan_yaw_offset', 0.0)
        # Sensor floor only. This used to be 0.20 to reject the extrusion
        # astern, which blinded the FRONT sector at close range and caused a
        # collision - the self-return is now masked by angle instead.
        self.declare_parameter('min_valid_range', 0.05)
        # The extrusion is ~0.117 m dead astern; drop returns only if they are
        # both beyond this bearing from straight ahead AND this close.
        self.declare_parameter('self_mask_from_deg', 165.0)
        self.declare_parameter('self_mask_range', 0.25)

        g = lambda n: self.get_parameter(n).value
        self.stop_distance = g('stop_distance')
        self.slow_distance = g('slow_distance')
        self.rear_stop_distance = g('rear_stop_distance')
        self.front_half = math.radians(g('front_half_angle_deg'))
        self.side_span = math.radians(g('side_span_deg'))
        self.max_linear = g('max_linear')
        self.max_angular = g('max_angular')
        self.min_linear = g('min_linear')
        self.min_rear_points = g('min_rear_points')
        self.cmd_timeout = g('cmd_timeout')
        self.scan_timeout = g('scan_timeout')
        self.base_frame = g('base_frame')
        self.scan_frame = g('scan_frame')
        self.min_valid_range = g('min_valid_range')
        self.self_mask_from_deg = g('self_mask_from_deg')
        self.self_mask_range = g('self_mask_range')

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
        self._counts = {}
        self._last_scan_time = None

        # resolve_scan_yaw() below spins the node while waiting for TF, which
        # can dispatch _on_scan before scan_yaw is assigned. Define it first
        # and let the callback no-op until it is known, or startup races to an
        # AttributeError depending on whether TF resolves on the first try.
        self.scan_yaw = None

        # Lidar drivers publish sensor data BEST_EFFORT. Subscribing with the
        # default RELIABLE profile is QoS-incompatible, and ROS silently
        # delivers nothing - the guard just reports no_scan forever.
        self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
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
        if self.scan_yaw is None:
            return  # mounting yaw not resolved yet; see __init__
        self._sectors, self._counts = sector_distances(
            msg, self.scan_yaw, self.front_half, self.side_span,
            self.min_valid_range, self.self_mask_from_deg,
            self.self_mask_range, with_counts=True)
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

            # See min_linear above: scaling below the achievable floor makes
            # the base judder instead of slow, so the scaled command is lifted
            # back to min_linear rather than handed over as an unachievable
            # fraction. stop_distance remains the real safety line.
            if lin > 0.0:
                front = s['front']
                if front <= self.stop_distance:
                    lin = 0.0
                    reason = 'front_blocked'
                elif front < self.slow_distance:
                    span = self.slow_distance - self.stop_distance
                    scale = (front - self.stop_distance) / span if span > 0 else 0.0
                    lin_req = lin
                    lin *= max(0.0, min(1.0, scale))
                    if 0.0 < lin < self.min_linear:
                        # Never hand down a speed the base cannot sustain - but
                        # never hand UP more than was asked for either, or the
                        # governor would accelerate the robot it is protecting.
                        # If the request itself is under the floor it simply
                        # cannot be slowed, and passes through unchanged.
                        lin = min(self.min_linear, lin_req)
                        reason = 'slow_floor'
                    else:
                        reason = 'slowing'
            elif lin < 0.0:
                # Coverage BEFORE distance: see min_rear_points. Reversing on
                # the strength of an empty sector is how the robot hit a wall.
                if self._counts.get('rear', 0) < self.min_rear_points:
                    lin = 0.0
                    reason = 'rear_unknown'
                elif s['rear'] <= self.rear_stop_distance:
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
            'points': dict(self._counts),
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
