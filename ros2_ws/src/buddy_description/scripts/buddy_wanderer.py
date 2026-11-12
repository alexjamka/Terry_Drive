#!/usr/bin/env python3
"""
Random-walk exploration.

Drives the robot around a room without a map or a goal: cruise forward, turn
away when something gets close, occasionally turn for no reason so it doesn't
just trace the same loop forever.

Publishes to /cmd_vel_raw, so buddy_scan_guard still has the final say on the
motors. This node decides WHERE to go; the guard decides whether that's safe.
The two read /scan independently and deliberately - the wanderer turns early to
look natural, the guard stops late as a backstop.

    ros2 topic pub --once /wander_enable std_msgs/Bool "{data: false}"   # pause

State machine:

    DRIVE ──front close──> TURN ──clear──> DRIVE
      ^                     │
      │                 can't clear
      │                     v
      └────────────────── BACK
"""

import json
import math
import random

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from buddy_scan_utils import resolve_scan_yaw, sector_distances

DRIVE, TURN, BACK = 'DRIVE', 'TURN', 'BACK'


class Wanderer(Node):

    def __init__(self):
        super().__init__('buddy_wanderer')

        # Must stay above the drivetrain's break-away (0.18 m/s, PWM 149).
        self.declare_parameter('cruise_speed', 0.20)
        # 1.2 rad/s puts each wheel at 0.198 m/s, just above break-away.
        self.declare_parameter('turn_speed', 1.2)
        self.declare_parameter('back_speed', 0.20)
        # Turn earlier than the guard stops (0.35 m), so normal wandering never
        # trips the safety layer - but not so early that an ordinary room never
        # looks clear. Measured live 2026-11-11: front clearance in this space
        # sat at 0.55-0.90 m, so the original 0.70/0.95 pair meant the robot
        # almost never saw "clear" and spent its time pirouetting, covering
        # 0.47 m in two minutes. 0.50/0.65 leaves 0.15 m over the guard's stop
        # and lets it commit to driving in normal indoor clutter.
        self.declare_parameter('turn_distance', 0.50)
        self.declare_parameter('clear_distance', 0.65)
        self.declare_parameter('min_turn_time', 0.6)
        self.declare_parameter('max_turn_time', 6.0)
        self.declare_parameter('back_time', 1.2)
        # Occasional aimless turns, so it explores instead of pacing a groove.
        self.declare_parameter('curiosity_interval', 12.0)
        self.declare_parameter('curiosity_turn_time', 1.0)
        self.declare_parameter('front_half_angle_deg', 35.0)
        self.declare_parameter('side_span_deg', 55.0)
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('start_enabled', True)
        self.declare_parameter('base_frame', 'BuddyBaseFootprint_link')
        self.declare_parameter('scan_frame', 'laser')
        self.declare_parameter('scan_yaw_offset', 0.0)
        # Rejects the 40x40 mm extrusion behind the lidar, seen at 0.117 m.
        self.declare_parameter('min_valid_range', 0.20)

        g = lambda n: self.get_parameter(n).value
        self.cruise_speed = g('cruise_speed')
        self.turn_speed = g('turn_speed')
        self.back_speed = g('back_speed')
        self.turn_distance = g('turn_distance')
        self.clear_distance = g('clear_distance')
        self.min_turn_time = g('min_turn_time')
        self.max_turn_time = g('max_turn_time')
        self.back_time = g('back_time')
        self.curiosity_interval = g('curiosity_interval')
        self.curiosity_turn_time = g('curiosity_turn_time')
        self.front_half = math.radians(g('front_half_angle_deg'))
        self.side_span = math.radians(g('side_span_deg'))
        self.enabled = g('start_enabled')
        self.min_valid_range = g('min_valid_range')

        if self.clear_distance <= self.turn_distance:
            self.get_logger().warn(
                'clear_distance <= turn_distance; the robot may oscillate '
                'between DRIVE and TURN at the threshold.')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state = DRIVE
        self.state_since = self.get_clock().now()
        self.turn_dir = 1.0
        self.sectors = None
        self.last_curiosity = self.get_clock().now()
        self.curiosity_until = None

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
        self.create_subscription(Bool, '/wander_enable', self._on_enable, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.status_pub = self.create_publisher(String, '/wander_status', 10)

        self.scan_yaw = resolve_scan_yaw(self, g('base_frame'), g('scan_frame'))
        if self.scan_yaw is None:
            self.scan_yaw = g('scan_yaw_offset')
            self.get_logger().warn(
                f'TF unavailable; using scan_yaw_offset={self.scan_yaw:.3f} rad.')

        self.create_timer(1.0 / max(g('rate'), 1.0), self._tick)
        self.get_logger().info(
            f'wanderer up ({"enabled" if self.enabled else "paused"}): '
            f'cruise={self.cruise_speed} turn_at={self.turn_distance}m')

    def _on_scan(self, msg: LaserScan):
        if self.scan_yaw is None:
            return  # mounting yaw not resolved yet; see __init__
        self.sectors = sector_distances(
            msg, self.scan_yaw, self.front_half, self.side_span,
            self.min_valid_range)

    def _on_enable(self, msg: Bool):
        if msg.data != self.enabled:
            self.get_logger().info(f'wander {"enabled" if msg.data else "paused"}')
        self.enabled = msg.data

    def _elapsed(self):
        return (self.get_clock().now() - self.state_since).nanoseconds / 1e9

    def _goto(self, state):
        self.state = state
        self.state_since = self.get_clock().now()

    def _pick_turn_dir(self):
        """Turn toward whichever side has more room; coin-flip if equal."""
        s = self.sectors
        left, right = s['left'], s['right']
        if left == right:
            return random.choice((-1.0, 1.0))
        return 1.0 if left > right else -1.0

    def _tick(self):
        cmd = Twist()

        if not self.enabled or self.sectors is None:
            self.cmd_pub.publish(cmd)
            self._publish_status('idle' if not self.enabled else 'waiting_for_scan')
            return

        front = self.sectors['front']

        if self.state == DRIVE:
            if front <= self.turn_distance:
                self.turn_dir = self._pick_turn_dir()
                self._goto(TURN)
            elif self.curiosity_until is not None:
                # In-place turn, NOT an arc. An arc's inner wheel lands below
                # the 0.18 m/s break-away and simply stalls, so the robot would
                # lurch straight instead of curving.
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_speed * self.turn_dir
                if self.get_clock().now() >= self.curiosity_until:
                    self.curiosity_until = None
            else:
                cmd.linear.x = self.cruise_speed
                since = (self.get_clock().now() - self.last_curiosity).nanoseconds / 1e9
                if since > self.curiosity_interval:
                    # Gentle arc in a random direction for a moment.
                    self.turn_dir = random.choice((-1.0, 1.0))
                    self.curiosity_until = self.get_clock().now() + rclpy.duration.Duration(
                        seconds=self.curiosity_turn_time)
                    self.last_curiosity = self.get_clock().now()

        elif self.state == TURN:
            cmd.angular.z = self.turn_speed * self.turn_dir
            t = self._elapsed()
            if t >= self.min_turn_time and front > self.clear_distance:
                self.last_curiosity = self.get_clock().now()
                self._goto(DRIVE)
            elif t > self.max_turn_time:
                # Spun a full sweep without finding room - probably wedged.
                self.get_logger().warn('turn timed out, backing up')
                self._goto(BACK)

        elif self.state == BACK:
            cmd.linear.x = -self.back_speed
            if self._elapsed() > self.back_time:
                self.turn_dir = self._pick_turn_dir()
                self._goto(TURN)

        self.cmd_pub.publish(cmd)
        self._publish_status(self.state)

    def _publish_status(self, state):
        s = self.sectors or {}
        self.status_pub.publish(String(data=json.dumps({
            'state': state,
            'turn_dir': 'left' if self.turn_dir > 0 else 'right',
            'sectors': {k: (None if math.isinf(v) else round(v, 2))
                        for k, v in s.items()},
        })))


def main(args=None):
    rclpy.init(args=args)
    node = Wanderer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
