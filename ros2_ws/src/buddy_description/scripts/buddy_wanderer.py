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

        # Measured 2026-11-12/13, superseding the old 0.18 m/s figure: the
        # firmware's v*833 scale is wrong (true constant ~581), so MIN_PWM 150
        # is a real floor of 0.26 m/s. Commanding 0.20 lands at PWM 166 and
        # achieves only 43% of it - the wheels stall intermittently and the
        # robot judders instead of driving. 0.28 is PWM 233, comfortably clear.
        self.declare_parameter('cruise_speed', 0.28)
        # The slowest non-stalling in-place turn is 2*0.26/0.3302 = 1.57 rad/s.
        # The old 1.2 was BELOW that, so every turn was a stall.
        self.declare_parameter('turn_speed', 1.6)
        self.declare_parameter('back_speed', 0.28)
        # Turn earlier than the guard stops (0.35 m), so normal wandering never
        # trips the safety layer - but not so early that an ordinary room never
        # looks clear. Measured live 2026-11-11: front clearance in this space
        # sat at 0.55-0.90 m, so the original 0.70/0.95 pair meant the robot
        # almost never saw "clear" and spent its time pirouetting, covering
        # 0.47 m in two minutes. 0.50/0.65 leaves 0.15 m over the guard's stop
        # and lets it commit to driving in normal indoor clutter.
        self.declare_parameter('turn_distance', 0.50)
        self.declare_parameter('clear_distance', 0.65)
        # Pulsed turning. The drivetrain cannot turn slower than ~1.09 rad/s
        # (break-away floors each wheel at 0.18 m/s), and the lidar only scans
        # at 3.68 Hz - so a continuous turn puts ~19 deg between scans, more
        # than correlative scan matching can associate, and SLAM maps smear.
        # Bursting the turn drops the AVERAGE rate without going under
        # break-away, so scans land in the gaps between bursts.
        # Set turn_pulse_period to 0.0 for a continuous turn.
        # Pulsing is now DISABLED by default (0.0 = continuous turn).
        #
        # It was introduced to lower the average turn rate for the 3.7 Hz
        # lidar, but measurement on 2026-11-13 showed it makes things worse:
        # every burst is a fresh break-away from rest, and break-away is where
        # the tyres slip. Continuous turns measured an odometry yaw gain of
        # 1.002 against the lidar; pulsed turns 1.038. Pulsed turns also
        # overshoot, because the base coasts after each command stops.
        self.declare_parameter('turn_pulse_period', 0.0)
        self.declare_parameter('turn_pulse_duty', 0.45)
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
        # Sensor floor only. This used to be 0.20 to reject the extrusion
        # astern, which blinded the FRONT sector at close range and caused a
        # collision - the self-return is now masked by angle instead.
        self.declare_parameter('min_valid_range', 0.05)
        # The extrusion is ~0.117 m dead astern; drop returns only if they are
        # both beyond this bearing from straight ahead AND this close.
        self.declare_parameter('self_mask_from_deg', 165.0)
        self.declare_parameter('self_mask_range', 0.25)

        g = lambda n: self.get_parameter(n).value
        self.cruise_speed = g('cruise_speed')
        self.turn_speed = g('turn_speed')
        self.back_speed = g('back_speed')
        self.turn_distance = g('turn_distance')
        self.clear_distance = g('clear_distance')
        self.turn_pulse_period = g('turn_pulse_period')
        self.turn_pulse_duty = g('turn_pulse_duty')
        self.min_turn_time = g('min_turn_time')
        self.max_turn_time = g('max_turn_time')
        self.back_time = g('back_time')
        self.curiosity_interval = g('curiosity_interval')
        self.curiosity_turn_time = g('curiosity_turn_time')
        self.front_half = math.radians(g('front_half_angle_deg'))
        self.side_span = math.radians(g('side_span_deg'))
        self.enabled = g('start_enabled')
        self.min_valid_range = g('min_valid_range')
        self.self_mask_from_deg = g('self_mask_from_deg')
        self.self_mask_range = g('self_mask_range')

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
            self.min_valid_range, self.self_mask_from_deg,
            self.self_mask_range)

    def _on_enable(self, msg: Bool):
        if msg.data != self.enabled:
            self.get_logger().info(f'wander {"enabled" if msg.data else "paused"}')
        self.enabled = msg.data

    def _elapsed(self):
        return (self.get_clock().now() - self.state_since).nanoseconds / 1e9

    def _goto(self, state):
        self.state = state
        self.state_since = self.get_clock().now()

    def _turn_cmd(self):
        """
        Angular velocity for this tick, pulsed.

        Returns full turn_speed during the 'on' fraction of each period and 0
        otherwise. Average rate is turn_speed * duty, but every commanded burst
        still clears break-away - which a simply-reduced turn_speed would not.
        """
        if self.turn_pulse_period <= 0.0:
            return self.turn_speed * self.turn_dir
        phase = (self._elapsed() % self.turn_pulse_period) / self.turn_pulse_period
        return (self.turn_speed * self.turn_dir) if phase < self.turn_pulse_duty else 0.0

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
                cmd.angular.z = self._turn_cmd()
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
            cmd.angular.z = self._turn_cmd()
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
