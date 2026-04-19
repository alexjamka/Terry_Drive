#!/usr/bin/env python3
"""
buddy_follow_controller.py

Subscribes to /tracked_target and /tracked_target_info.
Publishes /cmd_vel to follow a locked target at STOP_DISTANCE.

Enable/disable with:
    ros2 topic pub /follow_enable std_msgs/Bool "data: true" --once
    ros2 topic pub /follow_enable std_msgs/Bool "data: false" --once

Disabled by default.
"""

import json
import math
import time
from threading import Lock

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool


STOP_DISTANCE = 1.0
STOP_BAND = 0.20
MAX_ENGAGE_DISTANCE = 4.0
KP_LINEAR = 0.5
KP_ANGULAR = 1.0
MAX_LINEAR = 0.35
MAX_ANGULAR = 1.2
TURN_THRESHOLD = math.radians(30)
TARGET_TIMEOUT_SEC = 1.5
LOOP_HZ = 20
TARGET_SMOOTH_ALPHA = 0.65


class BuddyFollowController(Node):
    def __init__(self):
        super().__init__('buddy_follow_controller')

        self._lock = Lock()
        self._last_target = None
        self._last_target_stamp = 0.0
        self._locked = False
        self._enabled = False
        self._smoothed_x = None
        self._smoothed_y = None

        self.create_subscription(PoseStamped, '/tracked_target', self._on_target, 10)
        self.create_subscription(String, '/tracked_target_info', self._on_target_info, 10)
        self.create_subscription(Bool, '/follow_enable', self._on_enable, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/follow_status', 10)

        self.create_timer(1.0 / LOOP_HZ, self._tick)

        self.get_logger().info(
            'buddy_follow_controller ready. Enable with: '
            'ros2 topic pub /follow_enable std_msgs/Bool "data: true" --once')

    def _on_target(self, msg: PoseStamped):
        with self._lock:
            self._last_target = msg
            self._last_target_stamp = time.time()

    def _on_target_info(self, msg: String):
        try:
            info = json.loads(msg.data)
            with self._lock:
                self._locked = bool(info.get('locked', False))
        except (json.JSONDecodeError, ValueError):
            pass

    def _on_enable(self, msg: Bool):
        with self._lock:
            if msg.data and not self._enabled:
                self.get_logger().info('Follow mode ENABLED')
            elif not msg.data and self._enabled:
                self.get_logger().info('Follow mode DISABLED')
            self._enabled = bool(msg.data)

    def _tick(self):
        now = time.time()
        with self._lock:
            enabled = self._enabled
            locked = self._locked
            target = self._last_target
            target_age = (now - self._last_target_stamp
                          if self._last_target_stamp else 999.0)

        stop_reason = None
        if not enabled:
            stop_reason = 'disabled'
        elif not locked:
            stop_reason = 'no_lock'
        elif target is None:
            stop_reason = 'no_target'
        elif target_age > TARGET_TIMEOUT_SEC:
            stop_reason = f'target_stale({target_age:.1f}s)'

        if stop_reason is not None:
            if stop_reason != 'disabled':
                with self._lock:
                    self._smoothed_x = None
                    self._smoothed_y = None
            self._publish_stop(stop_reason)
            return

        raw_x = target.pose.position.x
        raw_y = target.pose.position.y

        with self._lock:
            if self._smoothed_x is None:
                self._smoothed_x = raw_x
                self._smoothed_y = raw_y
            else:
                a = TARGET_SMOOTH_ALPHA
                self._smoothed_x = a * self._smoothed_x + (1 - a) * raw_x
                self._smoothed_y = a * self._smoothed_y + (1 - a) * raw_y
            x = self._smoothed_x
            y = self._smoothed_y

        distance = math.sqrt(x*x + y*y)

        if distance < 0.05:
            self._publish_stop('on_top_of_target')
            return

        angle_error = math.atan2(y, x)

        angular_cmd = KP_ANGULAR * angle_error
        angular_cmd = max(-MAX_ANGULAR, min(MAX_ANGULAR, angular_cmd))

        if abs(angle_error) > TURN_THRESHOLD:
            linear_cmd = 0.0
            mode = 'turn'
        else:
            distance_error = distance - STOP_DISTANCE
            if abs(distance_error) < STOP_BAND:
                linear_cmd = 0.0
                mode = 'holding'
            elif distance > MAX_ENGAGE_DISTANCE:
                linear_cmd = MAX_LINEAR
                mode = 'chase'
            else:
                linear_cmd = KP_LINEAR * distance_error
                linear_cmd = max(-MAX_LINEAR, min(MAX_LINEAR, linear_cmd))
                mode = 'approach' if linear_cmd > 0 else 'back_off'

        self._publish_vel(linear_cmd, angular_cmd)
        self._publish_status({
            'mode': mode,
            'distance': round(distance, 2),
            'angle_deg': round(math.degrees(angle_error), 1),
            'linear': round(linear_cmd, 3),
            'angular': round(angular_cmd, 3),
        })

    def _publish_stop(self, reason: str):
        t = Twist()
        self.cmd_pub.publish(t)
        self._publish_status({'mode': 'stopped', 'reason': reason})

    def _publish_vel(self, linear: float, angular: float):
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        self.cmd_pub.publish(t)

    def _publish_status(self, payload: dict):
        m = String()
        m.data = json.dumps(payload)
        self.status_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = BuddyFollowController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()