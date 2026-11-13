#!/usr/bin/env python3
"""
Keyboard teleop for mapping, built for this robot's quirks.

Why not teleop_twist_keyboard: that node publishes ONE message per keypress,
while buddy_scan_guard zeroes its output after cmd_timeout (0.5 s) of silence.
A tap therefore produces a brief twitch and holding a key relies on terminal
auto-repeat. This node instead latches the last command and republishes it at
`rate` Hz, so the robot keeps moving until you tell it to stop.

Speeds default above the drivetrain's real break-away (~0.26 m/s, measured
2026-11-12 - see buddy_scan_guard's min_linear note). Commanding less does not
produce a slower robot, it produces a juddering one.

Publishes /cmd_vel_raw, so the guard still refuses to drive into a wall. The
guard never blocks rotation, so you can always spin away from an obstacle.

    w / s   forward / reverse
    a / d   rotate left / right
    space   stop
    + / -   faster / slower
    q       quit (stops the robot)
"""
import math
import os
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

HELP = __doc__.split('    w / s')[1]


class Teleop(Node):

    def __init__(self):
        super().__init__('buddy_teleop')
        # Above break-away. 0.28 m/s is PWM 233; 1.6 rad/s puts each wheel at
        # 0.264 m/s, just clear of the floor.
        self.declare_parameter('linear_speed', 0.28)
        self.declare_parameter('angular_speed', 1.6)
        self.declare_parameter('rate', 15.0)
        self.lin = self.get_parameter('linear_speed').value
        self.ang = self.get_parameter('angular_speed').value
        self.cmd = Twist()
        self.pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.guard = '(waiting for guard)'
        self.create_subscription(String, '/scan_guard_status', self._on_guard, 10)
        self.create_timer(1.0 / max(self.get_parameter('rate').value, 1.0),
                          self._tick)

    def _on_guard(self, msg):
        self.guard = msg.data[:110]

    def _tick(self):
        self.pub.publish(self.cmd)

    def key(self, k):
        if k == 'w':
            self.cmd = Twist(); self.cmd.linear.x = self.lin
        elif k == 's':
            self.cmd = Twist(); self.cmd.linear.x = -self.lin
        elif k == 'a':
            self.cmd = Twist(); self.cmd.angular.z = self.ang
        elif k == 'd':
            self.cmd = Twist(); self.cmd.angular.z = -self.ang
        elif k == ' ':
            self.cmd = Twist()
        elif k in '+=':
            self.lin = min(0.30, self.lin + 0.02)
        elif k in '-_':
            self.lin = max(0.26, self.lin - 0.02)
        return k != 'q'


def main():
    rclpy.init()
    node = Teleop()
    print(HELP)
    print(f'linear={node.lin:.2f} m/s  angular={node.ang:.2f} rad/s'
          '   (publishing /cmd_vel_raw)\n')
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        running = True
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if select.select([sys.stdin], [], [], 0.0)[0]:
                running = node.key(sys.stdin.read(1))
            sys.stdout.write(
                f'\r lin={node.cmd.linear.x:+.2f} ang={node.cmd.angular.z:+.2f}'
                f' | speed={node.lin:.2f} | {node.guard}\033[K')
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        # Leaving the base rolling because the operator pressed ctrl-C is not
        # acceptable; publish a few explicit stops before going away.
        for _ in range(5):
            node.pub.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.02)
        print('\nstopped')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
