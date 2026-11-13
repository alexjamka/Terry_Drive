#!/usr/bin/env python3
"""
Perimeter wall-follower, built for this drivetrain's constraints.

Random wandering has never closed a loop on this robot. Tracing the perimeter
and returning past the start is what loop closure needs, so this follows the
RIGHT-hand wall at a target standoff.

Two hardware constraints shape it:
  * Gentle arcs are impossible - MIN_PWM 150 floors the inner wheel, so an arc
    just stalls one side. Steering is therefore STRAIGHT / STOP / TURN-IN-PLACE.
  * Repeated break-away from rest is where the tyres slip (measured 2026-11-13:
    pulsed turns slipped MORE than continuous ones), so corrections are as few
    and as long as possible rather than constant small nudges.

Publishes /cmd_vel_raw, so buddy_scan_guard still has the final say.

RESULT OF THE FIRST RUN (2026-11-13), kept as a counter-example:

    TARGET 0.65, BAND 0.18, 0.9 s straights  ->  167 turn events in 270 s
    perimeter tracked well, odometry loop CLOSED (35.06 m path, back to
    0.57 m from start) - but the map was the WORST recorded,
    maps/wallfollow_20261113 at 29.54% occupied, walls smeared solid.

Each turn is a break-away from rest, and break-away is where this drivetrain
slips (continuous turns measured gain 1.002; pulsed turns 1.038). A controller
built on constant small corrections maximises exactly the wrong thing.

The constants below are the RETUNED version - wide deadband, long straight
legs, far fewer turns. It was interrupted before it ever ran, so it is
UNTESTED. Run it on a clear floor, with the power leads watched for wind-up
and interior doors shut; both bit on the first run.

NOTE: the robot is taller than its lidar. The 0.406 m scan plane does not see a
desk surface above it, and the first run drove into one. Until the OAK-D covers
the other height bands, supervise this.
"""
import json, math, sys, time
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 240.0
CRUISE = 0.28           # above the 0.26 m/s break-away floor
TURN = 1.6              # above the 1.57 rad/s minimum non-stalling turn
TARGET = 0.75           # desired distance to the right-hand wall
# WIDE deadband, deliberately. The first run (2026-11-13) used 0.18 and made
# 167 turn events in 270 s, producing the worst map yet (29.5% occupied, walls
# smeared solid). Every turn is a break-away from rest, and break-away is where
# this drivetrain slips. Correcting rarely beats correcting precisely.
BAND = 0.45
FRONT_STOP = 0.55       # turn away from a wall ahead
LOST_WALL = 1.6         # right side this far = wall lost, curve back toward it


class Follow(Node):
    def __init__(self):
        super().__init__('wall_follow')
        self.scan = None
        self.odom = None
        self.create_subscription(LaserScan, '/scan', self._s, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._o, 20)
        self.pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)

    def _s(self, m): self.scan = m
    def _o(self, m): self.odom = m

    def sector(self, centre_deg, half_deg):
        """Nearest return in a bearing window, and how many points backed it."""
        m = self.scan
        if m is None:
            return math.inf, 0
        r = np.array(m.ranges, dtype=float)
        a = m.angle_min + np.arange(len(r))*m.angle_increment
        a = np.degrees(np.arctan2(np.sin(a), np.cos(a)))
        d = np.abs(((a - centre_deg + 180) % 360) - 180)
        sel = (d < half_deg) & (r > 0.05) & (r < m.range_max) & np.isfinite(r)
        if sel.sum() < 3:
            return math.inf, int(sel.sum())
        return float(np.percentile(r[sel], 10)), int(sel.sum())

    def pose(self):
        if self.odom is None:
            return (0.0, 0.0)
        p = self.odom.pose.pose.position
        return (p.x, p.y)

    def drive(self, lin, ang, secs):
        t = Twist(); t.linear.x = lin; t.angular.z = ang
        e = time.time()+secs
        while time.time() < e:
            self.pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.02)

    def stop(self, secs=0.35):
        self.pub.publish(Twist())
        e = time.time()+secs
        while time.time() < e:
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)


def main():
    rclpy.init()
    n = Follow()
    while n.scan is None or n.odom is None:
        rclpy.spin_once(n, timeout_sec=0.1)

    t0 = time.time()
    start = n.pose()
    path = 0.0
    last = start
    actions = {}
    print(f'{"t":>6} {"front":>6} {"right":>6} {"action":>14} {"path":>6} {"from start":>10}')
    while time.time()-t0 < DURATION:
        front, nf = n.sector(0.0, 25.0)
        right, nr = n.sector(-90.0, 25.0)
        rfront, _ = n.sector(-45.0, 20.0)

        if front < FRONT_STOP:
            act = 'corner:turn L'
            n.stop(); n.drive(0.0, TURN, 0.55); n.stop()
        elif rfront < 0.38:
            act = 'nudge L'
            n.stop(); n.drive(0.0, TURN, 0.22); n.stop()
        elif right > LOST_WALL:
            act = 'wall lost:R'
            n.drive(CRUISE, 0.0, 0.7)
            n.stop(); n.drive(0.0, -TURN, 0.30); n.stop()
        elif right > TARGET + BAND:
            act = 'drift in:R'
            n.drive(CRUISE, 0.0, 1.6)
            n.stop(); n.drive(0.0, -TURN, 0.20); n.stop()
        elif right < TARGET - BAND:
            act = 'drift out:L'
            n.drive(CRUISE, 0.0, 1.6)
            n.stop(); n.drive(0.0, TURN, 0.20); n.stop()
        else:
            act = 'straight'
            n.drive(CRUISE, 0.0, 2.2)

        actions[act] = actions.get(act, 0)+1
        p = n.pose()
        path += math.hypot(p[0]-last[0], p[1]-last[1]); last = p
        d0 = math.hypot(p[0]-start[0], p[1]-start[1])
        print(f'{time.time()-t0:6.0f} {front:6.2f} {right:6.2f} {act:>14} '
              f'{path:6.2f} {d0:10.2f}', flush=True)

    n.stop(1.0)
    p = n.pose()
    print(f'\npath={path:.2f} m  net from start={math.hypot(p[0]-start[0],p[1]-start[1]):.2f} m')
    print('actions:', actions)
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
