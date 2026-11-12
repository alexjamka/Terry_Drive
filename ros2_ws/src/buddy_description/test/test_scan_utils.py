#!/usr/bin/env python3
"""Verify the scan sector maths, especially the lidar mounting yaw."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from sensor_msgs.msg import LaserScan
from buddy_scan_utils import sector_distances, wrap_angle

N = 360


def make_scan(obstacles, default=float('inf')):
    """obstacles: {laser_frame_degrees: range}"""
    s = LaserScan()
    s.angle_min = -math.pi
    s.angle_max = math.pi
    s.angle_increment = 2 * math.pi / N
    s.range_min = 0.10
    s.range_max = 10.0
    r = [default] * N
    for deg, rng in obstacles.items():
        idx = int(round((math.radians(deg) - s.angle_min) / s.angle_increment)) % N
        r[idx] = rng
    s.ranges = r
    return s


# Calibrated on hardware: laser-frame 0 deg points forward on Buddy.
YAW = 0.0
fails = []


def check(name, got, want):
    ok = (want is None and math.isinf(got)) or (
        want is not None and math.isfinite(got) and abs(got - want) < 0.05)
    print(f"  {'PASS' if ok else 'FAIL'}  {name}: got {got:.2f}" if math.isfinite(got)
          else f"  {'PASS' if ok else 'FAIL'}  {name}: got inf")
    if not ok:
        fails.append(name)


print("1. Obstacle physically in FRONT of the robot = laser-frame 0 deg.")
s = sector_distances(make_scan({0: 0.5}), YAW)
check("front sees it", s['front'], 0.5)
check("rear does not", s['rear'], None)

print("\n2. Obstacle physically BEHIND the robot = laser-frame 180 deg.")
s = sector_distances(make_scan({180: 0.4}), YAW)
check("rear sees it", s['rear'], 0.4)
check("front does not", s['front'], None)

print("\n3. Obstacle on the robot's LEFT (+90 deg robot) = laser-frame +90.")
s = sector_distances(make_scan({90: 0.6}), YAW)
check("left sees it", s['left'], 0.6)
check("right does not", s['right'], None)

print("\n4. Regression: the launch file used to claim yaw=pi. That mirrors")
print("   everything - a real obstacle ahead becomes invisible, and the")
print("   extrusion behind the lidar masquerades as a frontal blocker.")
s = sector_distances(make_scan({0: 0.5}), math.pi)
check("front blind under the old yaw", s['front'], None)
check("front obstacle wrongly reported behind", s['rear'], 0.5)

print("\n4b. min_valid_range drops the robot's own structure.")
s = sector_distances(make_scan({180: 0.117, 0: 1.2}), YAW, min_valid_range=0.20)
check("self-return filtered from rear", s['rear'], None)
check("real obstacle ahead still seen", s['front'], 1.2)

print("\n4c. COLLISION REGRESSION: a wall closer than the old min_valid_range")
print("    must still register. A flat 0.20 m range filter discarded every")
print("    front return as the robot closed in, the sector went empty, empty")
print("    reads as +inf = 'clear', and it drove into the wall at full speed.")
wall = {d: 0.15 for d in range(-20, 21)}          # wall 0.15 m dead ahead
s = sector_distances(make_scan(wall), YAW)
check("wall at 0.15 m is SEEN ahead", s['front'], 0.15)
s_old = sector_distances(make_scan(wall), YAW, min_valid_range=0.20)
check("old range filter blinded it (the bug)", s_old['front'], None)

print("\n4d. ...while the extrusion astern is still rejected by ANGLE.")
s = sector_distances(make_scan({178: 0.117, 0: 1.4}), YAW)
check("extrusion astern filtered", s['rear'], None)
check("real obstacle ahead unaffected", s['front'], 1.4)
print("\n4e. A genuine obstacle astern, beyond the mask range, is still seen.")
s = sector_distances(make_scan({178: 0.60}), YAW)
check("rear obstacle at 0.60 m seen", s['rear'], 0.60)

print("\n5. Invalid returns are dropped, not treated as obstacles.")
sc = make_scan({}, default=float('inf'))
sc.ranges = [float('nan')] * N
s = sector_distances(sc, YAW)
check("front inf on all-NaN", s['front'], None)

print("\n6. Out-of-range returns are not phantom obstacles at range_max.")
sc = make_scan({}, default=99.0)  # beyond range_max
s = sector_distances(sc, YAW)
check("front inf when all beyond max", s['front'], None)

print("\n7. Nearest wins within a sector.")
s = sector_distances(make_scan({0: 2.0, -5: 0.8, 5: 1.5}), YAW)
check("front takes the min", s['front'], 0.8)

print("\n8. wrap_angle sanity.")
for a, want in [(3 * math.pi, -math.pi), (-3 * math.pi, -math.pi), (0.5, 0.5)]:
    got = float(wrap_angle(a))
    ok = abs(got - want) < 1e-9
    print(f"  {'PASS' if ok else 'FAIL'}  wrap({a:.2f}) = {got:.4f}")
    if not ok:
        fails.append(f"wrap {a}")

print("\n" + ("ALL PASS" if not fails else f"FAILURES: {fails}"))
sys.exit(1 if fails else 0)
