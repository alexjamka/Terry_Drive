#!/usr/bin/env python3
"""
Shared LaserScan helpers for Buddy's reactive behaviours.

Everything here works in the ROBOT's frame, not the lidar's. That distinction
matters: Buddy's X4 is mounted rotated 180 deg (see the laser_tf static
transform in buddy_hardware.launch.py), so scan angle 0 points out the BACK of
the robot. Treating scan angle 0 as "forward" would make the robot steer away
from open space and into walls.

Rather than hardcode the flip, resolve_scan_yaw() reads it out of TF, so it
stays correct if the lidar is ever remounted.
"""

import math

import numpy as np

# Sector edges in the robot frame, radians. 0 = straight ahead, +ve = left.
DEFAULT_FRONT_HALF_ANGLE = math.radians(35.0)
DEFAULT_SIDE_SPAN = math.radians(55.0)


def wrap_angle(a):
    """
    Wrap angle(s) to [-pi, pi). Works on scalars and arrays.

    Note the half-open interval: exactly-backwards comes out as -pi, never +pi.
    The rear sector below straddles the seam and tests both signs, so this
    doesn't create a blind spot directly behind the robot.
    """
    return (np.asarray(a) + math.pi) % (2.0 * math.pi) - math.pi


def scan_to_robot_frame(scan, yaw_offset, min_valid_range=0.0):
    """
    Convert a LaserScan into (bearings, ranges) in the robot frame.

    Invalid readings (NaN, inf, out-of-spec) are dropped rather than clamped:
    an out-of-range return means "nothing seen", and clamping it to range_max
    would invent a phantom obstacle at the edge of the sensor. The X4 reports a
    hard 0 for "no return", which range_min filtering already removes.

    min_valid_range additionally rejects returns closer than a given distance.
    Buddy's lidar sees a few rays of the robot's own structure at ~0.12 m, and
    a permanent self-return inside the stop distance pins the base in place
    forever. Keep this comfortably below stop_distance so a real obstacle is
    still caught on approach.

    Returns two float arrays of equal length, possibly empty.
    """
    n = len(scan.ranges)
    if n == 0:
        return np.empty(0), np.empty(0)

    ranges = np.asarray(scan.ranges, dtype=np.float64)
    angles = scan.angle_min + np.arange(n, dtype=np.float64) * scan.angle_increment

    good = np.isfinite(ranges) & (ranges > max(scan.range_min, min_valid_range, 1e-3))
    if scan.range_max > 0:
        good &= ranges < scan.range_max

    return wrap_angle(angles[good] + yaw_offset), ranges[good]


def min_range_in_sector(bearings, ranges, lo, hi):
    """
    Closest return with bearing in [lo, hi], or +inf if the sector is empty.

    Handles sectors that straddle the +/-pi seam.
    """
    if bearings.size == 0:
        return math.inf

    lo = float(wrap_angle(lo))
    hi = float(wrap_angle(hi))
    if lo <= hi:
        mask = (bearings >= lo) & (bearings <= hi)
    else:  # wrapped across the back of the robot
        mask = (bearings >= lo) | (bearings <= hi)

    return float(ranges[mask].min()) if np.any(mask) else math.inf


def sector_distances(scan, yaw_offset,
                     front_half=DEFAULT_FRONT_HALF_ANGLE,
                     side_span=DEFAULT_SIDE_SPAN,
                     min_valid_range=0.0):
    """
    Summarise a scan as the nearest obstacle in each sector around the robot.

    Returns a dict with keys front, left, right, rear - all metres, +inf when
    that sector saw nothing.
    """
    bearings, ranges = scan_to_robot_frame(scan, yaw_offset, min_valid_range)
    return {
        'front': min_range_in_sector(bearings, ranges, -front_half, front_half),
        'left': min_range_in_sector(bearings, ranges, front_half, front_half + side_span),
        'right': min_range_in_sector(bearings, ranges, -(front_half + side_span), -front_half),
        'rear': min_range_in_sector(bearings, ranges,
                                    math.pi - side_span, -(math.pi - side_span)),
    }


def resolve_scan_yaw(node, base_frame, scan_frame, timeout_sec=5.0):
    """
    Look up the lidar's mounting yaw in the robot frame, via TF.

    Returns the yaw in radians, or None if TF didn't come up in time - callers
    should fall back to their configured default and say so loudly.
    """
    import rclpy
    from rclpy.duration import Duration
    from rclpy.time import Time

    deadline = node.get_clock().now() + Duration(seconds=timeout_sec)
    while rclpy.ok() and node.get_clock().now() < deadline:
        try:
            tf = node.tf_buffer.lookup_transform(
                base_frame, scan_frame, Time(), timeout=Duration(seconds=0.5))
        except Exception:
            rclpy.spin_once(node, timeout_sec=0.1)
            continue

        q = tf.transform.rotation
        # yaw from quaternion; roll/pitch are irrelevant for a planar scan
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return None
