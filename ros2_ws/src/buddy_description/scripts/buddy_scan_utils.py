#!/usr/bin/env python3
"""
Shared LaserScan helpers for Buddy's reactive behaviours.

Everything here works in the ROBOT's frame, not the lidar's. Getting the
lidar's mounting yaw wrong mirrors every avoidance decision - the robot steers
away from open space and into walls - so resolve_scan_yaw() reads it out of TF
rather than hardcoding it, and callers log what they found at startup.

On Buddy the offset is 0: laser-frame 0 deg points forward. Verified against a
landmark - the 40x40 mm extrusion behind the lidar appears at 0.117 m and lands
at laser 180 deg, which is the robot's rear. The launch file previously claimed
180 deg here, which inverted avoidance; see its laser_tf comment.
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


def scan_to_robot_frame(scan, yaw_offset, min_valid_range=0.0,
                        self_mask_from_deg=165.0, self_mask_range=0.25):
    """
    Convert a LaserScan into (bearings, ranges) in the robot frame.

    Invalid readings (NaN, inf, out-of-spec) are dropped rather than clamped:
    an out-of-range return means "nothing seen", and clamping it to range_max
    would invent a phantom obstacle at the edge of the sensor. The X4 reports a
    hard 0 for "no return", which range_min filtering already removes.

    The robot's own structure (a 40x40 mm extrusion ~0.117 m dead astern) must
    be rejected, but rejecting it BY RANGE is dangerous and caused a real
    collision: a flat min_valid_range of 0.20 m meant that as the robot closed
    on a wall, the wall crossed inside the filter, every front return was
    discarded, the sector went EMPTY, and an empty sector reads as +inf - i.e.
    "clear". The guard then drove at full speed into it.

    So the self-return is masked by ANGLE instead: returns are dropped only if
    they are both beyond self_mask_from_deg of dead ahead (i.e. astern, where
    the extrusion is) AND closer than self_mask_range. A genuine obstacle
    anywhere else stays visible right down to the sensor floor.

    min_valid_range is now just the sensor floor; keep it small.

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

    bearings = wrap_angle(angles + yaw_offset)

    # Drop only the robot's own structure: close AND astern. Never drop a close
    # return that could be a real obstacle in a direction we might drive.
    if self_mask_range > 0.0:
        astern = np.abs(bearings) >= math.radians(self_mask_from_deg)
        good &= ~(astern & (ranges < self_mask_range))

    return bearings[good], ranges[good]


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
                     min_valid_range=0.0,
                     self_mask_from_deg=165.0,
                     self_mask_range=0.25):
    """
    Summarise a scan as the nearest obstacle in each sector around the robot.

    Returns a dict with keys front, left, right, rear - all metres, +inf when
    that sector saw nothing.
    """
    bearings, ranges = scan_to_robot_frame(
        scan, yaw_offset, min_valid_range, self_mask_from_deg, self_mask_range)
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
