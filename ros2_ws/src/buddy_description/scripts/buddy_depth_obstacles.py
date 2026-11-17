#!/usr/bin/env python3
"""
Turn the OAK-D depth image into a 2D obstacle scan, covering the heights the
lidar cannot see.

WHY THIS EXISTS
---------------
The lidar sees a single horizontal slice at 0.406 m. The robot is ~0.9 m tall,
so anything whose surface sits above that slice is invisible to it - and on
2026-11-13 the robot drove into a desk whose top was above the scan plane while
the guard reported the way ahead clear. Obstacles BELOW the slice (a low
furniture base, a step) are equally invisible.

WHAT IT DOES
------------
Projects each depth pixel into the robot frame, keeps the ones inside a height
band that would actually collide with the robot, and reduces them to a nearest
obstacle per bearing - published as a LaserScan on /scan_camera.

The scan is published in the BASE frame, not the camera frame: each range is the
horizontal distance from the robot's own origin, so buddy_scan_guard can treat
it with the same sector maths as the lidar, with no mounting yaw to apply.

LIMITS, which are real and should not be forgotten
--------------------------------------------------
  * Horizontal field of view is ~70 deg, not 360. This covers what is AHEAD.
    It is a complement to the lidar, never a replacement.
  * Measured minimum depth is ~0.35 m, and in practice the near field is sparse.
    An obstacle already touching the robot will NOT appear. stop_distance must
    stay inside what the lidar can see.
  * Points below min_height are discarded as floor. Anything shorter than that
    is invisible to this node as well - see the parameter comment.
  * Stereo depth is noisy on blank surfaces; min_points_per_bin exists so a
    single speckle cannot fabricate an obstacle.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

try:
    from foxglove_msgs.msg import (Color, ImageAnnotations, Point2,
                                   PointsAnnotation, TextAnnotation)
    HAVE_FOXGLOVE = True
except ImportError:          # annotations are a debugging aid, not a dependency
    HAVE_FOXGLOVE = False


class DepthObstacles(Node):

    def __init__(self):
        super().__init__('buddy_depth_obstacles')

        self.declare_parameter('depth_topic', '/oak/stereo/image_raw')
        self.declare_parameter('info_topic', '/oak/rgb/camera_info')
        self.declare_parameter('scan_topic', '/scan_camera')
        self.declare_parameter('base_frame', 'BuddyBaseFootprint_link')
        # Height band that would actually hit the robot. The floor is at 0 and
        # the robot's tallest point is ~0.9 m.
        #
        # min_height is a floor-rejection threshold, NOT a statement that
        # shorter things are harmless: stereo noise and any small error in the
        # camera pitch smear the floor upward, and a floor misread as an
        # obstacle would stop the robot dead in an empty room. 0.10 m is the
        # compromise. Obstacles under ~0.10 m are invisible to BOTH sensors.
        self.declare_parameter('min_height', 0.10)
        self.declare_parameter('max_height', 0.95)
        # Near-field depth from this sensor is unreliable, and a spurious close
        # return would trip the guard's stop line and freeze the robot in an
        # empty room. Measured 2026-11-13 with min_depth at 0.30: 13 of 67
        # scans reported something under 0.40 m while the lidar saw 0.66 m,
        # and the bearing of the nearest return wandered across the whole field
        # of view - i.e. speckle, not structure. 0.45 m sits above that noise.
        # Closer than this is the LIDAR's job; it can see to 0.12 m.
        self.declare_parameter('min_depth', 0.45)
        self.declare_parameter('max_depth', 6.0)
        # Decimation. 1280x720 is 921k pixels; every 8th in each axis leaves
        # ~14k, which is ample for a 2D obstacle summary and keeps the Orin
        # Nano cool.
        self.declare_parameter('step', 8)
        self.declare_parameter('bins', 61)
        self.declare_parameter('fov_deg', 72.0)
        # A lone speckle must not become a wall.
        self.declare_parameter('min_points_per_bin', 3)
        # Temporal persistence. A real obstacle appears in consecutive frames;
        # stereo speckle does not. A bin is only reported if it was occupied in
        # at least persist_min of the last persist_frames scans, and its range
        # is the median of those, which also rejects a single short outlier.
        # Cost is latency: at 5 Hz, 3 frames is ~0.6 s, or ~17 cm at cruise.
        # That is affordable because the lidar, not the camera, guards the
        # close stop line.
        self.declare_parameter('persist_frames', 3)
        self.declare_parameter('persist_min', 2)
        self.declare_parameter('rate', 5.0)
        # Live overlay for Foxglove's Image panel: the pixels this node counted
        # as obstacles, drawn on the camera view and coloured by distance, so
        # you can see WHAT it is stopping for rather than inferring it from a
        # separate 3D panel.
        self.declare_parameter('publish_annotations', True)
        self.declare_parameter('annotation_topic', '/oak/obstacle_annotations')
        # Cap the overlay size; at 5 Hz a few hundred points is plenty and
        # keeps the websocket cheap over wifi.
        self.declare_parameter('max_annotation_points', 500)

        g = lambda n: self.get_parameter(n).value
        self.base_frame = g('base_frame')
        self.min_h = g('min_height')
        self.max_h = g('max_height')
        self.min_d = g('min_depth')
        self.max_d = g('max_depth')
        self.step = int(g('step'))
        self.bins = int(g('bins'))
        self.fov = math.radians(g('fov_deg'))
        self.min_pts = int(g('min_points_per_bin'))
        self.persist_frames = int(g('persist_frames'))
        self.persist_min = int(g('persist_min'))
        self._history = []
        self.annotate = bool(g('publish_annotations')) and HAVE_FOXGLOVE
        self.max_ann = int(g('max_annotation_points'))
        if g('publish_annotations') and not HAVE_FOXGLOVE:
            self.get_logger().warn(
                'publish_annotations requested but foxglove_msgs is not '
                'available; continuing without the overlay')

        self.info = None
        self.depth = None
        self._warned_encoding = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, g('info_topic'),
                                 self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, g('depth_topic'),
                                 self._on_depth, qos_profile_sensor_data)
        self.scan_pub = self.create_publisher(LaserScan, g('scan_topic'), 10)
        self.ann_pub = (self.create_publisher(
            ImageAnnotations, g('annotation_topic'), 10) if self.annotate else None)
        self.status_pub = self.create_publisher(String, '/depth_obstacle_status', 10)

        self.create_timer(1.0/max(g('rate'), 1.0), self._tick)
        self.get_logger().info(
            f'depth obstacles up: band {self.min_h:.2f}-{self.max_h:.2f} m, '
            f'depth {self.min_d:.2f}-{self.max_d:.1f} m -> {g("scan_topic")}')

    def _on_info(self, msg):
        self.info = msg

    def _on_depth(self, msg):
        self.depth = msg

    def _decode(self, msg):
        """Depth image -> metres. Handles the two encodings depthai emits."""
        if msg.encoding == '16UC1':
            a = np.frombuffer(msg.data, dtype=np.uint16)
            a = a.reshape(msg.height, msg.width).astype(np.float32) / 1000.0
        elif msg.encoding == '32FC1':
            a = np.frombuffer(msg.data, dtype=np.float32)
            a = a.reshape(msg.height, msg.width).astype(np.float32)
        else:
            if not self._warned_encoding:
                self.get_logger().error(
                    f'unsupported depth encoding {msg.encoding!r}; no obstacles '
                    f'will be published')
                self._warned_encoding = True
            return None
        return a

    def _transform(self):
        """Rotation+translation base <- camera optical frame, or None."""
        if self.depth is None:
            return None
        try:
            tr = self.tf_buffer.lookup_transform(
                self.base_frame, self.depth.header.frame_id, rclpy.time.Time())
        except Exception:
            return None
        t = tr.transform.translation
        q = tr.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
            [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]], dtype=np.float32)
        return R, np.array([[t.x], [t.y], [t.z]], dtype=np.float32)

    def _tick(self):
        msg, info = self.depth, self.info
        if msg is None or info is None:
            self._status('waiting_for_camera', 0, 0)
            return
        tf = self._transform()
        if tf is None:
            self._status('waiting_for_tf', 0, 0)
            return
        R, T = tf

        d = self._decode(msg)
        if d is None:
            self._status('bad_encoding', 0, 0)
            return

        s = self.step
        d = d[::s, ::s]
        K = np.array(info.k, dtype=np.float32).reshape(3, 3)
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        if fx <= 0 or fy <= 0:
            self._status('bad_camera_info', 0, 0)
            return

        h, w = d.shape
        ys, xs = np.mgrid[0:h, 0:w].astype(np.float32)
        # mgrid is in decimated pixels; scale back to full-resolution indices
        # or the intrinsics do not apply.
        xs = xs*s
        ys = ys*s

        ok = (d > self.min_d) & (d < self.max_d) & np.isfinite(d)
        n_valid = int(ok.sum())
        if n_valid < 50:
            self._publish_empty(msg)
            self._status('no_depth', n_valid, 0)
            return

        zc = d[ok]
        xc = (xs[ok]-cx)*zc/fx
        yc = (ys[ok]-cy)*zc/fy
        pts = R @ np.vstack([xc, yc, zc]) + T
        bx, by, bz = pts[0], pts[1], pts[2]

        px_all, py_all = xs[ok], ys[ok]

        band = (bz >= self.min_h) & (bz <= self.max_h)
        n_band = int(band.sum())
        if n_band == 0:
            self._publish_empty(msg)
            self._status('clear', n_valid, 0)
            if self.annotate:
                self._publish_annotations(msg, None, None, None)
            return
        bx, by = bx[band], by[band]
        px_band, py_band = px_all[band], py_all[band]

        rng = np.hypot(bx, by)
        brg = np.arctan2(by, bx)

        half = self.fov/2.0
        inc = self.fov/(self.bins-1)
        idx = np.round((brg+half)/inc).astype(int)
        keep = (idx >= 0) & (idx < self.bins) & (rng > 0.02)
        idx, rng = idx[keep], rng[keep]

        ranges = np.full(self.bins, float('inf'), dtype=np.float32)
        if idx.size:
            counts = np.bincount(idx, minlength=self.bins)
            # Nearest return per bin, but only where enough points agree.
            order = np.argsort(rng)
            np.minimum.at(ranges, idx[order], rng[order])
            ranges[counts < self.min_pts] = float('inf')

        # --- temporal persistence (see persist_frames) ---------------------
        self._history.append(ranges.copy())
        if len(self._history) > self.persist_frames:
            self._history.pop(0)
        stack = np.vstack(self._history)               # frames x bins
        finite_mask = np.isfinite(stack)
        seen = finite_mask.sum(axis=0)
        ranges = np.full(self.bins, np.inf, dtype=np.float32)
        # Median only over bins that actually have samples. Calling nanmedian
        # across all bins warns "All-NaN slice encountered" on every empty one,
        # which at 5 Hz floods the log.
        enough = np.where(seen >= self.persist_min)[0]
        for b in enough:
            col = stack[:, b]
            ranges[b] = np.median(col[np.isfinite(col)])

        out = LaserScan()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.base_frame
        out.angle_min = -half
        out.angle_max = half
        out.angle_increment = inc
        out.range_min = float(self.min_d)
        out.range_max = float(self.max_d)
        out.ranges = [float(r) if math.isfinite(r) else 0.0 for r in ranges]
        self.scan_pub.publish(out)

        if self.annotate:
            self._publish_annotations(msg, px_band, py_band,
                                      np.hypot(bx, by))

        finite = ranges[np.isfinite(ranges)]
        nearest = float(finite.min()) if finite.size else float('inf')
        self._status('ok', n_valid, n_band, nearest, int(np.isfinite(ranges).sum()),
                     float(bz[band].min()) if n_band else None,
                     float(bz[band].max()) if n_band else None)

    def _publish_annotations(self, msg, px, py, rng):
        """
        Draw the obstacle pixels onto the camera image for Foxglove.

        Coloured near (red) to far (green) over the sensor's working range, with
        a box and a label on the nearest cluster. Pixel coordinates come from
        the depth image, which is registered to the colour stream, so they line
        up with /oak/rgb/image_rect.
        """
        ann = ImageAnnotations()
        stamp = msg.header.stamp
        if px is not None and px.size:
            n = px.size
            stride = max(1, n // self.max_ann)
            sx, sy, sr = px[::stride], py[::stride], rng[::stride]

            pa = PointsAnnotation()
            pa.timestamp = stamp
            pa.type = PointsAnnotation.POINTS
            pa.thickness = 3.0
            span = max(self.max_d - self.min_d, 1e-3)
            for x, y, r in zip(sx, sy, sr):
                pa.points.append(Point2(x=float(x), y=float(y)))
                f = min(max((r - self.min_d)/span, 0.0), 1.0)
                pa.outline_colors.append(
                    Color(r=float(1.0-f), g=float(f), b=0.2, a=0.9))
            pa.outline_color = Color(r=1.0, g=0.0, b=0.0, a=0.9)
            pa.fill_color = Color(r=0.0, g=0.0, b=0.0, a=0.0)
            ann.points.append(pa)

            # Box the nearest cluster: everything within 0.15 m of the closest
            # return, which is what the guard will actually stop for.
            near = rng.min()
            sel = rng <= near + 0.15
            if sel.sum() >= self.min_pts:
                x0, x1 = float(px[sel].min()), float(px[sel].max())
                y0, y1 = float(py[sel].min()), float(py[sel].max())
                box = PointsAnnotation()
                box.timestamp = stamp
                box.type = PointsAnnotation.LINE_LOOP
                box.thickness = 2.0
                for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
                    box.points.append(Point2(x=x, y=y))
                box.outline_color = Color(r=1.0, g=0.25, b=0.0, a=1.0)
                box.fill_color = Color(r=0.0, g=0.0, b=0.0, a=0.0)
                ann.points.append(box)

                t = TextAnnotation()
                t.timestamp = stamp
                t.position = Point2(x=x0, y=max(y0-8.0, 12.0))
                t.text = f'{near:.2f} m'
                t.font_size = 20.0
                t.text_color = Color(r=1.0, g=1.0, b=1.0, a=1.0)
                t.background_color = Color(r=0.8, g=0.2, b=0.0, a=0.7)
                ann.texts.append(t)
        self.ann_pub.publish(ann)

    def _publish_empty(self, msg):
        out = LaserScan()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.base_frame
        half = self.fov/2.0
        out.angle_min, out.angle_max = -half, half
        out.angle_increment = self.fov/(self.bins-1)
        out.range_min, out.range_max = float(self.min_d), float(self.max_d)
        out.ranges = [0.0]*self.bins
        self.scan_pub.publish(out)

    def _status(self, reason, n_valid, n_band, nearest=None, occupied=None,
                zmin=None, zmax=None):
        import json
        self.status_pub.publish(String(data=json.dumps({
            'reason': reason,
            'valid_px': n_valid,
            'in_band_px': n_band,
            'occupied_bins': occupied,
            'nearest_m': (None if nearest is None or math.isinf(nearest)
                          else round(nearest, 2)),
            'band_z': (None if zmin is None else [round(zmin, 2), round(zmax, 2)]),
        })))


def main(args=None):
    rclpy.init(args=args)
    node = DepthObstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
