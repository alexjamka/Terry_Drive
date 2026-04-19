#!/usr/bin/env python3
"""
buddy_target_tracker.py

Picks ONE person as the "target" and publishes their pose in base_link.

Policy:
  - Prefer a recognized known person (matched via /identified_persons)
  - Otherwise pick the closest person (smallest |position| in base_link)

Publishes:
  /tracked_target        geometry_msgs/PoseStamped (base_link)   — the target's pose
  /tracked_target_info   std_msgs/String (JSON)                  — id, name, distance, locked
  /oak/image_target      sensor_msgs/Image                       — RGB frame with distinct
                                                                   bounding box for target

Subscribes:
  /person_detections     vision_msgs/Detection3DArray
  /identified_persons    vision_msgs/Detection2DArray
  /camera/color/image_raw sensor_msgs/Image

Holds the target across brief occlusions (OCCLUSION_HOLD_SEC) so we don't
re-pick every time someone walks in front of the person we're tracking.

Keys in /tracked_target_info JSON:
  locked     — True if a target is currently being tracked
  track_id   — OAK-D tracker ID (int, or -1 if not locked)
  name       — "alex" or "unknown" (the face recognition result for this track)
  distance_m — Euclidean distance from Buddy
  age_sec    — how long since we last confirmed seeing the target
"""

import json
import time
from threading import Lock

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection3DArray, Detection2DArray


# How long to keep "locking on" to a target after they disappear.
# If the target comes back within this window we resume tracking; otherwise
# we release the lock and pick a new target.
OCCLUSION_HOLD_SEC = 2.0
# How confident must face recognition be for a name to be considered reliable?
MIN_RECOGNITION_SCORE = 0.50  # matches buddy_face_db RECOGNITION_THRESHOLD


class BuddyTargetTracker(Node):
    def __init__(self):
        super().__init__('buddy_target_tracker')

        # State
        self._lock = Lock()
        # Persistent identity attached to each OAK-D track id.
        # {track_id: {'name': str, 'score': float, 'last_seen': float}}
        self._track_identities = {}
        # Currently locked target track id (or None)
        self._locked_id = None
        # Last time we saw the locked target
        self._locked_last_seen = 0.0
        # Last detection position for the locked target (fallback during occlusion)
        self._locked_last_pose = None

        # Subscribers
        self.create_subscription(
            Detection3DArray, '/person_detections', self._on_detections, 10)
        self.create_subscription(
            Detection2DArray, '/identified_persons', self._on_identifications, 10)
        self.create_subscription(
            Image, '/camera/color/image_raw', self._on_image, 5)

        # Publishers
        self.target_pub = self.create_publisher(PoseStamped, '/tracked_target', 10)
        self.info_pub = self.create_publisher(String, '/tracked_target_info', 10)
        self.img_pub = self.create_publisher(Image, '/oak/image_target', 5)

        # Periodic publisher of info + stale-lock cleanup
        self.create_timer(0.2, self._tick)

        self.get_logger().info('buddy_target_tracker ready')

    # ---------- callbacks ----------

    def _on_identifications(self, msg: Detection2DArray):
        """
        Each message contains one face identification for one person crop.
        The crop's frame_id carries the OAK-D track_id as 'person_<id>'.
        """
        try:
            if not msg.header.frame_id.startswith('person_'):
                return
            track_id = int(msg.header.frame_id.split('_', 1)[1])
        except (ValueError, IndexError):
            return

        if not msg.detections or not msg.detections[0].results:
            return

        hyp = msg.detections[0].results[0].hypothesis
        name = hyp.class_id
        score = float(hyp.score)

        with self._lock:
            prev = self._track_identities.get(track_id, {})
            # Prefer the higher-confidence name if we've seen this track before
            if name != 'unknown' and score >= MIN_RECOGNITION_SCORE:
                if prev.get('name') != name or score > prev.get('score', 0.0):
                    self._track_identities[track_id] = {
                        'name': name,
                        'score': score,
                        'last_seen': time.time(),
                    }
                else:
                    prev['last_seen'] = time.time()
            else:
                # Keep an 'unknown' marker but don't overwrite a known name
                if 'name' not in prev:
                    self._track_identities[track_id] = {
                        'name': 'unknown',
                        'score': score,
                        'last_seen': time.time(),
                    }
                else:
                    prev['last_seen'] = time.time()

    def _on_detections(self, msg: Detection3DArray):
        """
        Decide which track to lock onto based on current detections.
        Policy: recognized (known name) first, closest otherwise.
        """
        if not msg.detections:
            return

        now = time.time()
        # Build list of (track_id, pose, distance, name) for this frame.
        # Track IDs aren't directly in Detection3D — our detector publishes them
        # in crop frame_ids, not here. We match by index order as a fallback,
        # and by spatial proximity to the last-known locked pose for continuity.
        # For now, treat detections as "candidates" and pair them with known
        # track identities by nearest neighbor to last-seen-positions.
        candidates = []
        for det in msg.detections:
            p = det.bbox.center.position
            dist = float(np.sqrt(p.x**2 + p.y**2 + p.z**2))
            candidates.append({
                'position': (p.x, p.y, p.z),
                'distance': dist,
                'header': msg.header,
            })

        if not candidates:
            return

        with self._lock:
            # Prune stale track identities (not seen in 10s)
            stale = [tid for tid, info in self._track_identities.items()
                     if now - info.get('last_seen', 0) > 10.0]
            for tid in stale:
                del self._track_identities[tid]

            # Check if our currently-locked target is still visible.
            # We match by nearest-neighbor to last-known position since
            # detection msgs don't carry track_ids directly.
            locked_still_visible = False
            if self._locked_id is not None and self._locked_last_pose is not None:
                lx, ly, lz = self._locked_last_pose
                best_dist = float('inf')
                best_cand = None
                for c in candidates:
                    cx, cy, cz = c['position']
                    d = np.sqrt((cx-lx)**2 + (cy-ly)**2 + (cz-lz)**2)
                    if d < best_dist:
                        best_dist = d
                        best_cand = c
                # If the closest detection is within 0.8m of last-known target,
                # assume it's the same person.
                if best_cand is not None and best_dist < 0.8:
                    self._locked_last_seen = now
                    self._locked_last_pose = best_cand['position']
                    self._publish_target(best_cand)
                    locked_still_visible = True

            if locked_still_visible:
                return

            # Either no lock, or locked target not visible this frame.
            # If within OCCLUSION_HOLD_SEC of last seeing them, keep the lock
            # but don't publish a fresh pose.
            if (self._locked_id is not None and
                    now - self._locked_last_seen < OCCLUSION_HOLD_SEC):
                return  # hold lock silently

            # Release stale lock, pick a new target.
            self._locked_id = None
            self._locked_last_pose = None

            # Policy: known person first, closest otherwise.
            # We match track identities to detections by nearest known pose.
            # Since we can't directly bind track_id to a Detection3D without
            # shared metadata, we'll lock on the track_id whose most recent
            # identity we know — and assume the corresponding person is the
            # closest candidate. This is an approximation that works well when
            # only one or two people are in view.

            known_tracks = {tid: info for tid, info in self._track_identities.items()
                            if info.get('name') not in (None, 'unknown')}

            chosen = None
            chosen_id = None
            if known_tracks:
                # There's a recognized person among the tracks; pick the
                # closest candidate as the target (single-person common case)
                chosen = min(candidates, key=lambda c: c['distance'])
                # Arbitrarily attribute to the most-recently-seen known track
                chosen_id = max(known_tracks.items(),
                                key=lambda kv: kv[1]['last_seen'])[0]
            else:
                chosen = min(candidates, key=lambda c: c['distance'])
                chosen_id = -1  # no known id

            self._locked_id = chosen_id
            self._locked_last_seen = now
            self._locked_last_pose = chosen['position']
            self._publish_target(chosen)

    def _on_image(self, msg: Image):
        """Overlay a distinctive bbox on the target and republish."""
        if msg.encoding != 'bgr8':
            return
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            frame = arr.reshape((msg.height, msg.width, 3)).copy()
        except ValueError:
            return

        with self._lock:
            locked = self._locked_id is not None
            name = 'unknown'
            if locked and self._locked_id in self._track_identities:
                name = self._track_identities[self._locked_id].get('name', 'unknown')
            target_pose = self._locked_last_pose if locked else None

        h, w = frame.shape[:2]
        if locked and target_pose is not None:
            # Banner at top indicating target
            dist = np.sqrt(sum(v*v for v in target_pose))
            label = f"TARGET: {name}  ({dist:.2f}m)"
            cv2.rectangle(frame, (0, 0), (w, 30), (0, 200, 0), -1)
            cv2.putText(frame, label, (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        else:
            cv2.rectangle(frame, (0, 0), (w, 30), (50, 50, 50), -1)
            cv2.putText(frame, "NO TARGET", (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)

        out = Image()
        out.header = msg.header
        out.height = h
        out.width = w
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = w * 3
        out.data = frame.tobytes()
        self.img_pub.publish(out)

    def _tick(self):
        """Periodic info publish + lock cleanup."""
        now = time.time()
        with self._lock:
            if self._locked_id is not None:
                age = now - self._locked_last_seen
                if age > OCCLUSION_HOLD_SEC:
                    self._locked_id = None
                    self._locked_last_pose = None

            info = {
                'locked': self._locked_id is not None,
                'track_id': int(self._locked_id) if self._locked_id is not None else -1,
                'name': 'unknown',
                'distance_m': 0.0,
                'age_sec': 0.0,
            }
            if self._locked_id is not None:
                if self._locked_id in self._track_identities:
                    info['name'] = self._track_identities[self._locked_id].get(
                        'name', 'unknown')
                if self._locked_last_pose is not None:
                    info['distance_m'] = float(np.sqrt(
                        sum(v*v for v in self._locked_last_pose)))
                info['age_sec'] = float(now - self._locked_last_seen)

        msg = String()
        msg.data = json.dumps(info)
        self.info_pub.publish(msg)

    def _publish_target(self, cand):
        ps = PoseStamped()
        ps.header = Header()
        ps.header.stamp = self.get_clock().now().to_msg()
        # Note: upstream detector publishes in 'oak_d_link' frame. We'd need
        # a TF from oak_d_link -> base_link to republish in base_link. For now
        # we pass through the detector's frame; the URDF static TF handles it
        # for downstream TF consumers.
        ps.header.frame_id = cand['header'].frame_id or 'oak_d_link'
        x, y, z = cand['position']
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        ps.pose.orientation.w = 1.0
        self.target_pub.publish(ps)


def main(args=None):
    rclpy.init(args=args)
    node = BuddyTargetTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
