#!/usr/bin/env python3
"""
buddy_face_recognizer.py

Subscribes to /person_crops (cropped upper-body images from buddy_person_detector).
Runs InsightFace on Jetson GPU, matches faces against ~/buddy_ws/face_db/.
Publishes /identified_persons and /buddy_say (greetings).
"""

import time
import threading
from collections import defaultdict, deque

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Header
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from insightface.app import FaceAnalysis

from buddy_face_db import load_db, match, RECOGNITION_THRESHOLD


CONFIRMATION_FRAMES = 3
GREET_COOLDOWN_SEC = 30 * 60
UNKNOWN_PROMPT_FRAMES = 6
UNKNOWN_PROMPT_COOLDOWN_SEC = 60


class BuddyFaceRecognizer(Node):
    def __init__(self):
        super().__init__('buddy_face_recognizer')

        self.db = load_db()
        if self.db:
            self.get_logger().info(
                f'Loaded face_db: {[(k, v.shape[0]) for k, v in self.db.items()]}'
            )
        else:
            self.get_logger().warn(
                'face_db is empty. Enroll someone with buddy_enroll_face.py first.'
            )

        self.get_logger().info('Loading InsightFace (buffalo_s) on GPU...')
        self.app = FaceAnalysis(
            name='buffalo_s',
            providers=['CPUExecutionProvider'],
        )
        self.app.prepare(ctx_id=0, det_size=(320, 320))
        self.get_logger().info('InsightFace ready')

        self._track_history = defaultdict(lambda: deque(maxlen=CONFIRMATION_FRAMES))
        self._last_greeted = {}
        self._unknown_state = defaultdict(lambda: {'count': 0, 'last_prompt': 0.0})

        self.id_pub = self.create_publisher(Detection2DArray, '/identified_persons', 10)
        self.say_pub = self.create_publisher(String, '/buddy_say', 10)

        self.sub = self.create_subscription(
            Image, '/person_crops', self._on_crop, 5,
        )

        self._lock = threading.Lock()
        self.get_logger().info('buddy_face_recognizer ready')

    def _image_msg_to_np(self, msg: Image):
        if msg.encoding != 'bgr8':
            self.get_logger().warn(f'Unexpected encoding: {msg.encoding}')
            return None
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        return arr.reshape((msg.height, msg.width, 3))

    def _on_crop(self, msg: Image):
        track_id = -1
        try:
            if msg.header.frame_id.startswith('person_'):
                track_id = int(msg.header.frame_id.split('_', 1)[1])
        except ValueError:
            pass

        frame = self._image_msg_to_np(msg)
        if frame is None or frame.size == 0:
            return

        faces = self.app.get(frame)
        if not faces:
            return

        face = max(faces, key=lambda f: (f.bbox[2] - f.bbox[0]) * (f.bbox[3] - f.bbox[1]))
        name, sim = match(face.normed_embedding, self.db)

        with self._lock:
            confirmed = self._update_history_and_confirm(track_id, name)
            self._handle_greeting(confirmed, track_id, sim)

        self._publish_identification(msg, face, name, sim)

    def _update_history_and_confirm(self, track_id, name):
        hist = self._track_history[track_id]
        hist.append(name)
        if len(hist) < CONFIRMATION_FRAMES:
            return None
        if all(n == hist[0] for n in hist):
            return hist[0]
        return None

    def _handle_greeting(self, confirmed, track_id, sim):
        now = time.time()
        if confirmed is None:
            return

        if confirmed == 'unknown':
            st = self._unknown_state[track_id]
            st['count'] += 1
            if (st['count'] >= UNKNOWN_PROMPT_FRAMES and
                    now - st['last_prompt'] > UNKNOWN_PROMPT_COOLDOWN_SEC):
                self._say("Hello, who are you?")
                st['last_prompt'] = now
                st['count'] = 0
            return

        self._unknown_state[track_id]['count'] = 0
        last = self._last_greeted.get(confirmed, 0.0)
        if now - last > GREET_COOLDOWN_SEC:
            hour = time.localtime().tm_hour
            if hour < 12:
                greeting = f"Good morning, {confirmed.capitalize()}"
            elif hour < 18:
                greeting = f"Good afternoon, {confirmed.capitalize()}"
            else:
                greeting = f"Good evening, {confirmed.capitalize()}"
            self._say(greeting)
            self._last_greeted[confirmed] = now

    def _say(self, text):
        self.get_logger().info(f'[say] {text}')
        msg = String()
        msg.data = text
        self.say_pub.publish(msg)

    def _publish_identification(self, crop_msg, face, name, sim):
        out = Detection2DArray()
        out.header = Header()
        out.header.stamp = crop_msg.header.stamp
        out.header.frame_id = crop_msg.header.frame_id

        det = Detection2D()
        det.header = out.header

        bbox = face.bbox
        det.bbox.center.position.x = float((bbox[0] + bbox[2]) / 2)
        det.bbox.center.position.y = float((bbox[1] + bbox[3]) / 2)
        det.bbox.size_x = float(bbox[2] - bbox[0])
        det.bbox.size_y = float(bbox[3] - bbox[1])

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = name
        hyp.hypothesis.score = float(sim)
        det.results.append(hyp)

        out.detections.append(det)
        self.id_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = BuddyFaceRecognizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
