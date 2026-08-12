#!/usr/bin/env python3
"""
buddy_person_detector.py
- Runs OAK-D Lite spatial detection + object tracker pipeline (depthai v3)
- Publishes /person_detections (vision_msgs/Detection3DArray) with XYZ per person
- Publishes /camera/color/image_raw (sensor_msgs/Image) with bounding box overlays
- Publishes /person_crops (sensor_msgs/Image) for each tracked person, upper 55% of bbox
- Headless: all visualization via ROS topics -> Foxglove
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose, PoseWithCovariance
from std_msgs.msg import Header
import depthai as dai
import numpy as np
import cv2
import threading
import time


# Colors for different tracker IDs
COLORS = [
    (255, 100, 100), (100, 255, 100), (100, 100, 255),
    (255, 255, 100), (255, 100, 255), (100, 255, 255),
    (200, 200, 200), (255, 180, 50),
]

STEREO_SIZE = (640, 400)
PREVIEW_SIZE = (640, 400)
CONFIDENCE_THRESHOLD = 0.5
DEPTH_LOWER_MM = 100
DEPTH_UPPER_MM = 5000


class BuddyPersonDetector(Node):
    def __init__(self):
        super().__init__('buddy_person_detector')

        # Publishers
        self.det_pub = self.create_publisher(Detection3DArray, '/person_detections', 10)
        self.img_pub = self.create_publisher(Image, '/camera/color/image_raw', 5)
        self.crop_pub = self.create_publisher(Image, '/person_crops', 5)

        self.pipeline_running = False

        # Start depthai pipeline in background thread
        self._thread = threading.Thread(target=self._run_pipeline, daemon=True)
        self._thread.start()
        self.get_logger().info('Person detector node started')

    def _run_pipeline(self):
        try:
            self._build_and_run()
        except Exception as e:
            self.get_logger().error(f'Pipeline error: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def _build_and_run(self):
        model_desc = dai.NNModelDescription('yolov6-nano')
        model_desc.platform = 'RVC2'

        with dai.Pipeline() as p:
            # --- Camera nodes ---
            cam_rgb = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            mono_left = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
            mono_right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

            # --- Stereo depth ---
            stereo = p.create(dai.node.StereoDepth)
            stereo.setExtendedDisparity(True)
            mono_left.requestOutput(STEREO_SIZE).link(stereo.left)
            mono_right.requestOutput(STEREO_SIZE).link(stereo.right)

            # --- Spatial detection network ---
            spatial_nn = p.create(dai.node.SpatialDetectionNetwork).build(
                cam_rgb, stereo, model_desc
            )
            spatial_nn.setConfidenceThreshold(CONFIDENCE_THRESHOLD)
            spatial_nn.input.setBlocking(False)
            spatial_nn.setBoundingBoxScaleFactor(0.5)
            spatial_nn.setDepthLowerThreshold(DEPTH_LOWER_MM)
            spatial_nn.setDepthUpperThreshold(DEPTH_UPPER_MM)

            label_map = spatial_nn.getClasses()
            self.get_logger().info(f'Model classes: {label_map}')

            # Find person label index
            person_label_idx = None
            for i, name in enumerate(label_map):
                if name == 'person':
                    person_label_idx = i
                    break
            if person_label_idx is None:
                self.get_logger().error('No "person" class found in model!')
                return
            self.get_logger().info(f'Person label index: {person_label_idx}')

            # --- Object tracker ---
            tracker = p.create(dai.node.ObjectTracker)
            tracker.setDetectionLabelsToTrack([person_label_idx])
            tracker.setTrackerType(dai.TrackerType.SHORT_TERM_IMAGELESS)
            tracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID)

            # Link spatial NN outputs -> tracker
            spatial_nn.passthrough.link(tracker.inputTrackerFrame)
            spatial_nn.passthrough.link(tracker.inputDetectionFrame)
            spatial_nn.out.link(tracker.inputDetections)

            # Output queues
            tracklet_q = tracker.out.createOutputQueue()
            preview_q = tracker.passthroughTrackerFrame.createOutputQueue()

            # --- Run ---
            self.pipeline_running = True
            self.get_logger().info('OAK-D pipeline started')

            fps_counter = 0
            fps_start = time.monotonic()
            fps_val = 0.0

            # Throttle /person_crops to ~3 Hz per track so CPU face recognition can keep up.
            # key = track id, value = last published time
            last_crop_pub = {}
            CROP_PUB_INTERVAL = 0.33  # seconds between crops per track

            p.start()
            while p.isRunning() and rclpy.ok():
                img_frame = preview_q.get()
                track_msg = tracklet_q.get()

                if img_frame is None or track_msg is None:
                    continue

                frame = img_frame.getCvFrame()
                h, w = frame.shape[:2]

                # FPS calculation
                fps_counter += 1
                elapsed = time.monotonic() - fps_start
                if elapsed > 1.0:
                    fps_val = fps_counter / elapsed
                    fps_counter = 0
                    fps_start = time.monotonic()

                # Build Detection3DArray
                det_array = Detection3DArray()
                det_array.header = Header()
                det_array.header.stamp = self.get_clock().now().to_msg()
                det_array.header.frame_id = 'oak_d_link'

                tracklets = track_msg.tracklets
                now = time.monotonic()

                for t in tracklets:
                    # Only publish tracked / new detections
                    if t.status.name not in ('TRACKED', 'NEW'):
                        continue

                    # Draw on frame
                    roi = t.roi.denormalize(w, h)
                    x1 = int(roi.topLeft().x)
                    y1 = int(roi.topLeft().y)
                    x2 = int(roi.bottomRight().x)
                    y2 = int(roi.bottomRight().y)

                    color = COLORS[t.id % len(COLORS)]
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

                    # --- Publish crop for face recognition (throttled per track) ---
                    last_t = last_crop_pub.get(t.id, 0.0)
                    if now - last_t >= CROP_PUB_INTERVAL:
                        fx1 = max(0, x1)
                        fy1 = max(0, y1)
                        fx2 = min(w, x2)
                        fy2 = min(h, y1 + int((y2 - y1) * 0.55))
                        if fx2 - fx1 > 40 and fy2 - fy1 > 40:
                            crop = frame[fy1:fy2, fx1:fx2]
                            # Use ascontiguousarray so .tobytes() is safe after slicing
                            crop = np.ascontiguousarray(crop)
                            crop_msg = Image()
                            crop_msg.header = Header()
                            crop_msg.header.stamp = det_array.header.stamp
                            crop_msg.header.frame_id = f'person_{t.id}'
                            crop_msg.height = crop.shape[0]
                            crop_msg.width = crop.shape[1]
                            crop_msg.encoding = 'bgr8'
                            crop_msg.is_bigendian = 0
                            crop_msg.step = crop.shape[1] * 3
                            crop_msg.data = crop.tobytes()
                            self.crop_pub.publish(crop_msg)
                            last_crop_pub[t.id] = now

                    sx = int(t.spatialCoordinates.x)
                    sy = int(t.spatialCoordinates.y)
                    sz = int(t.spatialCoordinates.z)

                    info_lines = [
                        f'Person ID:{t.id}',
                        f'{t.status.name}',
                        f'X:{sx}mm Y:{sy}mm',
                        f'Z:{sz}mm',
                    ]
                    for i, line in enumerate(info_lines):
                        cv2.putText(frame, line, (x1 + 5, y1 + 15 + i * 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

                    # Build Detection3D
                    det3d = Detection3D()
                    det3d.header = det_array.header

                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = 'person'
                    hyp.hypothesis.score = float(t.srcImgDetection.confidence) if t.srcImgDetection else 0.0

                    pose = Pose()
                    # OAK coordinates: X=right, Y=down, Z=forward (mm -> meters)
                    pose.position.x = t.spatialCoordinates.z / 1000.0  # forward
                    pose.position.y = -t.spatialCoordinates.x / 1000.0  # left
                    pose.position.z = -t.spatialCoordinates.y / 1000.0  # up
                    pose.orientation.w = 1.0
                    pose_cov = PoseWithCovariance()
                    pose_cov.pose = pose
                    hyp.pose = pose_cov

                    det3d.results.append(hyp)

                    # Bounding box center in 3D (same as pose)
                    det3d.bbox.center = pose
                    # Size estimate from bounding box width at measured depth
                    if sz > 0:
                        bbox_width_m = (x2 - x1) / w * sz / 1000.0
                        bbox_height_m = (y2 - y1) / h * sz / 1000.0
                        det3d.bbox.size.x = float(sz) / 1000.0 * 0.1  # depth extent estimate
                        det3d.bbox.size.y = bbox_width_m
                        det3d.bbox.size.z = bbox_height_m

                    det_array.detections.append(det3d)

                # Clean up throttle dict of stale track ids (not in this frame)
                active_ids = {t.id for t in tracklets if t.status.name in ('TRACKED', 'NEW')}
                for stale_id in list(last_crop_pub.keys()):
                    if stale_id not in active_ids:
                        del last_crop_pub[stale_id]

                # FPS overlay
                cv2.putText(frame, f'FPS: {fps_val:.1f}  Persons: {len(det_array.detections)}',
                            (5, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                # Publish detections
                self.det_pub.publish(det_array)

                # Publish image (manual conversion to avoid cv_bridge numpy compat issue)
                img_msg = Image()
                img_msg.header = det_array.header
                img_msg.height = h
                img_msg.width = w
                img_msg.encoding = 'bgr8'
                img_msg.is_bigendian = 0
                img_msg.step = w * 3
                img_msg.data = frame.tobytes()
                self.img_pub.publish(img_msg)

            self.pipeline_running = False
            self.get_logger().info('OAK-D pipeline stopped')


def main(args=None):
    rclpy.init(args=args)
    node = BuddyPersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()