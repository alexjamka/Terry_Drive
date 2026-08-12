#!/usr/bin/env python3
"""
Person-Following Robot - Jetson Orin Nano Vision Script
=======================================================
Runs YOLOv8n TensorRT on the GPU, detects objects, and sends
detection data as JSON over UDP to the Raspberry Pi.

Sends in the same format the Pi's follow.py expects:
  [{"class": "person", "x": 150, "y": 80, "w": 200, "h": 350, "conf": 0.92}, ...]

Usage:
    python3 jetson_vision.py

Prerequisites:
    - yolov8n.engine in the same directory (already built)
    - USB webcam at /dev/video0
    - Pi running follow.py on 10.0.0.1:5005
"""

import cv2
import time
import socket
import json
import numpy as np
from ultralytics import YOLO
import torch

# ─────────────────────────────────────────────────────────────
#  CONFIG
# ─────────────────────────────────────────────────────────────

# Network - Pi's static IP on the ethernet link
PI_IP = "10.0.0.1"
PI_PORT = 5005

# Camera
CAMERA_INDEX = 1
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Model
ENGINE_PATH = "yolov8n.engine"

# Only send detections above this confidence
CONFIDENCE_THRESHOLD = 0.4

# COCO class names (only the ones we care about)
COCO_NAMES = {0: "person"}
# Add more if you want the Pi to react to other objects:
# 15: "cat", 16: "dog", etc.

# Show preview window (set False when running headless on the robot)
SHOW_PREVIEW = False


# ─────────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────────

def main():
    # Disable cudnn to avoid convolution engine issues on Orin
    torch.backends.cudnn.enabled = False

    print("Loading TensorRT engine...")
    model = YOLO(ENGINE_PATH)
    print("Engine loaded!")

    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    if not cap.isOpened():
        print("ERROR: Could not open camera")
        return

    # UDP socket (connectionless, fast)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"Sending detections to {PI_IP}:{PI_PORT}")
    print("Press 'q' to quit (if preview enabled)")
    print("-" * 50)

    fps_timer = time.time()
    frame_count = 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            # Run YOLOv8 TensorRT inference
            results = model(frame, verbose=False)

            # Build detection list in the format follow.py expects
            detections = []

            for box in results[0].boxes:
                cls = int(box.cls)
                conf = float(box.conf)

                if conf < CONFIDENCE_THRESHOLD:
                    continue

                # Only send classes we care about
                if cls not in COCO_NAMES:
                    continue

                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                det = {
                    "class": COCO_NAMES[cls],
                    "x": float(x1),
                    "y": float(y1),
                    "w": float(x2 - x1),
                    "h": float(y2 - y1),
                    "conf": round(conf, 2)
                }
                detections.append(det)

                # Draw on preview
                if SHOW_PREVIEW:
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)),
                                  (0, 255, 0), 2)
                    label = f"{COCO_NAMES[cls]} {conf:.2f}"
                    cv2.putText(frame, label, (int(x1), int(y1) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Send detections as JSON over UDP
            payload = json.dumps(detections).encode()
            try:
                sock.sendto(payload, (PI_IP, PI_PORT))
            except Exception:
                pass  # don't crash if Pi is unreachable

            # FPS counter
            frame_count += 1
            if time.time() - fps_timer >= 2.0:
                fps = frame_count / (time.time() - fps_timer)
                n_persons = sum(1 for d in detections if d["class"] == "person")
                print(f"FPS: {fps:.1f} | Detections: {len(detections)} | Persons: {n_persons}")
                frame_count = 0
                fps_timer = time.time()

            # Preview
            if SHOW_PREVIEW:
                cv2.putText(frame, f"Dets: {len(detections)}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.imshow("YOLOv8 TensorRT", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        # Send empty detection list so Pi knows to stop
        sock.sendto(json.dumps([]).encode(), (PI_IP, PI_PORT))
        cap.release()
        sock.close()
        if SHOW_PREVIEW:
            cv2.destroyAllWindows()
        print("Done.")


if __name__ == "__main__":
    main()
