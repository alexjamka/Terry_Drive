#!/usr/bin/env python3
"""
Enroll a face in Buddy's database.

Captures N frames from the OAK-D Lite, runs InsightFace to extract a
512-d ArcFace embedding per detected face, and saves them under
~/buddy_ws/face_db/<n>/.

Usage:
    python3 buddy_enroll_face.py --name alex --samples 20

Tips while enrolling:
    - Stand ~1m from the camera, well-lit
    - Slightly vary angle and expression across samples
    - The tool pauses ~0.3s between captures so you have time to move
"""

import argparse
import sys
import time

import cv2
import depthai as dai
import numpy as np
from insightface.app import FaceAnalysis

from buddy_face_db import save_embedding, list_enrolled, person_dir


def build_pipeline(width: int = 640, height: int = 480) -> dai.Pipeline:
    """
    Minimal depthai v3 pipeline: color camera → host output.
    """
    pipeline = dai.Pipeline()

    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    cam_out = cam.requestOutput((width, height), dai.ImgFrame.Type.BGR888i)

    q = cam_out.createOutputQueue(maxSize=4, blocking=False)
    return pipeline, q


def main() -> int:
    parser = argparse.ArgumentParser(description="Enroll a face in Buddy's face_db")
    parser.add_argument("--name", required=True, help="Person's name (e.g., alex)")
    parser.add_argument("--samples", type=int, default=20,
                        help="Number of embedding samples to capture (default 20)")
    parser.add_argument("--min-interval", type=float, default=0.3,
                        help="Min seconds between captures (default 0.3)")
    parser.add_argument("--overwrite", action="store_true",
                        help="Wipe existing samples for this person before enrolling")
    args = parser.parse_args()

    name = args.name.lower().strip()

    # Optional wipe
    if args.overwrite:
        d = person_dir(name)
        if d.exists():
            for f in d.glob("*.npy"):
                f.unlink()
            print(f"[enroll] wiped existing samples for '{name}'")

    # InsightFace on GPU
    print("[enroll] loading InsightFace (buffalo_s) on GPU...")
    app = FaceAnalysis(name="buffalo_s",
                       providers=["CPUExecutionProvider"])
    app.prepare(ctx_id=0, det_size=(320, 320))

    # OAK-D
    print("[enroll] starting OAK-D Lite pipeline...")
    pipeline, q = build_pipeline()

    captured = 0
    last_capture = 0.0
    print(f"[enroll] capturing {args.samples} samples for '{name}'. "
          f"Look at the camera, vary angle slightly.")

    with pipeline:
        pipeline.start()
        while captured < args.samples:
            frame_msg = q.get()
            if frame_msg is None:
                continue
            frame = frame_msg.getCvFrame()  # BGR numpy

            now = time.time()
            if now - last_capture < args.min_interval:
                continue

            faces = app.get(frame)
            if not faces:
                print(f"\r[enroll] no face detected...", end="", flush=True)
                continue

            # Use the largest face in frame (likely the closest person)
            face = max(faces, key=lambda f: (f.bbox[2] - f.bbox[0]) * (f.bbox[3] - f.bbox[1]))
            emb = face.normed_embedding  # already 512-d unit-norm

            save_embedding(name, emb)
            captured += 1
            last_capture = now
            print(f"\r[enroll] captured sample {captured}/{args.samples} "
                  f"(det_score={face.det_score:.2f})    ", flush=True)

    print(f"\n[enroll] done. Enrolled database:")
    for n, count in list_enrolled():
        print(f"  {n}: {count} samples")
    return 0


if __name__ == "__main__":
    sys.exit(main())
