# Buddy — Prototypes & Experiments

Standalone scripts written before the ROS 2 stack existed. These were loose in
`$HOME`, untracked. Collected here so home stays clean and nothing is lost.

Nothing in here is launched by the robot. See `~/buddy_ws` for the live stack.

## `voice/` — phase 3, not yet integrated

The voice interaction pipeline. **This is unfinished future work, not dead
code.** It runs split across two machines:

```
Jetson mic ──UDP──> desktop PC (RTX 5070)
                      └─> Silero VAD ─> faster-whisper STT
                          ─> Qwen3 8B (Ollama) ─> Kokoro TTS (Docker)
                          ─> Bluetooth speaker
```

| File                   | Role                                              |
| ---------------------- | ------------------------------------------------- |
| `jetson_mic_stream.py` | Runs **on the Jetson**. Streams Brio mic over UDP. |
| `voice_assistant_v2.py`| Runs **on the desktop PC**. Full STT→LLM→TTS loop. |
| `test_connection.py`   | Websocket ping used to debug the Jetson↔PC link.   |

When this gets integrated, the natural shape is a ROS 2 node wrapping the
desktop side, publishing recognized intents onto a topic the robot subscribes
to — rather than the current ad-hoc UDP socket.

## `vision/` — superseded

Early perception experiments, all replaced by `buddy_person_detector.py` and
`buddy_face_recognizer.py` in the `buddy_description` package. Kept only for
reference; safe to delete once you're sure you don't want them.

| File                | Superseded by                                          |
| ------------------- | ------------------------------------------------------ |
| `jetson_vision.py`  | The ROS detector + `buddy_follow_controller.py`. Ran YOLOv8n TensorRT, sent targets over a raw socket. |
| `detect_stream.py`  | Same — socket-based detection streaming.               |
| `detect.py`         | YOLOv8n `.engine` smoke test.                          |
| `yolo_live.py`      | Webcam YOLO smoke test.                                |
| `yolo_view.py`      | Webcam YOLO smoke test.                                |
| `cam_test.py`       | Bare OpenCV camera-open check.                         |

Note the live stack no longer uses YOLOv8 at all — `buddy_person_detector.py`
runs **yolov6-nano** on the OAK-D's RVC2 via the DepthAI model zoo, so the old
`yolov8n.pt/.onnx/.engine` weights were removed.
