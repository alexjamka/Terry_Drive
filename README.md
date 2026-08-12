# terry_drive

Robot software stack for Terry/Buddy, running on an NVIDIA Jetson (ARM64 / Tegra).

**Start here: [`ros2_ws/src/buddy_description/README.md`](ros2_ws/src/buddy_description/README.md)**
— hardware, bringup modes, obstacle avoidance, node graph, TF tree, known issues.

## Layout

| Path | Description |
|------|-------------|
| `ros2_ws/src/buddy_description/` | Main ROS 2 package: URDF/xacro, launch, Nav2 + SLAM params, RViz config, sim world, and Buddy's nodes (obstacle avoidance, wander, person detection, face recognition, follow, serial bridge). |
| `firmware/buddy_esp32/` | ESP32 motor controller firmware (PlatformIO), plus an archived Arduino variant. |
| `scripts/` | Pre-ROS experiments: `voice/` (unfinished phase 3) and `vision/` (superseded). |
| `config/` | CycloneDDS profiles and the Foxglove layout. |
| `maps/` | Saved occupancy grids (`.pgm` + `.yaml`). |
| `face_db/` | Enrolled face embeddings. **Gitignored** — biometric data, public repo. |

## Quick start

```bash
cd ~/Terry_Drive/ros2_ws
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch buddy_description buddy_hardware.launch.py mode:=wander   # explore a room
ros2 launch buddy_description buddy_hardware.launch.py mode:=follow   # follow a person
ros2 launch buddy_description buddy_hardware.launch.py mode:=nav2     # goal navigation
```

Flash the firmware with `cd firmware/buddy_esp32 && pio run -t upload`.

## Notes

Third-party ROS dependencies (ydlidar_ros2_driver, and optionally depthai-ros /
rplidar_ros) are not vendored. Restore them with:

```bash
vcs import ros2_ws/src < ros2_ws/src/buddy_description/buddy.repos
```

Large model artifacts (`*.engine`, `*.onnx`, `*.pt`) are gitignored. The live
stack doesn't use them — the person detector runs yolov6-nano on the OAK-D
itself, pulled from the DepthAI model zoo.
