# terry_drive

Robot software stack for Terry, running on an NVIDIA Jetson (ARM64 / Tegra).

## Layout

| Path | Description |
|------|-------------|
| `ros2_ws/src/buddy_description/` | Main ROS 2 package: URDF/xacro, launch files, Nav2 + SLAM params, RViz config, simulation world, and Python nodes (face recognition, person detection, follow controller, serial bridge, lidar). |
| `firmware/buddy_esp32/` | ESP32 firmware (PlatformIO). |
| `firmware/buddy_motor_controller/` | Arduino motor controller sketch. |
| `scripts/` | Standalone Jetson vision/voice scripts (YOLO detection, camera tests, mic streaming, voice assistant). |
| `config/` | CycloneDDS configs and Foxglove layout. |
| `maps/` | Saved occupancy-grid maps (`.pgm` + `.yaml`). |

## Notes

Third-party ROS dependencies (depthai-ros, rplidar_ros, ydlidar_ros2_driver) are
not vendored here — clone them into `ros2_ws/src/` from their upstream repos.

Large model artifacts (`*.engine`, `*.onnx`, `*.pt`) are gitignored.
