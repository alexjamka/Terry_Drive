# Buddy — Home Service Robot

Differential-drive mobile robot built on a Jetson Orin Nano. Navigates the house
with Nav2 + slam_toolbox, detects and identifies people with an OAK-D, and can
follow a recognized person. Voice interaction is planned but not integrated.

ROS 2 Humble. This package (`buddy_description`) holds the URDF, the launch
files, and all of Buddy's own nodes.

## Hardware

| Part            | Detail                                                    |
| --------------- | --------------------------------------------------------- |
| Compute         | Jetson Orin Nano                                           |
| Base            | Differential drive, wheel separation **0.3302 m**, quadrature encoders |
| Motor control   | ESP32 over UART @ 115200 — see the [`buddy_esp32`](../../../buddy_esp32) repo |
| Lidar           | YDLidar X4                                                 |
| Camera          | Luxonis OAK-D (depth + RGB, runs the detector NN on-device) |

## Build

```bash
cd ~/buddy_ws
colcon build --symlink-install
source install/setup.bash
```

Third-party sources under `src/` are listed in `buddy.repos`. To restore them on
a fresh machine:

```bash
vcs import src < src/buddy_description/buddy.repos
```

## Bringup

```bash
source ~/buddy_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch buddy_description buddy_hardware.launch.py
```

`buddy_hardware.launch.py` starts everything, with two deliberate delays:
slam_toolbox at **T+15 s** and Nav2 at **T+20 s**, so the TF tree and laser scan
are alive before the mapping and navigation stacks latch onto them.

Visualize with Foxglove — the bridge starts automatically, and there's a saved
layout at `../../buddy_foxglove_layout.json`.

Shut down with `pkill -f ros2` (the delayed launches don't always die with Ctrl-C).

## Node graph

```
                   ┌─ /odom ─────────────────────────────┐
ESP32 ──UART──> buddy_serial_bridge <── /cmd_vel ─┐      │
                                                   │      v
YDLidar X4 ──> ydlidar_ros2_driver ── /scan ──> slam_toolbox ──> Nav2 ──┘
                                                          (map, TF)

OAK-D ──> buddy_person_detector ──> /person_detections (3D poses)
              │                        └──> buddy_target_tracker ──> /tracked_target
              └──> /person_crops                                          │
                        v                                                 v
              buddy_face_recognizer ──> /identified_persons    buddy_follow_controller
                        └──> /buddy_say                                   └──> /cmd_vel
```

| Node                         | Role                                                       |
| ---------------------------- | ---------------------------------------------------------- |
| `buddy_serial_bridge.py`     | `/cmd_vel` → ESP32; encoder ticks → `/odom`                |
| `buddy_person_detector.py`   | OAK-D pipeline, **yolov6-nano** on the RVC2, emits 3D person poses + face crops |
| `buddy_face_recognizer.py`   | InsightFace embeddings vs. `face_db/`, names people        |
| `buddy_face_db.py`           | Embedding store + match threshold (imported, not a node)    |
| `buddy_enroll_face.py`       | CLI to enroll a new person                                  |
| `buddy_target_tracker.py`    | Chooses and tracks which detection is *the* follow target   |
| `buddy_follow_controller.py` | Target pose → `/cmd_vel`, gated by `/follow_enable`         |

`/buddy_say` is already published by the face recognizer — it's the natural hook
for the voice phase.

## Enrolling a face

```bash
ros2 run buddy_description buddy_enroll_face.py --name alex --samples 20
```

Writes embeddings to `~/buddy_ws/face_db/<name>/`. `alex` is already enrolled
with 20 samples.

## Maps

`~/buddy_ws/maps/` — `home_20260418` is the current house map; `buddy_map` is an
older capture kept for reference.

> **Note:** bringup currently runs **slam_toolbox in online-async mode**, which
> re-maps every session, so the saved map goes unused. For repeatable autonomous
> navigation the stack should switch to AMCL localizing against
> `home_20260418`. This is the main gap between what runs today and the
> "navigate the home autonomously" goal.

## TF tree

Verified consistent — every consumer agrees on `BuddyBaseFootprint_link`.

```
map                     <- slam_toolbox
 └── odom               <- slam_toolbox
      └── BuddyBaseFootprint_link      <- buddy_serial_bridge (wheel odometry)
           └── BuddyBase_link          <- robot_state_publisher, from here down
                ├── BuddyNeck_link
                │    └── BuddyCameraMount_link
                │         └── oak_d_link        <- static, identity
                ├── LidarMount_link
                │    └── laser                  <- static, yaw 3.14159
                ├── BuddyWheel{Left,Right}_link
                └── caster_link, caster_rear_link
```

The URDF root is `BuddyBaseFootprint_link`, which is also what the serial bridge
attaches `odom` to — so each frame has exactly one parent and the chain is valid.
The two static publishers carry no translation *by design*: the URDF joints
already place the mounts (`LidarMount_to_base` at `x=0.05, z=0.124`,
`CameraMount_to_neck` at `x=-0.16, z=0.6128`), so the statics are just frame
renames, plus the lidar's 180° mounting flip.

## DDS

Two CycloneDDS profiles live in `~/buddy_ws/config/`. Neither is active — set
`CYCLONEDDS_URI` to opt in.

- `cyclonedds_loopback.xml` — multicast off, localhost peers only. Use for
  single-machine runs and to stop cross-talk with other ROS hosts.
- `cyclonedds_largemsg.xml` — bigger max message + fragment size. Use when
  streaming camera images to Foxglove on another machine.

## Fixed 2026-08-12 — "goal does nothing" bugs

Four independent faults, any one of which stops navigation:

1. **`bt_navigator` had no `plugin_lib_names`.** The config used the
   `navigators:` / `nav2_bt_navigator::…` form, which is **Iron/Jazzy syntax**.
   Humble ignores it and needs the BT node libraries listed explicitly. The
   result was a behavior tree with zero registered nodes, so every
   `NavigateToPose` goal failed the instant it was accepted. **This was the
   main cause.**
2. **Neither costmap used the lidar.** Both listed a single observation source
   `depth` on `/oak/stereo/image_raw` — a **depthai-ros** topic that nothing
   publishes (the detector drives the OAK-D through the DepthAI Python API and
   publishes `/camera/color/image_raw`). It was also declared `PointCloud2`
   while the name says image, with `sensor_frame: oak_rgb_camera_optical_frame`,
   a frame absent from the TF tree. Wrong topic, wrong type, wrong frame — so
   `/scan` from the YDLidar was never consumed and the robot navigated blind.
   Both costmaps now take `/scan` as a `LaserScan` in frame `laser`.
3. **`odom_topic: /rtabmap/odom`** in `bt_navigator` and `velocity_smoother` —
   left from an abandoned RTAB-Map experiment. Nothing publishes it; the serial
   bridge publishes `/odom`.
4. **Motor deadband.** `RotateToGoal.slowing_factor: 5.0` makes the final
   in-place turn ~0.2 rad/s → PWM 27, below stiction, so the base stalled just
   short of the goal. The firmware now floors nonzero commands at `MIN_PWM`.

## Known issues

1. **SLAM runs instead of localization** — see Maps above.
2. **Firmware fork is unresolved** — the active `main.cpp` is open-loop; the
   archived `.ino` has PID. Deliberate choice for now: the base moves slowly
   enough that deadband compensation matters more than closed-loop tracking.
   Revisit if odometry drift shows up under load.
3. `collision_monitor` is configured but never launched on Humble; `worlds/apartment.sdf`
   and the `ros_gz_*` deps in `package.xml` are left from a Gazebo phase with no
   sim launch file. Either wire them up or drop them.
