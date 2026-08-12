# Buddy — Home Service Robot

Differential-drive mobile robot on a Jetson Orin Nano. Wanders or follows a
person with lidar obstacle avoidance, maps with slam_toolbox, navigates with
Nav2, and identifies people with an OAK-D. Voice interaction is planned.

ROS 2 Humble. This package holds the URDF, launch files, and Buddy's own nodes.

## Hardware

| Part          | Detail                                                      |
| ------------- | ----------------------------------------------------------- |
| Compute       | Jetson Orin Nano                                            |
| Base          | Differential drive, wheel separation **0.3302 m**, encoders |
| Motor control | ESP32 over UART @ 115200 — see `firmware/buddy_esp32`       |
| Lidar         | YDLidar X4, **mounted rotated 180°**                        |
| Camera        | Luxonis OAK-D (runs the detector NN on-device)              |

## Build

```bash
cd ~/Terry_Drive/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Third-party sources are gitignored; restore them with:

```bash
vcs import src < src/buddy_description/buddy.repos
```

## Bringup

```bash
source ~/Terry_Drive/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch buddy_description buddy_hardware.launch.py mode:=wander
```

`mode` picks who drives the base. They're mutually exclusive — each would
otherwise fight for `/cmd_vel`.

| `mode`   | What runs                                              |
| -------- | ------------------------------------------------------ |
| `idle`   | Sensors, TF, Foxglove only. Best for bringup debugging. |
| `wander` | Random exploration + scan guard. No perception, no Nav2. |
| `follow` | Person following + scan guard + the perception stack.   |
| `nav2`   | Goal-based navigation. Nav2's costmaps do the avoiding. |

`use_slam:=false` turns off slam_toolbox (it's on by default; required for
`mode:=nav2`).

slam_toolbox starts at **T+15 s** and Nav2 at **T+20 s**, so the TF tree and
`/scan` are alive before they latch on.

Shut down with `pkill -f ros2` — the delayed launches don't always die on Ctrl-C.

## Obstacle avoidance

Behaviour nodes never touch the motors directly. They publish to
`/cmd_vel_raw`, and `buddy_scan_guard` applies lidar limits before anything
reaches the ESP32:

```
follow_controller ─┐
                   ├─> /cmd_vel_raw ─> [scan_guard] ─> /cmd_vel ─> serial_bridge ─> ESP32
wanderer ──────────┘                        ^
                                          /scan
```

Putting the limiting in one place means a new behaviour can't forget to avoid
obstacles, and there's a single spot to tune. Guard rules:

- Forward motion scales down inside `slow_distance` (0.75 m), cut at
  `stop_distance` (0.35 m).
- **Rotation is never blocked** — spinning in place is how the robot escapes,
  and a guard that freezes rotation deadlocks against a wall.
- Reverse is limited by the rear sector (the X4 sees 360°).
- Missing or stale scans, or a silent driver, mean stop. It fails closed.

`/scan_guard_status` and `/wander_status` publish JSON with the live sector
distances — the fastest way to see what the robot thinks it's looking at.

### The lidar is mounted backwards

The X4 sits rotated 180°, so **scan angle 0 points out the back of the robot**.
Reading `/scan` naively makes the robot steer away from open space and into
walls. `buddy_scan_utils.resolve_scan_yaw()` reads the mounting out of TF rather
than hardcoding it, so it stays correct if the lidar is ever remounted. On
startup the guard logs what it found:

```
[buddy_scan_guard]: Lidar mounting yaw from TF: 180.0 deg
```

If that says 0.0, the static transform isn't up and avoidance will be inverted.

## Node graph

```
ESP32 ──UART──> buddy_serial_bridge ──> /odom + odom->base TF
                        ^
                    /cmd_vel
                        │
YDLidar X4 ──> /scan ──┴──> [scan_guard] <── /cmd_vel_raw <── wanderer | follow_controller
                  └──> slam_toolbox ──> map, map->odom TF ──> Nav2 (mode:=nav2)

OAK-D ──> buddy_person_detector ──> /person_detections ──> buddy_target_tracker
              └──> /person_crops                              └──> /tracked_target
                        v
              buddy_face_recognizer ──> /identified_persons, /buddy_say
```

| Node                         | Role                                                    |
| ---------------------------- | ------------------------------------------------------- |
| `buddy_serial_bridge.py`     | `/cmd_vel` → ESP32; encoder ticks → `/odom` + TF        |
| `buddy_scan_guard.py`        | Lidar safety governor, `/cmd_vel_raw` → `/cmd_vel`      |
| `buddy_wanderer.py`          | Random exploration → `/cmd_vel_raw`                     |
| `buddy_follow_controller.py` | Target pose → `/cmd_vel_raw`, gated by `/follow_enable` |
| `buddy_person_detector.py`   | OAK-D + **yolov6-nano** on RVC2 → 3D person poses       |
| `buddy_face_recognizer.py`   | InsightFace embeddings vs `face_db/` → named people     |
| `buddy_face_db.py`           | Embedding store (imported, not a node)                  |
| `buddy_enroll_face.py`       | CLI to enroll a new person                              |
| `buddy_target_tracker.py`    | Picks and tracks the follow target                      |
| `buddy_scan_utils.py`        | Shared scan sector maths (imported, not a node)         |

`/buddy_say` is already published by the face recognizer — the natural hook for
the voice phase.

## Tests

```bash
./test/run_tests.sh
```

Unit tests for the scan sector maths (including a regression test proving that
a wrong mounting yaw blinds the robot straight ahead), plus two integration
tests that run the real nodes against a synthetic lidar and assert on `/cmd_vel`.
No hardware needed.

## Enrolling a face

```bash
ros2 run buddy_description buddy_enroll_face.py --name alex --samples 20
```

Writes to `~/Terry_Drive/face_db/<name>/`; override the location with
`$BUDDY_FACE_DB`. `alex` is already enrolled with 20 samples. The store is
gitignored — biometric data, public repo — so back it up out of band.

## Maps

`~/Terry_Drive/maps/` — `home_20260418` is the current house map; `buddy_map` is
an older capture.

> Bringup runs **slam_toolbox in online-async mode**, which re-maps every
> session, so the saved map goes unused. For repeatable autonomous navigation
> the stack should switch to AMCL localizing against `home_20260418`. This is
> the main remaining gap for the "navigate the home autonomously" goal.

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

The URDF root is `BuddyBaseFootprint_link`, which is what the serial bridge
attaches `odom` to — so each frame has one parent and the chain is valid. The
static publishers carry no translation *by design*: the URDF joints already
place the mounts (`LidarMount_to_base` at x=0.05, z=0.124; `CameraMount_to_neck`
at x=-0.16, z=0.6128), so they're frame renames plus the lidar's 180° flip.

## DDS

Profiles in `~/Terry_Drive/config/`. Neither is active — set `CYCLONEDDS_URI`.

- `cyclonedds_loopback.xml` — multicast off, localhost only. Stops cross-talk
  with other ROS hosts.
- `cyclonedds_largemsg.xml` — bigger max message/fragment size, for streaming
  camera images to Foxglove on another machine.

## Fixed 2026-08-12 — "goal does nothing" bugs

1. **`bt_navigator` had no `plugin_lib_names`.** The config used the
   `navigators:` / `nav2_bt_navigator::…` form, which is **Iron/Jazzy syntax**.
   Humble ignores it and needs the BT node libraries listed explicitly, so the
   tree loaded with zero registered nodes and every `NavigateToPose` goal failed
   on acceptance. **Main cause.**
2. **Neither costmap used the lidar.** Both listed one observation source on
   `/oak/stereo/image_raw` — a **depthai-ros** topic nothing publishes, typed
   `PointCloud2` despite the name, in a frame absent from TF. `/scan` went
   unused entirely. Both now take `/scan` as a `LaserScan` in frame `laser`.
3. **`odom_topic: /rtabmap/odom`** in `bt_navigator` and `velocity_smoother`,
   left from an abandoned RTAB-Map experiment. The bridge publishes `/odom`.
4. **Motor deadband.** `RotateToGoal.slowing_factor: 5.0` makes the final
   in-place turn ~0.2 rad/s → PWM 27, below stiction, so the base stalled just
   short of the goal. The firmware now floors nonzero commands at `MIN_PWM`.

## Known issues

1. **SLAM runs instead of localization** — see Maps.
2. **Firmware fork unresolved** — the active `main.cpp` is open-loop; the
   archived `.ino` has PID. Deliberate: at these speeds break-away torque
   matters more than velocity tracking. Revisit if odometry drift appears.
3. `collision_monitor` is configured but never launched on Humble;
   `worlds/apartment.sdf` and the `ros_gz_*` deps are left from a Gazebo phase
   with no sim launch file.
