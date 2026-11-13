# CLAUDE.md — Buddy / Terry robot

Orientation for an agent picking this up cold. Full reference lives in
[`ros2_ws/src/buddy_description/README.md`](ros2_ws/src/buddy_description/README.md);
this file is the hardware truth and the traps.

**Everything below was measured on the robot, not inferred.** Where a number is
still uncalibrated it says so. Last verified on hardware **2026-11-13**.
Historical measurements below retain their dates; the 2026-11-13 findings
supersede the former universal lidar-range claim and wheel-side convention.

## Canonical layout

`~/Terry_Drive` is the only source of truth → `github.com/alexjamka/Terry_Drive`.
Consolidated here 2026-08-12 from `~/buddy_ws`, `~/buddy_esp32` and loose
`$HOME` scripts, which are **gone**. Two older GitHub accounts (`Beamer94`,
`beamer94-cmd`) hold a stale `buddy-robot` repo — don't push there.

```
ros2_ws/src/buddy_description/   the ROS 2 package (build from ros2_ws/)
firmware/buddy_esp32/            ESP32 firmware (PlatformIO) + archived PID .ino
scripts/                         pre-ROS experiments: voice/ (phase 3), vision/
config/ maps/ face_db/           DDS profiles, occupancy grids, embeddings
```

```bash
cd ~/Terry_Drive/ros2_ws && colcon build --symlink-install
source install/setup.bash && export ROS_DOMAIN_ID=42
./src/buddy_description/test/run_tests.sh      # no hardware needed

# Lidar SLAM (slam_toolbox), corrected wheels only (default):
ros2 launch buddy_description buddy_hardware.launch.py mode:=teleop use_slam:=true

# Select ESP IMU fusion for lidar SLAM (hold still ~6 s at startup):
ros2 launch buddy_description buddy_hardware.launch.py mode:=teleop use_slam:=true use_imu:=true

# Camera SLAM (rtabmap) — needs use_slam:=false or the two fight over map->odom:
ros2 launch buddy_description buddy_hardware.launch.py mode:=teleop use_slam:=false
ros2 launch buddy_description buddy_vslam.launch.py

# Drive it (latching keyboard teleop, publishes /cmd_vel_raw through the guard):
ros2 run buddy_description buddy_teleop.py
```

`mode` = `idle` | `wander` | `follow` | `nav2` | `teleop`, mutually exclusive
(each would fight for `/cmd_vel`). `use_slam:=false` to skip slam_toolbox.

**`--symlink-install` means edits to `scripts/`, `config/`, `urdf/` and
`launch/` take effect on the next node start with no rebuild.** Only a NEW file
needs `colcon build` (to create the symlink) — and `chmod +x` it, because
symlink-install does not apply `install(PROGRAMS)` permissions.

## Hardware facts (measured)

### Drivetrain — the velocity scale is WRONG and shapes everything

`firmware/buddy_esp32/src/main.cpp` maps velocity→PWM as `v * 833`, i.e. it
assumes PWM 250 = 0.30 m/s. Measured 2026-11-12 by commanding a speed and
reading `/odom` distance:

| commanded | PWM | achieved |
| --------- | --- | -------- |
| 0.20 m/s | 166 | **43%** (stalls intermittently) |
| 0.25 m/s | 208 | 100% |
| 0.30 m/s | 249 | **144%** |

PWM 249 really produces **~0.43 m/s**, so the true constant is ≈581, not 833.
Consequences:

- `MIN_PWM 150` is a real floor of **0.26 m/s**, not the 0.18 m/s previously
  documented here.
- The slowest non-stalling in-place turn is `2 × 0.26 / 0.3302` = **1.57 rad/s**
  (≈90 °/s). The guard's `max_angular` was 1.5, just under that, so every turn
  the wanderer asked for was throttled into the stall zone. It is now **2.0**.
- PWM 150 breaks away on the bench with wheels ELEVATED but **not reliably
  under load on the floor**. Commanding anything near it makes the robot judder
  rather than crawl.
- Gentle arcs remain impossible; only straight runs and in-place turns.

The firmware constant has deliberately NOT been changed — only odometry and the
guard were corrected. Changing it would shift every tuned speed at once.

### Drivetrain — sign conventions, explicit flags

Derived by driving **one wheel at a time** and watching which counter moved and
in which direction. In `firmware/buddy_esp32/src/main.cpp`:

```c
#define INVERT_LEFT_MOTOR    true   // both motors ran backwards
#define INVERT_RIGHT_MOTOR   true
#define INVERT_LEFT_ENCODER  true
#define INVERT_RIGHT_ENCODER false
#define ENC_LA 4   // ENC_L/ENC_R were TRANSPOSED before 2026-11-11
#define ENC_LB 13
#define ENC_RA 18
#define ENC_RB 19
// Motor driver: ENA 25, IN1 33, IN2 32 | ENB 26, IN3 14, IN4 27
```

**If the drivetrain is rewired, re-derive one flag at a time — do not reason it
out.** Encoder signs alone cannot distinguish "motor reversed" from "encoder
reversed"; both produce identical data. Only watching the wheels resolves it.

**2026-11-13 correction:** the ESP's L/R protocol labels are physically swapped.
A positive angular command actually turned RIGHT, confirmed by the owner,
lidar alignment and the new gyro. The bridge now defaults
`swap_wheel_sides:=true`: it exchanges incoming encoder sides AND negates
outgoing angular commands. This restores +yaw = counterclockwise/left in ROS.
Do not undo the measured low-level inversion flags above. Direct `V:` commands
to the ESP still use its original convention; the normalization is in ROS.
The earlier turn magnitudes/track calibration remain useful, but their signed
ROS/physical interpretation was wrong. A final hardware run verified positive
ROS command, encoder yaw and gyro yaw now agree.

The right motor is stiffer than the left. The owner considers this normal and
does **not** want it treated as a fault.

### Odometry — calibrated against the lidar, not a ruler

In `scripts/buddy_serial_bridge.py`:

```python
self.WHEEL_SEPARATION = 0.3884   # EFFECTIVE; physical track is 15" = 0.381
self.WHEEL_DIAMETER   = 0.0635   # measured 2.5"
self.TICKS_PER_REV    = 455      # field-calibrated
```

**Owner corrected the physical track to 15 inches / 0.381 m centre-to-centre
on 2026-11-13; it was already that width during these recordings.** Refined to
**0.3884** the same day (see the 2026-11-13 calibration section) - only **2%
above physical**, not 21%. The older 0.3302 m starting value
was geometrically wrong, so its measured yaw error included wrong geometry as
well as tyre scrub. An in-place turn can rotate less than wheel travel implies. Measured by commanding pure in-place turns and comparing
odom yaw against rotation measured from the lidar (align successive scans, take
the best yaw shift — independent of the encoders):

| commanded | odom | lidar truth | ratio |
| --------- | ---- | ----------- | ----- |
| +1.6 × 0.9 s | + 87.0° | + 70.7° | 1.23 |
| −1.6 × 0.9 s | − 98.2° | − 82.8° | 1.19 |
| +1.6 × 1.4 s | +139.8° | +115.1° | 1.21 |
| −1.6 × 1.4 s | −142.3° | −115.8° | 1.23 |

Mean gain 1.215 → `0.3302 × 1.215 = 0.4012`. After the change the same test
reads **0.983** (2%, scattered either side of 1.0 = noise, not bias).

This mattered enormously: every turn told slam_toolbox the robot had rotated a
fifth further than it had, so walls landed at the wrong angle and maps showed
rooms drawn twice. **Re-run after any tyre, wheel or track change** — the rubber
bands on the tyres change the scrub.

Firmware SOURCE now uses physical `WHEEL_SEPARATION = 0.381` to convert
`cmd_vel` to wheel speeds. URDF wheel centres are y = +/-0.1905 m and Gazebo
uses the same 0.381 m track. Chassis radius and the IMU/lidar mounting offsets
were not changed by this lateral-spacing correction.
**Deployment status:** this session built the corrected firmware
but did NOT flash it. Installed ESP firmware still uses 0.3302 m for commands.
Flash and recheck achieved turn rates before relying on the new command scale;
previous command/PWM measurements below belong to the installed old scale.
The empirical ROS odometry width remains 0.4012 m, so existing bag odometry
must not be rescaled merely because the physical-width documentation changed.

**UNRESOLVED: linear odometry over-reports by ~18%.** Measured the same way
(drive at a flat wall, compare the change in front range with odom distance):
one clean accepted run gave 0.543 m true vs 0.642 m odom with the robot yawing
only 0.3°. This contradicts an earlier tape-measure check (odom 0.546 m vs 21–22"
measured) — short bursts may be dominated by spin-up slip. **`TICKS_PER_REV` has
NOT been changed on one run.** To settle it without slip: lift the robot, mark a
wheel, turn it exactly one revolution by hand, count the tick delta.

### Lidar — range depends on the observation; the 2.5 m limit was disproved

YDLidar **S2PRO** (it reports itself as such; it was configured from the
vendor's X4.yaml for a long time). Spinning disc faces **forward**, motor faces
back. A 40×40 mm extrusion sits ~0.117 m behind the disc as a permanent return.

**Measured 2026-11-12 — 79,305 returns over 20 s:**

```
  0.5-1.0  34096 ############################################################
  1.0-1.5  16290 ############################
  1.5-2.0  19984 ###################################
  2.0-2.5   8752 ###############
  2.5-3.0      2
  3.0+         0
```

That 2026-11-12 sample had a maximum return of 2.51 m. It was incorrectly
promoted to a universal hardware limit. **2026-11-13: valid returns reach
8.069 m in the first IMU/lidar recording** (95th percentile 2.10 m, 99th 6.25 m).
Do not recommend replacing the sensor based on the older sample alone.
`loop_search_maximum_distance` is a candidate scan-pose proximity threshold,
not a minimum range the lidar has to reach. The old explanation that a 3 m
setting prevents a 2.5 m lidar from ever closing a loop was incorrect.

Scan rate is **3.7 Hz** (1340 pts, 360°) and is NOT settable — in the SDK,
`hasScanFrequencyCtrl()` excludes S4 (== S2PRO) and X4, so the `frequency`
parameter is silently ignored. At 3.7 Hz a 1.6 rad/s turn puts ~25° between
scans, more than correlative scan matching can associate.

`laser_tf` publishes **`--yaw 0.0`**. It used to say `3.14159`, which mirrored
every avoidance decision. `resolve_scan_yaw()` reads this from TF and logs it at
startup — **that did not protect us**, because TF itself was wrong and the node
applied it faithfully. A unit test pins the old wrong value as a regression.

`min_valid_range` is **0.05** (a sensor floor only). It used to be 0.20 to reject
the extrusion, which blinded the FRONT sector at close range and caused a
collision; the self-return is now masked by ANGLE instead
(`self_mask_from_deg 165`, `self_mask_range 0.25`). The S2PRO reports **`0.0`
for "no return"**, not `inf`.

### Lidar is BLIND ASTERN — and the guard used to drive into it

Measured 2026-11-12, bearings in the robot frame:

| band | returns | nearest |
| ---- | ------- | ------- |
| 150–160° | 35 | 1.48 m |
| 160–170° | 22 | 2.10 m |
| 170–180° | 3 | 0.117 m (the extrusion) |
| −180…−170° | **0** | — |
| −170…−160° | **0** | — |

**68% of astern rays return nothing**, including a 20° wedge with zero returns —
the neck pole sits ~0.29 m behind the lidar and occludes it. With the extrusion
masked out, the guard computed rear ≈ 0.98 m and called it clear **while a wall
was right there, and reversed into it.**

Root cause: `min_range_in_sector()` returns `+inf` for an empty sector, and
`+inf` reads as "infinitely clear" when it means "I cannot see". **Fail-open.**
This is the same bug class as the earlier front collision.

Fixed: `sector_distances(..., with_counts=True)` also returns per-sector return
counts, and the guard checks **coverage before distance** when reversing —
fewer than `min_rear_points` (8) returns means `reason: "rear_unknown"` and
reverse is refused. Rotation is never blocked, so the robot can always turn and
drive out forwards. A regression test pins "blind ≠ clear".

### OAK-D — better mapping sensor than the lidar

```
name     OAK-D-LITE-FF
cameras  CAM_A IMX214 (colour), CAM_B/CAM_C OV7251 (stereo)
IMU      BMI270, firmware 1.0.0 — CONFIRMED STREAMING
mounted  0.8636 m (34") above the floor, centred, facing forward
```

Measured depth from the same spot the lidar died at 2.5 m: **0.35 m to ~19.8 m**,
95th pct 12.6 m, 69% of pixels valid (79% in the centre quarter). Treat 4–8 m as
the trustworthy range — far-field stereo from a short baseline is noisy.

**The OAK-D has a working IMU. Do not buy one.** Gravity reads −9.76 on Y; gyro
bias at rest is ~−0.012 rad/s (0.7 °/s), so it needs bias estimation — feed it
through `robot_localization` rather than integrating raw.

**It is plugged into USB 2.0** (enumerates on bus 001; bus 002 is the unused USB
3.0 hub). That is why requesting 15 fps yields ~5.5 fps RGB / 2.8 Hz depth.
Moving it to USB 3 should roughly triple it.

### ESP IMU — MPU-6500 identity on an MPU-9250/6500/9255 family board

Owner added this sensor, independent of the OAK-D BMI270. Confirmed on hardware
2026-11-13: `WHO_AM_I=0x70` (MPU-6500), address **0x68**, SDA **GPIO21**,
SCL **GPIO22**, AD0 **GND**. Only the accelerometer/gyro are used; no
magnetometer or absolute heading is claimed.

Owner supplied +X forward, +Y left, hence +Z up. Position is **100 mm forward
of the chassis centre**, **95 mm right**, **160 mm above the floor**:

- `BuddyBase_link -> BuddyImu_link`: xyz **(0.100, -0.095, 0.02825)**, rpy zero.
- `BuddyBaseFootprint_link -> BuddyImu_link`: xyz **(-0.0016, -0.095, 0.160)**.
- The 0.13175 m base-frame height is subtracted exactly once. Live TF verified.

Firmware now emits `I:<micros>,ax,ay,az,gx,gy,gz` with signed raw counts at
**50 Hz**, alongside unchanged **20 Hz** `E:` encoder frames. Accel range is
+/-2 g (16384 LSB/g); gyro +/-250 deg/s (131 LSB/(deg/s)); both use the
41 Hz DLPF. `buddy_imu.py` converts to m/s² and rad/s. The MCU timestamp is
carried into ROS timing; 32-bit wrap is supported and reset/data outages trigger
recalibration. A failed read never becomes a zero-valued valid sample.

`/imu/data_raw` is uncorrected; `/imu/data` subtracts startup gyro bias.
Hold still for ~6 s: 2 s warm-up + 200 consecutive stationary samples. Encoder
motion, active drive commands or excessive gyro/accel reset the calibration
window. Calibration is fixed during motion, not continuously adapted. Startup
means in the first run were approximately **(0.00125, 0.01931, 0.00376) rad/s**;
corrected stationary Z mean was **-0.000114 rad/s**, noise std **0.000620 rad/s**.
Orientation covariance starts at -1 because no orientation is measured.

`use_imu:=true` starts `robot_localization` with `config/ekf_imu.yaml`:
wheel vx/vy-zero/yaw-rate + calibrated gyro Z, planar mode. The bridge publishes
`/odom/wheel` without odom TF; the EKF publishes `/odom` and the sole
`odom -> BuddyBaseFootprint_link` transform. Wheel pose is not double-counted.
Acceleration is NOT fused: gravity, tilt, vibration and off-centre lever-arm
motion would need further treatment. The measured lever arm still belongs in TF.
`use_imu:=false` (default) keeps corrected wheel `/odom` + bridge TF for comparison.
The EKF does not itself estimate gyro bias; the serial bridge does that.

`robot_localization` was absent and this account lacks passwordless sudo.
Signed-APT binary dependencies were installed into
`~/.local/share/buddy_imu_deps` (robot_localization 3.5.4, geographic_msgs,
diagnostic_updater, GeographicLib). The rebuilt workspace's `install/setup.bash`
chains this prefix automatically. No shell startup file or /opt/ros was changed.
If rebuilding from a fresh shell, source `/opt/ros/humble/setup.bash`, then
`~/.local/share/buddy_imu_deps/local_setup.bash` before `colcon build`.
On another machine install the normal `ros-humble-robot-localization` dependency.

### Serial devices — identify them by what they SAY

The ESP32 and the lidar are both CP2102 (`10c4:ea60`) reporting serial `0001`,
so **`/dev/serial/by-id` collides** and `ttyUSB*` order is unstable. The path has
changed at least four times. As of 2026-11-12:

```
usb-0:2.2   ESP32
usb-0:2.3   lidar   <- set in config/ydlidar_x4.yaml
```

Re-derive by probing, never by position — the ESP32 answers 115200 with
printable `E:<l>,<r>` frames; the lidar emits binary with a `0x55` ('U') sync
byte. **Allow a full ~1.8 s settle before reading**: a shorter one reads the
ESP32 mid-boot-log (74880 baud, garbage at 115200) and mis-identifies it as the
lidar. That cost a wrong "the ESP32 is dead" call on 2026-11-12. The probe
script is in `config/ydlidar_x4.yaml`'s header comment.

`buddy_serial_bridge.py` autodetects by probing for `E:` frames. The ESP32 is on
**USB**, not the 40-pin UART.

## Traps that cost real time

**ESP32 power contention.** Feeding the driver's 5 V into the ESP32 while USB is
connected browns out the 3.3 V rail. **Two boards were wrongly declared dead
this way.** Power from USB **or** the breadboard, never both.

**`pkill -f` matches your own shell command.** `pkill -f 'buddy_wanderer'` run
from a shell whose command line contains that text kills the shell itself
(exit 144). Use the bracket trick: `pkill -f '[b]uddy_wanderer'`. Likewise
`pgrep -fc <name>` self-matches and reports dead nodes as alive — **`ros2 node
list` is the only honest check.** Also `pkill -f 'a\|b'` matches nothing (it
takes an ERE: `'a|b'`), and a non-zero `pkill` aborts an `&&` chain.

**Leftover ROS processes corrupt serial.** `ros2 launch` children survive killing
the launch, and **two readers on one tty garbles the stream for both**. Verify
with `fuser /dev/ttyUSB0 /dev/ttyUSB1` (must be empty) and `ros2 node list`.
Note that `ros2 node list` can also show *stale* entries for several seconds
after a kill — cross-check with `ps` before concluding two stacks are running.

**Sensor QoS.** Lidar drivers publish `/scan` **BEST_EFFORT**. Subscribing with
the default RELIABLE profile is QoS-incompatible and ROS delivers **nothing**,
silently. Use `qos_profile_sensor_data`.

**An empty sector is not a clear sector.** See the astern section above. Any new
code that acts on `sector_distances()` must check coverage before trusting a
distance.

**The guard must never output more than it was asked for.** The first version of
the `min_linear` fix lifted a 0.20 m/s request to 0.26 — a safety governor
accelerating the robot. The integration test caught it; the floor is now
`min(min_linear, requested)`.

**rtabmap silently ignores unknown parameter names.** `Grid/FromDepth` does not
exist in rtabmap 0.22 (it became `Grid/Sensor`: 0=scan, 1=depth, 2=both), and
passing it looks like it worked. Verify with
`ros2 param get /rtabmap Grid/Sensor`.

**`Grid/FootprintHeight` is not the sensor height.** It filters points over the
robot's own footprint and needs `Grid/FootprintLength`/`Width` set with it.
Setting it to 0.9 (the camera height) produced a map with **zero occupied
cells** — all free space, no walls.

**`resolve_scan_yaw()` spins the node** while waiting for TF, which can dispatch
`_on_scan` before `scan_yaw` exists. Both nodes define it before creating
subscriptions and no-op until it is known.

**PlatformIO:** `python3.10-venv` is **not installed**, so the IDE extension
cannot build `~/.platformio/penv` — that is the "Python error". The CLI works
(`python3 -m platformio`). Needs `sudo apt install python3.10-venv`. Also
`~/.local/bin` is added only by `.profile` (login shells).

## Mapping — what has been tried

The historical metric is `occupied/(occupied+free)`; unknown cells (205) are
excluded. It is a wall-cell fraction, NOT sufficient evidence of crisper walls:
changes in explored area, missing walls or a frozen map also change it. Compare
the same recording and inspect maps/accepted scans; do not rank unrelated runs
by this ratio alone. Counting unknown as free is a parser bug.

| map | occ/(occ+free) | explored | notes |
| --- | -------------- | -------- | ----- |
| `home_20260418` | 24.09% | 21.3 m² | oldest |
| `driven_20261112` | 13.03% | 8.3 m² | driven, before yaw calibration |
| `buddy_map` | 12.50% | 18.0 m² | wander |
| `driven_cal_20261112` | 10.70% | 15.4 m² | driven, AFTER yaw calibration |
| `test_20261111_1943` | 18.36% | 12.4 m² | wander |
| `oakd_run_20261112` | 9.63% | 6.8 m² | wander; map froze at 24 s |
| `vslam_20261112` | **0%** | 7.4 m² | rtabmap, broken `Grid/FootprintHeight` |

Random wandering is a poor mapping strategy — it never revisits a place
deliberately, so loops do not close. Deliberate perimeter driving that returns
PAST the start is what closes them.

**2026-11-13 mapping fixes:** `throttle_scans` is now **1**. The installed
slam_toolbox 2.6.10 code throttles in its common callback even though the README
labels it synchronous-only. `minimum_travel_distance` is now **0.0** because
that version's outer gate checks translation alone and can discard pure turns
before Karto sees `minimum_travel_heading`. This also admits stationary scans:
long-run graph growth/memory is a tradeoff that has NOT been endurance tested.
A proper heading-aware gate/upstream update is the preferable future fix.

`fixed_resolution` is now **true** in the YDLidar config. The old false setting
produced 1337–1341 rays with changing angular increments; Karto rejected scans
with `LaserRangeScan contains ... expected ...`. The SDK picks ~1340 rays from
measured revolutions; this setting fixes the published ray layout. Small SDK
"Real point count > fixed point count" crop warnings can remain; these are
separate from Karto rejecting the entire scan. Final replay accepted **126 pose
updates from 132 input scans**, versus **5** in the earlier variable-count test
with the rotation gate already opened. These are different recordings, so the
counts demonstrate scan acceptance, not an isolated map-quality effect.

## Camera SLAM (rtabmap) — set up, not yet validated

`launch/buddy_vslam.launch.py` + `config/oakd_rtabmap.yaml`. Runs the depthai
driver and `rtabmap_slam` with the SAME wheel odometry slam_toolbox used
(`odom_frame_id: odom`), so a comparison changes only the sensor.

Verified working end to end: TF chain resolves
(`oak → BuddyCameraMount_link → … → odom → map`), depth is registered to RGB at
1280×720, and projecting the depth image into `BuddyBaseFootprint_link` puts
67.8% of points in the 0.08–1.2 m obstacle band — i.e. the geometry is right.

**The first driven run produced a grid with zero obstacles** because of the
`Grid/FootprintHeight` trap above. The grid params are now corrected but **the
corrected config has NOT been validated on hardware.**

The database from that drive is intact (`~/.ros/rtabmap.db`, 762 nodes).
Reprocessing it with corrected grid params found **4 loop closures** where the
live run accepted none:

```bash
rtabmap-reprocess --Grid/Sensor 1 --Grid/FootprintHeight 0 \
  --Grid/RayTracing true --Grid/MaxGroundHeight 0.10 \
  --Grid/MaxObstacleHeight 1.5 --Grid/NormalsSegmentation false \
  in.db out.db
```

Extracting a PGM from a reprocessed DB was in progress when the session ended —
`rtabmap-export` has no grid option, so the route is to run an rtabmap node
against the DB and call `/rtabmap/publish_map`, then `map_saver_cli`.

## State as of 2026-11-12

**Working and verified on hardware:** firmware flashed; 20 Hz encoder stream;
autodetect; `/odom` at 20 Hz; all six drivetrain sign cases; rotational
odometry calibrated to 2%; guard with `min_linear` and rear-coverage fail-closed;
`buddy_teleop.py` (latching, built because `teleop_twist_keyboard` publishes once
per keypress and the guard's 0.5 s watchdog zeroes it between taps); OAK-D
streaming RGB+depth+IMU; rtabmap running and processing. Test suite passes.

**Not done / known broken:**
1. **Superseded 2026-11-13:** the 2.5 m universal lidar limit was disproved by
   returns to 8.069 m. Resolve the measured scan-processing issues first.
2. **Camera SLAM not yet validated** with corrected grid params.
3. **Linear odometry ~18% out**, measured once, not yet corrected.
4. **Fixed 2026-11-13:** throttle is 1; fixed ray layout and rotation gate tested.
5. OAK-D BMI270 remains unused. The independent ESP MPU-6500 gyro now has a
   tested optional wheel/IMU fusion path; see the new hardware section.
6. **SLAM runs instead of localization** — bringup re-maps every session, so
   `maps/` goes unused. Switching to AMCL is the gap for "navigate the home".
7. PID — see the velocity-scale section. The owner has so far declined; the data
   favours it.
8. `python3.10-venv`, and the `~/.local/bin` PATH line (editing shell startup
   files is blocked by the permission classifier — hand it to the owner).

**Nothing has ever been pushed.** All work is local commits on `main`, and there
are uncommitted changes to the guard, scan utils, serial bridge, URDF, tests and
the lidar config, plus new untracked files (`buddy_teleop.py`,
`buddy_vslam.launch.py`, `oakd_rtabmap.yaml`, four map pairs).

## Lidar + ESP IMU findings — 2026-11-13

**Result: better short-turn heading estimates; no demonstrated map-quality gain
from the IMU alone.** Fusion remains selectable, default **false**, pending a
longer deliberate route/loop-closure comparison. Camera SLAM was left untouched.

Tested four initial in-place turns (+/-1.6 rad/s for 0.9/1.4 s), then two final
0.9 s turns after correcting wheel sides and lidar processing. All went through
`/cmd_vel_raw` and the existing guard, on a clear floor with owner supervision.
No commanded translation. The owner confirmed the original positive command
turned clockwise/right; final sensor signs agree after normalization.

First-run stationary-endpoint 2D lidar alignment (trimmed symmetric nearest-
neighbour fit, no gyro/encoder yaw prior) found:

| physical lidar yaw | normalized wheels | recorded IMU fusion | alignment RMS |
| --- | --- | --- | --- |
| -83.76 deg | -80.84 deg | -83.23 deg | 0.011 m |
| +84.59 deg | +82.46 deg | +83.86 deg | 0.046 m |
| -126.01 deg | -121.60 deg | -125.16 deg | 0.010 m |
| +125.92 deg | +122.10 deg | +124.93 deg | 0.008 m |

Mean absolute endpoint error was **3.32 deg wheels vs 0.77 deg recorded fusion**.
The wheel baseline above is sign-normalized after recording; this initial EKF
still received the old opposite-sign wheel rate. Do not confuse the large
sign correction with an IMU-only improvement. Its gyro-dominated estimate was
already much closer despite that input issue. The second fit is noisier than
other endpoints. Final-run endpoint fits had 0.062/0.151 m RMS and were rejected
as independent truth (do not use their angle errors as a calibration result).

Final fair map comparison replayed the SAME 35 s, 132-scan `run2` recording in
isolated domains 93/94, with corrected wheel signs and identical mapper settings.
Only wheel/IMU odometry differed; neither replay starts motor hardware.

| pipeline | accepted pose updates | occupied | free | explored | occ/(occ+free) |
| --- | --- | --- | --- | --- | --- |
| corrected wheels | 126 | 943 | 5841 | 16.96 m² | 13.90% |
| corrected wheels + IMU | 126 | 956 | 5616 | 16.43 m² | 14.55% |

Visual maps are similar; the IMU result is slightly worse on the historical
ratio. This short rotation-only dataset does NOT establish whole-home mapping,
loop closure, trajectory accuracy, or sustained CPU/memory performance.
Do not claim that IMU fusion has improved maps yet.

Artifacts in `maps/`: `lidar_wheels_20261113.{pgm,yaml}`,
`lidar_imu_20261113.{pgm,yaml}`, `imu_lidar_comparison_20261113.{png,json}`.
Raw bags, replay configs/logs and endpoint analysis:
`~/buddy_imu_test_20261113/`. Original dirty files and pre-IMU firmware binary:
`~/buddy_imu_backup_20261113/`. No previous edits were discarded or pushed.

Reproduce (source the ROS workspace first):

```bash
# Live recording physically MOVES the robot: require clear floor + observer.
ROS_DOMAIN_ID=42 python3 ros2_ws/src/buddy_description/test/record_imu_turns.py \
  /absolute/new/run --turns --short-turns
# Replay an existing run; no motors:
ROS_DOMAIN_ID=93 python3 ros2_ws/src/buddy_description/test/replay_imu_slam.py \
  /home/alex/buddy_imu_test_20261113/run2 wheel --output /absolute/new/wheel
ROS_DOMAIN_ID=94 python3 ros2_ws/src/buddy_description/test/replay_imu_slam.py \
  /home/alex/buddy_imu_test_20261113/run2 imu --output /absolute/new/imu
# First recording only: --swap-wheel-sides normalizes its old wheel convention.
python3 ros2_ws/src/buddy_description/test/analyze_imu_turns.py \
  /home/alex/buddy_imu_test_20261113/run1 --swap-wheel-sides
```

Validation: ESP firmware built/flashed with verified hashes; xacro/check_urdf
passed; live IMU TF checked; five protocol/calibration/clock unit tests passed;
real bridge vs fake ESP PTY test passed (command sign, encoder sign, calibrated
SI IMU messages); existing scan/guard/wanderer suite passed. All test drive
commands were zeroed; hardware and replay processes stopped, both serial ports
released at session end.

Primary implementation references:
[robot_localization sensor configuration](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/configuring_robot_localization.rst),
[sensor frames and TF ownership](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/preparing_sensor_data.rst),
[slam_toolbox common scan gate](https://github.com/SteveMacenski/slam_toolbox/blob/humble/src/slam_toolbox_common.cpp),
[YDLidar SDK fixed ray layout](https://github.com/YDLIDAR/YDLidar-SDK/blob/master/src/CYdLidar.cpp).

## Wheel-separation recalibration + autonomous mapping — 2026-11-13 (later)

**`WHEEL_SEPARATION` 0.4012 -> 0.3884.** Re-measured after the owner corrected
the physical track to 15". The original 0.4012 was derived as `0.3302 x 1.215`
using the WRONG geometric baseline (13"), so most of the "21% tyre scrub" was
really the bad track figure. At 0.3884 the implied scrub is ~2% over physical
0.381 m, which is believable for rubber-banded tyres.

### Slip depends on the TURN PROFILE, and slower is not gentler

Same calibration, same constant (0.3884), three turn profiles:

| profile | gain | spread | lidar fit RMS |
| ------- | ---- | ------ | ------------- |
| fast continuous, 1.6 rad/s 0.9-1.4 s | **1.002** | 0.987-1.005 | 0.076-0.119 |
| pulsed, long (hit the +/-180 wrap) | 1.014 | 0.965-1.037 | 0.110-0.181 |
| pulsed, short (cleanest fit) | 1.038 | 1.036-1.048 | **0.031-0.052** |

**Pulsing a turn to slow it down makes this drivetrain slip MORE, not less.**
Every burst is a fresh break-away from rest at PWM 150+, and break-away is
exactly where the tyres let go; a continuous turn breaks away once then rotates
steadily. Pulsed turns also overshoot badly (103-179 deg where continuous gave
80-131 deg) because the base coasts after the command stops.

0.3884 is tuned to the CONTINUOUS profile, because that is what the robot does
while mapping. The residual ~4% spread across profiles IS the slip dependence
and no single constant removes it - closing the velocity loop is the real fix.

### The lidar reaches 8 m — independently re-confirmed

40 s, 146 scans, 152,687 valid returns: **max 8.061 m**, per-scan max median
8.02 m, 4.08% of returns beyond the old "2.51 m ceiling". Every scan reaches
~8 m. The earlier 2.5 m claim was one sample taken with the robot boxed in a
2.55 m corridor facing a wall. **Do not replace this sensor.**

### Wall-following produced the WORST map yet — the lesson is turn count

`scratchpad` wall-follower: right-hand wall at 0.65 m, 0.18 m deadband. It
tracked the perimeter well and the odometry loop closed (35.06 m path, back to
0.57 m from start, passing within 0.25 m mid-run) - but the map was
`maps/wallfollow_20261113` at **29.54% occupied, the worst ratio recorded**,
walls smeared into solid blobs.

Cause: **167 separate turn events in 270 s** (51 corner turns, 31 nudges, 85
drift corrections). Each is a break-away slip event, per the table above. A
controller built on constant small stop-start corrections is the worst possible
motion profile for this base. Correcting RARELY beats correcting precisely:
widen the deadband and lengthen the straight legs. A retuned version (0.45 m
deadband, 2.2 s straights) was written but **interrupted before it ran**, so it
is untested.

Good maps on this robot so far have come from the owner driving deliberately,
not from autonomy.

### Hardware gaps found by driving it

**The lidar plane misses real obstacles.** The robot is taller than its lidar
(scan plane 0.406 m; camera/neck reach ~0.87 m), so it **drove under and into a
desk** whose surface was above the scan plane. Low obstacles below 0.4 m are
equally invisible. The OAK-D could cover both bands - project its depth image
into the base frame, keep points in roughly 0.05-0.95 m, and merge the nearest
per bearing into the guard alongside `/scan`. **NOT BUILT** - the owner wants a
better camera mount first. The camera's measured minimum range is ~0.35 m, so
it will not cover the very close band either.

**Cable wind-up is an operational hazard.** The long power leads twisted during
a perimeter run and the robot had to be picked up mid-run. Autonomous runs need
either a tether watch or an untethered power source.

**Closing interior doors matters.** The same run drove out through an open door.

### Artifacts

`maps/wallfollow_20261113.{pgm,yaml}` (the bad map, kept as the counter-example)
and the full recording at `/home/alex/buddy_map_20261113b/run1/bag`, replayable
with `test/replay_imu_slam.py` without touching hardware.

Earlier the same day, the owner's own 120 s teleop drive was recorded to
`/home/alex/buddy_map_20261113/run1/bag` and replayed through both pipelines:

| pipeline | poses | occupied | free | explored | occ/(occ+free) |
| -------- | ----- | -------- | ---- | -------- | -------------- |
| wheels | 433 | 2021 | 5927 | 19.87 m2 | 25.43% |
| wheels + IMU | 433 | 2024 | 5917 | 19.85 m2 | 25.49% |

A 0.06% difference is noise, and the maps are visually indistinguishable. That
is now **two independent tests** (35 s rotation-only, and this 120 s driven loop
with real translation) showing **no map-quality gain from IMU fusion**. Keep
`use_imu:=false`. Saved as `maps/driven_loop_{wheels,imu}_20261113`.

### Two more traps

**The `ros2` CLI daemon dies with `!rclpy.ok()`** and then `ros2 topic list` /
`node list` return nothing while the stack is perfectly healthy. A wait-loop
polling `ros2 topic list` will hang forever on a working robot. `ros2 daemon
stop && ros2 daemon start` may not fix it; check `pgrep` for the actual
processes, and prefer a plain rclpy script (which bypasses the daemon) to
decide whether topics are live.

**The `pkill` bracket trick only protects the PATTERN.** `pkill -f
'[b]uddy_serial_bridge'` still kills your own shell if the same command line
mentions `buddy_serial_bridge.py` anywhere else - e.g. the path you are about
to launch. Put the `pkill` in its own call, separate from anything naming the
target.

**A calibration tool can carry a stale convention.** `yaw_cal.py` hardcoded the
sign of the lidar scan-alignment shift. When the driver's ray layout was
corrected the sign flipped, and the tool reported a gain of **-0.97** - which
reads as "odometry turns backwards", a catastrophic-looking result from a
healthy robot. The gyro settled it (odom +92.5 vs gyro +94.3 deg; -92.3 vs
-95.3). The tool now compares magnitudes, reports the sign relationship
separately, and reads `WHEEL_SEPARATION` from the bridge source so it cannot
print a stale number. **When two sensors disagree about a sign, add a third.**

## Overhead obstacle detection from the OAK-D — 2026-11-13 (built)

The lidar is ONE horizontal slice at 0.406 m and the robot is ~0.9 m tall, so a
desk top above that slice is invisible to it - the robot drove into exactly that.
`buddy_depth_obstacles.py` closes the gap.

**Camera re-measured: 32" (0.8128 m) above the floor**, was 34". URDF joint
`CameraMount_to_neck` is now `xyz="0 0 0.53105"`; live TF reads
`[-0.102, 0.000, 0.813]` with rotation `(-90, 0, -90) deg`.

### How it works

Projects each depth pixel into the robot frame, keeps points inside a height
band that would actually collide, and reduces them to a nearest obstacle per
bearing. Publishes `/scan_camera` **in the BASE frame**, so each range is the
horizontal distance from the robot's own origin and `buddy_scan_guard` applies
the same sector maths as the lidar with no mounting yaw.

The guard takes **whichever sensor sees an obstacle nearer**, and reports
`front_blocked_camera` when the camera is the one that stopped it. Only the
FRONT sector is taken from the camera - its field of view is ~72 deg, so it must
never be allowed to imply the sides or rear are clear.

    ros2 launch buddy_description buddy_hardware.launch.py \
        mode:=teleop use_slam:=true use_camera:=true

`use_camera` defaults **false** (it needs the OAK-D plugged in and costs CPU),
and it also sets the guard's own `use_camera`, so with the camera off the guard
does not subscribe or warn.

### Verified on hardware

Projected points by height, with the robot facing a wall:

| band | points | nearest |
| ---- | ------ | ------- |
| 0.00-0.10 m (floor) | **0** | - |
| 0.10-0.35 m, lidar blind | 176 | 1.07 m |
| 0.35-0.46 m, the lidar's own plane | 832 | 0.87 m |
| **0.46-0.95 m, lidar blind** | **5428** | **0.83 m** |
| above 0.95 m, ignored | 3361 | 1.06 m |

Zero floor leakage, agreement with the lidar at its own plane, and the overhang
band is the largest contributor. Cross-checked against the lidar in the
overlapping front sector at +/-10/20/30 deg: agreement within **0.03-0.04 m**,
with the camera reading slightly FARTHER - floor leakage would read nearer, so
that rules it out. Runs at ~4.4 Hz using ~10k decimated pixels.

### Near-field speckle nearly froze the robot

With `min_depth` at 0.30 m, **13 of 67 scans reported an obstacle under 0.40 m
while the lidar saw 0.66 m**, and the bearing of the nearest return wandered
across the whole field of view - speckle, not structure. Any one of those would
have tripped the 0.35 m stop line and frozen the robot in an empty room. Fixed
two ways:

  * `min_depth` 0.30 -> **0.45 m**. Closer than that is the LIDAR's job.
  * **Temporal persistence**: a bin is reported only if occupied in 2 of the
    last 3 scans, and its range is the median of those.

After: **0 of 121 scans under 0.40 m**, spread tightened from std 0.19 to 0.066.

### Limits that must not be forgotten

  * ~72 deg field of view, not 360. A COMPLEMENT to the lidar, never a
    replacement.
  * Cannot see nearer than 0.45 m. `stop_distance` must stay inside what the
    lidar can see.
  * Obstacles under ~0.10 m are invisible to BOTH sensors. `min_height` is a
    floor-rejection threshold, and lowering it risks a floor misread as an
    obstacle, which would stop the robot dead in an empty room.
  * Persistence costs ~0.6 s of latency, about 17 cm at cruise.
  * `require_camera` defaults FALSE: a camera dropout logs a throttled warning
    and reports `camera_stale`, but does not stop the robot, because the lidar
    still guards the stop line. The cost is that with the camera down the robot
    is blind to desks again. Set it true for unsupervised runs near overhanging
    furniture.

### A trap worth repeating

A test suite failure after this work looked like a code bug - the wanderer
integration test reported zero commands. The cause was **leaked nodes from
manual debugging in domain 91**, which is the suite's own default domain. The
runner's comments warn about exactly this. Before believing a test failure,
check `pgrep` for stray `buddy_*` nodes.

## Working with the owner

Mechanical engineering student (CU Boulder, controls/robotics) targeting Bay Area
robotics startups; this doubles as portfolio work. Ordering is deliberate:
**navigation → person recognition → voice.** `scripts/voice/` already holds a
working pipeline — don't pull it forward.

They have hands on the hardware and I do not. When a question needs eyes (which
way a wheel turned, is an LED lit), **ask — don't infer.** Their observations
have repeatedly overturned a confident wrong conclusion of mine: the extrusion
detail exposed the inverted lidar yaw, "both wheels the same way, but backwards"
killed my single-inverted-motor theory, and "I forgot to plug one of the wheels
back in" explained two wasted mapping runs.

Where a sensor can answer instead, use it. Comparing odom against lidar-measured
rotation settled the wheel-slip question in two minutes without needing eyes,
and the same trick (align successive scans, take the best yaw shift) works for
any "did it really move?" question.

## Mapping calibration and fresh hardware route — 2026-11-13

Owner confirmed **15 inches (0.381 m) between wheel centre lines**, unchanged
since the recorded 120 s drive. URDF wheel y positions are now +/-0.1905 m;
Gazebo and firmware SOURCE use physical 0.381 m. Effective encoder odometry
stays 0.4012 m. The earlier 21%-scrub interpretation was wrong: the old
0.3302 m starting value combined incorrect geometry with slip. The empirical
width is 5.3% larger than the corrected physical track.

**Firmware deployment:** corrected source built successfully but was NOT
flashed. The fresh route deliberately used the existing tested ESP firmware
(command width 0.3302 m, with the previously flashed IMU support). Flashing the
new binary will change angular command-to-PWM conversion; retest turn speeds
then. No motor gains, encoder scale, default IMU choice or live mapper settings
were changed by these offline experiments.

### Recorded-route experiments

Input: `~/buddy_map_20261113/run1/bag`, 120 s and 448 scans. Wheel data reports
5.44 m path length, 0.516 m endpoint displacement and 580 deg accumulated
absolute yaw; 73% of wheel samples are stationary. Scan ray count is fixed at
1340; median scan time 0.268 s, longest return 8.94 m.

All cases use wheel odometry and identical recorded scans. The optional
translation/rotation gate is an OFFLINE scan selector interpolating wheel pose
at scan acquisition timestamps. It does not run as a live hardware node.

| case | accepted poses | explored m² | occupied fraction |
| --- | ---: | ---: | ---: |
| baseline, distance 0 | 433 | 19.87 | 25.43% |
| translation-only distance 0.2 m | 20 | 12.83 | 22.10% |
| translation 0.2 m OR heading 0.1 rad | 50 | 15.36 | 28.42% |
| translation 0.05 m OR heading 0.1 rad | 96 | 16.70 | 32.77% |
| previous + reject scans spanning >0.05 rad wheel yaw | 72 | 16.46 | 32.25% |
| baseline + experimental linear gain 0.92322 | 433 | 19.49 | 25.61% |

The 0.2 m case makes portions of the room look cleaner but discards most poses
and much mapped coverage. Rotation-aware sparse cases still have doubled walls;
the finer gate/fast-turn rejection did not improve them. **No tested change is
established as a better general mapping configuration.** Baseline settings
remain active. A lower occupied fraction alone is not grounds to select the
0.2 m case. The baseline replay exactly reproduced the previously supplied
433 / 2021 / 5927 figures.

Loop-closure evidence: the installed Karto library exports
`Mapper::FireEndLoopClosure`; an offline LD_PRELOAD observer logs that callback
and forwards it unchanged to the original implementation. Verified with a real
Mapper callback self-test and the library's PLT call site. Instrumented baseline,
0.05 m motion gate, fast-turn rejection and linear-scale replays all recorded
**zero completed loop closures**. Baseline with the observer reproduced identical
map counts. Other cases were not instrumented; their loop status is not inferred
from appearance or total edge count. Instrument source/binary/self-test are in
`~/buddy_map_20261113/instrumentation/`; ordinary launches are unaffected.

Straight-run distance fit: only two segments had usable stationary endpoint
scans and low-residual alignment. Wheel vs lidar distances were
**0.3593 vs 0.3289 m** (ratio 0.9154, trimmed RMS 0.0102 m) and
**0.6672 vs 0.6212 m** (ratio 0.9310, RMS 0.0174 m). A third fit was rejected
for poor residual/implausible heading. Their average 0.92322 was tested offline
on TRANSLATION ONLY, retaining yaw calibration; it did not improve map metrics
or produce a closure. **Do not apply it as a live wheel-diameter/ticks calibration**
from only these two environment-dependent fits. No independent ruler truth or
held-out route was available.

Replayer now supports `--travel-distance`, `--motion-gate-distance`,
`--motion-gate-heading`, `--max-scan-yaw`, `--linear-scale`, and saves selected
scan decisions/trajectories/configs for reproduction. Use isolated ROS domains;
no motor node is launched. Experiment maps and two comparison PNGs are in
`maps/mapping_cal_*_20261113.*` and `maps/mapping_calibration*_comparison.png`.
Full artifacts: `~/buddy_map_20261113/calibration_*`.

### Fresh supervised hardware mapping

Owner explicitly requested a hardware route and confirmed clear floor + observer.
First attempt stopped without moving: front clearance was only 0.50 m.
Second attempt turned toward the clearer left side, then adapted four forward
legs to available clearance, turning about 90 deg between legs, plus a final
short forward leg. All commands went through `/cmd_vel_raw` and the scan guard;
additional checks required fresh lidar/odom/gyro/guard data, front return
coverage, front clearance, progress, bounded time, and limited heading drift.
No reversing. Gyro controlled the turns; **SLAM itself remained wheel-only**.

Result: **route completed, 2.666 m wheel-reported path, 260 recorded lidar scans,
70 recorded map updates, zero observed completed loop closures**. Map artifact:
`maps/hardware_calibration_20261113.{pgm,yaml,png,json}`. The live mapper had
already observed the initial stationary period/aborted attempt, so its accumulated
graph is not limited to the 260 scans in the second recording. Do not compare
its occupancy ratio against offline maps as though they were the same route.
This was an adapted four-leg route, not a verified return to the exact start.

Requested vs settled forward-leg distances were 0.65->0.705, 0.49->0.540,
0.39->0.451, 0.49->0.544 and 0.20->0.257 m: approximately **5–6 cm additional
wheel travel after the controller's distance cutoff**. Settled turn errors
were about 1–6 deg. These controller stopping errors are separate from SLAM
loop closure and wheel-scale error; future path execution should account for
braking/coasting rather than assume exact geometric squares.

Hardware script: `test/run_mapping_route.py`; raw recordings, events and launch
log: `~/buddy_map_20261113/hardware_route{1,2}` and `hardware_launch.log`.
Robot stop commands sent, route finished, hardware launch group terminated.
URDF expanded/validated with measured 0.381 m centre spacing; firmware compiled;
existing effective odometry constant retained. No new firmware flashed.
