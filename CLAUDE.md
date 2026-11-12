# CLAUDE.md — Buddy / Terry robot

Orientation for an agent picking this up cold. Full reference lives in
[`ros2_ws/src/buddy_description/README.md`](ros2_ws/src/buddy_description/README.md);
this file is the hardware truth and the traps.

**Everything below was measured on the robot, not inferred.** Where a number is
still uncalibrated it says so.

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
ros2 launch buddy_description buddy_hardware.launch.py mode:=wander
./src/buddy_description/test/run_tests.sh      # no hardware needed
```

`mode` = `idle` | `wander` | `follow` | `nav2`, mutually exclusive (each would
fight for `/cmd_vel`). `use_slam:=false` to skip slam_toolbox.

## Hardware facts (measured)

### Drivetrain — sign conventions were all wrong, now explicit flags

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
```

Verified: forward → both counts positive; reverse → both negative; each
rotation → opposite signs; each wheel isolates cleanly. **If the drivetrain is
rewired, re-derive one flag at a time — do not reason it out.** Encoder signs
alone cannot distinguish "motor reversed" from "encoder reversed"; both produce
identical data. Only watching the wheels resolves it.

### Break-away PWM — the dominant constraint

`drive()` maps velocity→PWM as `v * 833` (250 PWM at 0.30 m/s). Measured:

| PWM | result |
| --- | ------ |
| 41–83 | neither wheel moves |
| 99 | jitter only (±2 ticks) |
| 124 | **left** breaks away, right does not |
| 149 | **both** turn reliably |

`MIN_PWM = 150` floors any nonzero command. The two motors are different parts;
the right is stiffer. The owner considers this normal and does **not** want it
treated as a fault.

**Consequences that shape everything:**
- Minimum achievable speed is **0.18 m/s**. There is no slower range open-loop.
- A differential command whose **inner wheel** drops below that stalls the
  inner wheel. **Gentle arcs are impossible** — only straight runs and in-place
  turns. The wanderer's "curiosity" move is an in-place turn for this reason.
- `scan_guard`'s proportional slow zone **cannot actually slow the base** — the
  floor pushes it back up, so it behaves as go-or-stop. Kept because
  `stop_distance` is the real safety line.
- Break-away at 50–60% of max PWM is the strongest argument for closing the
  velocity loop (`archive/buddy_motor_controller.ino` has PI). A PI
  integrator winds up against a stalled wheel until it breaks free, buying back
  the low-speed range. The owner has so far declined PID; the data favours it.

### Lidar — mounting yaw is 0°, and getting it wrong mirrors everything

YDLidar X4. Spinning disc faces **forward**, motor faces **back**. A 40×40 mm
extrusion sits ~0.117 m behind the disc and shows up as a permanent return.

`laser_tf` publishes **`--yaw 0.0`**. It used to say `3.14159`, which mirrored
every avoidance decision: the rear extrusion was reported as a 0.12 m frontal
blocker, pinning the base inside `stop_distance` forever. Measured before/after,
same room:

| sector | yaw = π (wrong) | yaw = 0 (correct) |
| ------ | --------------- | ----------------- |
| front | 0.57 | **1.21** |
| left | 0.65 | 1.77 |
| right | 1.77 | 0.65 |
| rear | 2.82 | 0.57 (extrusion) |

`resolve_scan_yaw()` reads this from TF and logs it at startup. **That did not
protect us** — TF itself was wrong and the node applied it faithfully. To
recalibrate: put a known landmark at a known bearing, read `/scan`, solve for
the offset. A unit test pins the old wrong value as a regression.

Other lidar facts: `min_valid_range = 0.20` rejects the extrusion self-return.
The X4 reports **`0.0` for "no return"**, not `inf`. ~211 of 720 rays return
anything. Scan rate is **3.66 Hz** against `frequency: 10.0` — unexplained,
possibly motor speed; owner says it may rise once a drive program runs.

### Serial devices — both are CP2102 with identical serial numbers

The ESP32 and the lidar both enumerate as `10c4:ea60` reporting serial `0001`,
so **`/dev/serial/by-id` collides** and `ttyUSB*` order is unstable. The path
moved three times in one session.

`buddy_serial_bridge.py` therefore **autodetects**: it probes each port for
`E:<l>,<r>` frames and takes the one that answers, skipping ports another
process holds. Set the `port` parameter to override. For flashing, check
`ls -l /dev/serial/by-path/` and pass `--upload-port`.

The ESP32 is on **USB**, not the 40-pin UART. `buddy_serial_bridge.py` once
hardcoded `/dev/ttyTHS1`, which nothing is wired to — it logged one error and
then silently dropped every `/cmd_vel`.

## Traps that cost real time

**ESP32 power contention — most important.** Feeding the driver's 5 V into the
ESP32 while USB is also connected browns out the 3.3 V rail. Symptoms: port
enumerates, chip never answers the ROM bootloader, output flips between a `0x00`
flood and silence. **Two boards were wrongly declared dead this way.** A third
behaved identically until the external 5 V was unplugged, then booted cleanly.
Power from USB **or** the breadboard, never both. A power LED runs off USB VBUS
and lights even when the 3.3 V regulator is dead.

**All-zero serial bytes mean the RX line is held low**, not a device talking. An
idle UART sits high.

**Leftover ROS processes corrupt serial.** `ros2 launch` children survive killing
the launch, and **two readers on one tty garbles the stream for both** — it
presents as `Check Sum` errors from the lidar and nonsense from the ESP32. Once,
three full launches were running at once and the resulting mess was misread as a
hardware fault. Always:

```bash
pkill -f 'ydlidar|buddy_|foxglove_bridge|robot_state_publisher|static_transform_publisher|ros2 launch'
fuser /dev/ttyUSB0 /dev/ttyUSB1     # must be empty
```

**`pkill -f 'a\|b'` matches nothing** — it takes an ERE, so use `'a|b'`. Several
"cleanups" silently did nothing because of this. Also `pkill` returning non-zero
aborts a `&&` chain, so append `|| true`.

**Sensor QoS.** Lidar drivers publish `/scan` **BEST_EFFORT**. Subscribing with
the default RELIABLE profile is QoS-incompatible and ROS delivers **nothing**,
silently — the guard just reports `no_scan`. Use `qos_profile_sensor_data`. The
synthetic tests originally missed this because the test publisher was wrong the
same way; they now publish BEST_EFFORT deliberately.

**`--symlink-install` does not apply `install(PROGRAMS)` permissions.** A new
script written 644 is not runnable via `ros2 run` → "No executable found".
`chmod +x` new scripts.

**`resolve_scan_yaw()` spins the node** while waiting for TF, which can dispatch
`_on_scan` before `scan_yaw` exists. Both nodes now define it before creating
subscriptions and no-op until it is known.

**PlatformIO:** `python3.10-venv` is **not installed**, so `ensurepip` is missing
and the PlatformIO IDE extension cannot build `~/.platformio/penv` — that is the
"Python error". The CLI works regardless (pip-installed into `~/.local`). Needs
`sudo apt install python3.10-venv`. Also `~/.local/bin` is added only by
`.profile` (login shells), so `ssh host 'pio ...'` and VSCode's integrated
terminal get "command not found"; `python3 -m platformio` works anywhere.

## State as of 2026-11-11

**Working and verified on hardware:** firmware flashed and hash-verified; clean
single boot; 20 Hz encoder stream; autodetect; `/odom` at exactly 20.000 Hz with
`odom→base` TF; all six drivetrain sign cases; `/cmd_vel` 0.22 m/s for 3 s →
`/odom` x 0.0→0.573 m; `mode:=wander` running live off the real lidar with the
state machine driving `/cmd_vel` through the guard. Test suite passes.

**Not yet done:**
1. **Wander on the floor.** Everything so far was bench-tested with wheels
   elevated. Never verified driving freely.
2. **Odometry scale ~13% out** — 3 s at 0.22 m/s should be 0.66 m, measured
   0.573. `TICKS_PER_REV = 526` contradicts its own comment
   (`11 PPR × 131 × 4 = 5764`); the ISR counts one edge of one channel, so
   ~1441/rev is more likely. Calibrate by rolling a measured distance and
   counting ticks. (An earlier "2× wrong" estimate was wrong — deceleration
   inside the measurement window skewed it.)
3. **SLAM runs instead of localization.** Bringup runs slam_toolbox online-async,
   re-mapping every session, so `maps/home_20260418` goes unused. Switching to
   AMCL against it is the main gap for "navigate the home autonomously".
4. **PID** — see break-away above.
5. `python3.10-venv`, and the `~/.local/bin` PATH line in `.bashrc` (editing
   shell startup files is blocked by the permission classifier — hand it to the
   owner).

**Nothing has ever been pushed.** All work is local commits on `main`.

## Working with the owner

Mechanical engineering student (CU Boulder, controls/robotics) targeting Bay Area
robotics startups; this doubles as portfolio work. Ordering is deliberate:
**navigation → person recognition → voice.** `scripts/voice/` already holds a
working pipeline — don't pull it forward.

They have hands on the hardware and I do not. When a question needs eyes (which
way a wheel turned, is an LED lit), **ask — don't infer.** Their observations
have twice overturned a confident wrong conclusion of mine: the extrusion
detail exposed the inverted lidar yaw, and "both wheels the same way, but
backwards" killed my single-inverted-motor theory.
