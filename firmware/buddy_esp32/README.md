# Buddy — ESP32 Motor Controller Firmware

Low-level motor control for the Buddy robot. Talks to the Jetson over UART and
closes the loop on wheel encoders.

## Serial protocol

Shared by both firmware variants below. 115200 baud, newline-terminated ASCII,
carried over **USB** — there is no UART link to the Jetson's 40-pin header.
The firmware also mirrors everything onto `Serial2` (GPIO 16/17), which is
currently unconnected and harmless.

| Direction        | Message                          | Meaning                          |
| ---------------- | -------------------------------- | -------------------------------- |
| Jetson → ESP32   | `V:<linear_m/s>,<angular_rad/s>` | Velocity command                 |
| ESP32 → Jetson   | `E:<left_ticks>,<right_ticks>`   | Encoder counts, published @ 20 Hz |

The Jetson side of this link is `buddy_serial_bridge.py` in the
`buddy_description` ROS 2 package.

## ⚠️ Two firmware variants — unresolved

There are currently **two** implementations in this repo, and the newer one is
*not* strictly better. Resolve this before doing serious Nav2 work.

### `src/main.cpp` — PlatformIO, Apr 12 (currently the "active" project)

- 114 lines
- **No PID.** Velocity commands are applied without closed-loop correction.
- Bridges both USB `Serial` and `Serial2` (pins 16/17).
- Pin map: `ENA 25, IN1 33, IN2 32, ENB 26, IN3 14, IN4 27`

### `archive/buddy_motor_controller.ino` — Arduino IDE, Mar 26

- 339 lines
- **PID velocity control with slew-rate limiting** to prevent wheel spin.
- Documented protocol, USB debug commands.
- Pin map: `ENA 25, IN1 32, IN2 33, ...` — **`IN1`/`IN2` are swapped** relative
  to `main.cpp`, which inverts left-motor direction. One of the two is wrong for
  the current wiring.

**Why this matters:** Nav2's local planner assumes the base tracks commanded
velocities reasonably well. Open-loop PWM on a differential drive will not do
that under carpet/threshold load changes, and shows up as drift and overshoot at
goal. The PID version is very likely the one worth carrying forward — port it
into `src/main.cpp`, confirm the correct pin order against the physical wiring,
and delete the loser.

## Build & flash

```bash
pio run -t upload          # PlatformIO variant (src/main.cpp)
pio device monitor         # 115200 baud
```

The `archive/` .ino builds with arduino-cli or the Arduino IDE, not PlatformIO.

## Note on `platformio.ini`

`lib_deps` still pulls `micro_ros_arduino` and `build_flags` links `-lmicroros`,
but `src/main.cpp` uses plain `Serial` framing and includes no micro-ROS headers.
That dependency looks vestigial from an abandoned micro-ROS approach — it costs
build time and ~200 MB in `.pio/`. Verify, then drop it.
