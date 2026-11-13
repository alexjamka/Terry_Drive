#include <Arduino.h>
#include <Wire.h>

#define ENA 25
#define IN1 33
#define IN2 32
#define ENB 26
#define IN3 14
#define IN4 27

// Encoder pins, corrected 2026-11-11. These were transposed: driving the LEFT
// wheel alone moved the counter labelled "right" and vice versa, so odometry
// was attributing each wheel's motion to the other one. Measured by commanding
// one wheel at a time and watching which counter changed.
#define ENC_LA 4
#define ENC_LB 13
#define ENC_RA 18
#define ENC_RB 19

// Sign conventions, all measured on the bench rather than assumed.
//
// Target: a positive linear command drives the robot FORWARD, and both encoder
// counts increase. Before these flags, a forward command drove the robot
// BACKWARD with both wheels consistent, so both motors needed inverting; and
// with the encoder pins corrected, the left wheel counts down when rolling
// forward, so that one needs inverting too.
//
// If a drivetrain is rewired, re-derive these the same way: command one wheel,
// watch which counter moves and in which direction.
#define INVERT_LEFT_MOTOR    true
#define INVERT_RIGHT_MOTOR   true
#define INVERT_LEFT_ENCODER  true
#define INVERT_RIGHT_ENCODER false

// Physical track: owner confirmed 15 inches centre-to-centre, 2026-11-13.
// This command-kinematics value is distinct from effective ROS odometry width.
#define WHEEL_SEPARATION 0.381
#define MAX_PWM 250

// Below MIN_PWM the geared motors stall and buzz instead of turning, so any
// nonzero command smaller than this gets bumped up to it. Commands under
// MIN_VEL are treated as a genuine stop.
//
// Measured on the bench 2026-11-11 by ramping velocity and watching encoders:
//   PWM  41- 83  neither wheel moves
//   PWM  99       jitter only (+/-2 ticks)
//   PWM 124       LEFT breaks away, right does not
//   PWM 149       BOTH wheels turn reliably
// The two motors differ slightly; 150 covers the stiffer (right) one.
//
// Consequence worth knowing: minimum achievable speed is 150/833 = 0.18 m/s.
// There is no usable slower range open-loop, so a differential command whose
// INNER wheel falls below that will stall the inner wheel - gentle arcs are
// not achievable, only straight runs and in-place turns. Closing the loop
// (PI, as in archive/buddy_motor_controller.ino) is what would buy back the
// low-speed range, because the integrator winds up until the wheel breaks free.
#define MIN_PWM 150
#define MIN_VEL 0.005

volatile long leftTicks = 0;
volatile long rightTicks = 0;

float leftVel = 0.0;
float rightVel = 0.0;
// Last echoed command, so we only report changes (the host resends at 10 Hz
// to feed the watchdog; echoing every resend would flood the link).
float lastEchoL = 1e9;
float lastEchoR = 1e9;
unsigned long lastCmdTime = 0;
unsigned long lastPubTime = 0;
String buf = "";

void IRAM_ATTR leftISR() {
  int d = digitalRead(ENC_LB) ? 1 : -1;
  leftTicks += INVERT_LEFT_ENCODER ? -d : d;
}
void IRAM_ATTR rightISR() {
  int d = digitalRead(ENC_RB) ? 1 : -1;
  rightTicks += INVERT_RIGHT_ENCODER ? -d : d;
}

int velToPwm(float v) {
  if (fabs(v) < MIN_VEL) return 0;
  int p = (int)(v * 833.0);  // 250 / 0.3
  if (abs(p) < MIN_PWM) p = (v > 0) ? MIN_PWM : -MIN_PWM;
  return constrain(p, -MAX_PWM, MAX_PWM);
}

// Confirms a command was parsed, independent of whether the motors have
// power. Without this you cannot tell "command never arrived" from "motor
// rail is off" during bench bring-up.
void echoCmd() {
  if (fabs(leftVel - lastEchoL) < 1e-4 && fabs(rightVel - lastEchoR) < 1e-4) return;
  lastEchoL = leftVel; lastEchoR = rightVel;
  Serial.print("CMD:"); Serial.print(leftVel, 4);
  Serial.print(","); Serial.println(rightVel, 4);
}

void drive(float left, float right) {
  int lp = velToPwm(left);
  int rp = velToPwm(right);
  if (INVERT_LEFT_MOTOR)  lp = -lp;
  if (INVERT_RIGHT_MOTOR) rp = -rp;

  if (lp > 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else if (lp < 0) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); }
  ledcWrite(0, abs(lp));

  if (rp > 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else if (rp < 0) { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); }
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); }
  ledcWrite(1, abs(rp));
}

// MPU-9250: SDA=21, SCL=22, AD0=GND (0x68). Sensor axes are
// robot +X forward, +Y left, +Z up. Raw samples preserve the device clock;
// the ROS bridge converts to SI and estimates gyro bias while stationary.
constexpr uint8_t IMU_ADDR = 0x68;
bool imuReady = false;
unsigned long lastImuTime = 0;

bool imuRead(uint8_t reg, uint8_t *data, size_t n) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(IMU_ADDR, n) != n) return false;
  for (size_t i = 0; i < n; ++i) data[i] = Wire.read();
  return true;
}

bool imuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg); Wire.write(value);
  return Wire.endTransmission() == 0;
}

void initImu() {
  Wire.begin(21, 22, 400000);
  Wire.setTimeOut(10);  // disconnected IMU must not block motor watchdog
  uint8_t who = 0;
  if (!imuRead(0x75, &who, 1)) {
    Serial.println("IMU:NOT_FOUND at 0x68");
    return;
  }
  Serial.print("IMU:WHO_AM_I=0x"); Serial.println(who, HEX);
  // MPU-9250 / 9255 / 6500 share this accel/gyro register layout.
  // A 6500 identity does not establish that a magnetometer is present.
  if (who != 0x71 && who != 0x73 && who != 0x70) {
    Serial.println("IMU:UNSUPPORTED identity");
    return;
  }
  if (!imuWrite(0x6B, 0x80)) return;  // reset
  delay(100);
  imuReady = imuWrite(0x6B, 0x01) &&  // PLL, awake
             imuWrite(0x6C, 0x00) &&  // all accel/gyro axes enabled
             imuWrite(0x1A, 0x03) &&  // gyro DLPF: 41 Hz
             imuWrite(0x19, 19) &&    // 1 kHz / 20 = 50 Hz
             imuWrite(0x1B, 0x00) &&  // gyro +/-250 deg/s, 131 LSB/(deg/s)
             imuWrite(0x1C, 0x00) &&  // accel +/-2 g, 16384 LSB/g
             imuWrite(0x1D, 0x03);    // accel DLPF: 41 Hz
  Serial.println(imuReady ? "IMU:READY 50Hz" : "IMU:CONFIG_FAILED");
}

void publishImu() {
  if (!imuReady || millis() - lastImuTime < 5) return;
  lastImuTime = millis();
  uint8_t status = 0, data[14];
  if (!imuRead(0x3A, &status, 1) || !(status & 1)) return;
  if (!imuRead(0x3B, data, sizeof(data))) return;
  const uint32_t stamp = micros();
  // Skip the temperature word at byte 6. USB only: the ROS host uses USB.
  Serial.print("I:"); Serial.print(stamp);
  for (int offset : {0, 2, 4, 8, 10, 12}) {
    int16_t value = (int16_t)((uint16_t(data[offset]) << 8) | data[offset + 1]);
    Serial.print(','); Serial.print(value);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  ledcSetup(0, 5000, 8); ledcAttachPin(ENA, 0);
  ledcSetup(1, 5000, 8); ledcAttachPin(ENB, 1);

  pinMode(ENC_LA, INPUT_PULLUP); pinMode(ENC_LB, INPUT_PULLUP);
  pinMode(ENC_RA, INPUT_PULLUP); pinMode(ENC_RB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_LA), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RA), rightISR, RISING);

  drive(0, 0);
  initImu();
  Serial.println("Ready");
}

void loop() {
  // Read from both USB and UART
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (buf.startsWith("V:")) {
        int ci = buf.indexOf(',', 2);
        float lin = buf.substring(2, ci).toFloat();
        float ang = buf.substring(ci + 1).toFloat();
        leftVel = lin - (ang * WHEEL_SEPARATION / 2.0);
        rightVel = lin + (ang * WHEEL_SEPARATION / 2.0);
        lastCmdTime = millis();
        echoCmd();
      }
      buf = "";
    } else buf += c;
  }
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      if (buf.startsWith("V:")) {
        int ci = buf.indexOf(',', 2);
        float lin = buf.substring(2, ci).toFloat();
        float ang = buf.substring(ci + 1).toFloat();
        leftVel = lin - (ang * WHEEL_SEPARATION / 2.0);
        rightVel = lin + (ang * WHEEL_SEPARATION / 2.0);
        lastCmdTime = millis();
        echoCmd();
      }
      buf = "";
    } else buf += c;
  }

  // Timeout
  if (millis() - lastCmdTime > 500) {
    leftVel = 0; rightVel = 0;
  }

  drive(leftVel, rightVel);

  publishImu();

  // Publish encoders at 20Hz
  if (millis() - lastPubTime >= 50) {
    lastPubTime = millis();
    noInterrupts();
    long l = leftTicks, r = rightTicks;
    interrupts();
    String msg = "E:" + String(l) + "," + String(r) + "\n";
    Serial.print(msg);
    Serial2.print(msg);
  }
}