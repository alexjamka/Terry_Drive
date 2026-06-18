/*
 * Buddy Robot - ESP32 Motor Controller
 * 
 * Receives velocity commands from Jetson over UART2:
 *   "V:<linear_m/s>,<angular_rad/s>\n"
 * 
 * Sends encoder ticks back to Jetson over UART2 at 20Hz:
 *   "E:<left_ticks>,<right_ticks>\n"
 * 
 * Also accepts commands over USB Serial for debugging.
 * 
 * PID velocity control with slew rate limiting to prevent wheel spin.
 */

// ==================== PIN DEFINITIONS ====================
// Motor A (Left)
#define ENA 25
#define IN1 32
#define IN2 33

// Motor B (Right)
#define ENB 26
#define IN3 14
#define IN4 27

// Encoder A (Left motor)
#define ENC_LA 4
#define ENC_LB 13

// Encoder B (Right motor)
#define ENC_RA 18
#define ENC_RB 19

// UART2 to Jetson
#define JETSON_TX 17
#define JETSON_RX 16
#define JETSON_BAUD 115200

// ==================== ROBOT PARAMETERS ====================
#define WHEEL_SEPARATION 0.3302  // meters (2 * 0.1651)
#define WHEEL_DIAMETER   0.065   // meters
#define TICKS_PER_REV    5764    // 11 PPR * 131 gear ratio * 4 quadrature
#define MAX_SPEED        0.3     // m/s
#define MAX_PWM          250

// ==================== PWM CONFIG ====================
#define PWM_FREQ       5000
#define PWM_RESOLUTION 8

// ==================== PID PARAMETERS ====================
#define KP 2.5
#define KI 1.5
#define KD 0.01
#define PID_INTERVAL_MS 50  // 20Hz PID loop

// ==================== SLEW RATE LIMITER ====================
#define MAX_PWM_STEP 30  // Max PWM change per PID cycle

// ==================== TIMING ====================
#define CMD_TIMEOUT_MS      500
#define ENCODER_PUB_MS      50   // 20Hz encoder publishing

// ==================== GLOBAL STATE ====================

// Encoder tick counts (volatile for ISR access)
volatile long leftTicks = 0;
volatile long rightTicks = 0;

// PID state
float leftTargetVel = 0.0;   // m/s
float rightTargetVel = 0.0;  // m/s
float leftIntegral = 0.0;
float rightIntegral = 0.0;
float leftPrevError = 0.0;
float rightPrevError = 0.0;
long prevLeftTicks = 0;
long prevRightTicks = 0;
int prevLeftPWM = 0;
int prevRightPWM = 0;

// Timing
unsigned long lastCmdTime = 0;
unsigned long lastPIDTime = 0;
unsigned long lastEncoderPubTime = 0;

// Serial input buffer
String jetsonBuffer = "";
String usbBuffer = "";

// ==================== ENCODER ISRs ====================
void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(ENC_LB) == HIGH) {
    leftTicks++;
  } else {
    leftTicks--;
  }
}

void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(ENC_RB) == HIGH) {
    rightTicks++;
  } else {
    rightTicks--;
  }
}

// ==================== SETUP ====================
void setup() {
  // Debug serial (USB)
  Serial.begin(115200);
  Serial.println("Buddy Motor Controller Starting...");

  // UART2 to Jetson
  Serial2.begin(JETSON_BAUD, SERIAL_8N1, JETSON_RX, JETSON_TX);

  // Motor direction pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // PWM setup for motor speed
  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);

  // Encoder pins
  pinMode(ENC_LA, INPUT_PULLUP);
  pinMode(ENC_LB, INPUT_PULLUP);
  pinMode(ENC_RA, INPUT_PULLUP);
  pinMode(ENC_RB, INPUT_PULLUP);

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_LA), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RA), rightEncoderISR, RISING);

  // Motors off
  stopMotors();

  Serial.println("Ready. Waiting for commands...");
  Serial.println("Format: V:<linear_m/s>,<angular_rad/s>");

  lastPIDTime = millis();
  lastEncoderPubTime = millis();
  lastCmdTime = millis();
}

// ==================== MAIN LOOP ====================
void loop() {
  unsigned long now = millis();

  // Read commands from Jetson UART
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      processCommand(jetsonBuffer);
      jetsonBuffer = "";
    } else if (c != '\r') {
      jetsonBuffer += c;
    }
  }

  // Read commands from USB Serial (debug)
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      processCommand(usbBuffer);
      usbBuffer = "";
    } else if (c != '\r') {
      usbBuffer += c;
    }
  }

  // Safety timeout — stop if no command received
  if (now - lastCmdTime > CMD_TIMEOUT_MS) {
    leftTargetVel = 0.0;
    rightTargetVel = 0.0;
  }

  // PID loop at 20Hz
  if (now - lastPIDTime >= PID_INTERVAL_MS) {
    float dt = (now - lastPIDTime) / 1000.0;
    lastPIDTime = now;

    // Get current encoder ticks (atomic read)
    noInterrupts();
    long curLeft = leftTicks;
    long curRight = rightTicks;
    interrupts();

    // Compute actual wheel velocities (m/s)
    float metersPerTick = (WHEEL_DIAMETER * PI) / TICKS_PER_REV;
    float leftActualVel = ((curLeft - prevLeftTicks) * metersPerTick) / dt;
    float rightActualVel = ((curRight - prevRightTicks) * metersPerTick) / dt;
    prevLeftTicks = curLeft;
    prevRightTicks = curRight;

    // PID for left motor
    float leftError = leftTargetVel - leftActualVel;
    leftIntegral += leftError * dt;
    leftIntegral = constrain(leftIntegral, -100, 100);
    float leftDerivative = (leftError - leftPrevError) / dt;
    float leftOutput = KP * leftError + KI * leftIntegral + KD * leftDerivative;
    leftPrevError = leftError;

    // PID for right motor
    float rightError = rightTargetVel - rightActualVel;
    rightIntegral += rightError * dt;
    rightIntegral = constrain(rightIntegral, -100, 100);
    float rightDerivative = (rightError - rightPrevError) / dt;
    float rightOutput = KP * rightError + KI * rightIntegral + KD * rightDerivative;
    rightPrevError = rightError;

    // Convert PID output to PWM
    int leftPWM = (int)(leftOutput * (MAX_PWM / MAX_SPEED));
    int rightPWM = (int)(rightOutput * (MAX_PWM / MAX_SPEED));

    // Apply motors with slew rate limiting
    setMotorsPWM(leftPWM, rightPWM);
  }

  // Publish encoder ticks at 20Hz
  if (now - lastEncoderPubTime >= ENCODER_PUB_MS) {
    lastEncoderPubTime = now;
    publishEncoders();
  }
}

// ==================== COMMAND PROCESSING ====================
void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd.startsWith("V:")) {
    // Velocity command: V:<linear>,<angular>
    int commaIdx = cmd.indexOf(',', 2);
    if (commaIdx > 0) {
      float linear = cmd.substring(2, commaIdx).toFloat();
      float angular = cmd.substring(commaIdx + 1).toFloat();

      // Differential drive kinematics: convert (v, w) to wheel velocities
      leftTargetVel = linear - (angular * WHEEL_SEPARATION / 2.0);
      rightTargetVel = linear + (angular * WHEEL_SEPARATION / 2.0);

      // Clamp to max speed
      leftTargetVel = constrain(leftTargetVel, -MAX_SPEED, MAX_SPEED);
      rightTargetVel = constrain(rightTargetVel, -MAX_SPEED, MAX_SPEED);

      lastCmdTime = millis();

      Serial.print("CMD: L=");
      Serial.print(leftTargetVel, 3);
      Serial.print(" R=");
      Serial.println(rightTargetVel, 3);
    }
  } else if (cmd.startsWith("S")) {
    // Stop command
    stopMotors();
    Serial.println("STOPPED");
  }
}

// ==================== MOTOR CONTROL ====================
void setMotorsPWM(int leftPWM, int rightPWM) {
  // Slew rate limiting — prevent wheel spin
  if (leftPWM - prevLeftPWM > MAX_PWM_STEP) leftPWM = prevLeftPWM + MAX_PWM_STEP;
  if (prevLeftPWM - leftPWM > MAX_PWM_STEP) leftPWM = prevLeftPWM - MAX_PWM_STEP;
  if (rightPWM - prevRightPWM > MAX_PWM_STEP) rightPWM = prevRightPWM + MAX_PWM_STEP;
  if (prevRightPWM - rightPWM > MAX_PWM_STEP) rightPWM = prevRightPWM - MAX_PWM_STEP;

  prevLeftPWM = leftPWM;
  prevRightPWM = rightPWM;

  // Constrain to valid range
  leftPWM = constrain(leftPWM, -MAX_PWM, MAX_PWM);
  rightPWM = constrain(rightPWM, -MAX_PWM, MAX_PWM);

  // Left motor
  if (leftPWM > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(ENA, leftPWM);
  } else if (leftPWM < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    ledcWrite(ENA, -leftPWM);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    ledcWrite(ENA, 0);
  }

  // Right motor
  if (rightPWM > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    ledcWrite(ENB, rightPWM);
  } else if (rightPWM < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    ledcWrite(ENB, -rightPWM);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    ledcWrite(ENB, 0);
  }
}

void stopMotors() {
  leftTargetVel = 0.0;
  rightTargetVel = 0.0;
  leftIntegral = 0.0;
  rightIntegral = 0.0;
  leftPrevError = 0.0;
  rightPrevError = 0.0;
  prevLeftPWM = 0;
  prevRightPWM = 0;

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
}

// ==================== ENCODER PUBLISHING ====================
void publishEncoders() {
  noInterrupts();
  long l = leftTicks;
  long r = rightTicks;
  interrupts();

  // Send to Jetson over UART2
  String msg = "E:" + String(l) + "," + String(r) + "\n";
  Serial2.print(msg);

  // Also print to USB for debugging
  Serial.print(msg);
}
