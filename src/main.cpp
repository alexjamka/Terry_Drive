#include <Arduino.h>

#define ENA 25
#define IN1 33
#define IN2 32
#define ENB 26
#define IN3 14
#define IN4 27
#define ENC_LA 18
#define ENC_LB 19
#define ENC_RA 4
#define ENC_RB 13

#define WHEEL_SEPARATION 0.3302
#define MAX_PWM 250

volatile long leftTicks = 0;
volatile long rightTicks = 0;

float leftVel = 0.0;
float rightVel = 0.0;
unsigned long lastCmdTime = 0;
unsigned long lastPubTime = 0;
String buf = "";

void IRAM_ATTR leftISR() {
  leftTicks += digitalRead(ENC_LB) ? 1 : -1;
}
void IRAM_ATTR rightISR() {
  rightTicks += digitalRead(ENC_RB) ? 1 : -1;
}

void drive(float left, float right) {
  int lp = (int)(left * 833.0);  // 250 / 0.3
  int rp = (int)(right * 833.0);
  lp = constrain(lp, -MAX_PWM, MAX_PWM);
  rp = constrain(rp, -MAX_PWM, MAX_PWM);

  if (lp > 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else if (lp < 0) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); }
  ledcWrite(0, abs(lp));

  if (rp > 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else if (rp < 0) { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); }
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); }
  ledcWrite(1, abs(rp));
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
      }
      buf = "";
    } else buf += c;
  }

  // Timeout
  if (millis() - lastCmdTime > 500) {
    leftVel = 0; rightVel = 0;
  }

  drive(leftVel, rightVel);

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