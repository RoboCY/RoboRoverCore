/*
  ============================================================
  Project: Ultrasonic Distance Control
  Board: Kypruino UNO+ v0.8
  Platform: RoboRover Core 2
  ============================================================

  Description:
  The robot maintains a target distance from an obstacle using
  the ultrasonic sensor. Move the obstacle closer/further and
  the robot will move backward/forward to keep the distance.

  Behaviour:
  - If distance > target + deadband  -> move FORWARD
  - If distance < target - deadband  -> move BACKWARD
  - Otherwise                        -> stop

  Hardware:
  - Ultrasonic: TRIG D4, ECHO D5
  - Motors: DRV8836 MODE=1 (PHASE/ENABLE)
      AENBL = D11 (PWM), BENBL = D10 (PWM)
      M1_PHASE = P0, M2_PHASE = P1 via PCF8574 @ 0x20

  Wiring:
    FORWARD  = P0=0, P1=0
    BACKWARD = P0=1, P1=1
  ============================================================
*/

#include <Wire.h>

// -------------------- ULTRASONIC PINS --------------------
#define PIN_TRIG 4
#define PIN_ECHO 5

// -------------------- MOTOR PINS --------------------
#define PIN_AENBL 11
#define PIN_BENBL 10

// -------------------- I2C EXPANDER --------------------
#define EXP_ADDR 0x20

uint8_t expState = 0xFF;

// -------------------- SETTINGS --------------------
#define TARGET_CM       10.0
#define DEADBAND_CM      1.0

#define FWD_SPEED       95
#define BACK_SPEED      95
#define FWD_SLOW        70
#define BACK_SLOW       70

#define LOOP_DELAY_MS   40
#define ECHO_TIMEOUT_US 30000UL

void setup() {
  // Ultrasonic
  pinMode(PIN_TRIG, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);

  // Motors
  pinMode(PIN_AENBL, OUTPUT);
  pinMode(PIN_BENBL, OUTPUT);
  analogWrite(PIN_AENBL, 0);
  analogWrite(PIN_BENBL, 0);

  Wire.begin();

  // Default all HIGH
  expWrite(0xFF);
  stopMotors();
}

void loop() {
  float distanceCm = readDistanceCm();

  // If timeout / invalid reading, stop for safety
  if (distanceCm < 0) {
    stopMotors();
    delay(LOOP_DELAY_MS);
    return;
  }

  // -------- Distance control --------
  if (distanceCm > (TARGET_CM + DEADBAND_CM)) {
    // Too far -> move FORWARD
    setPhaseBits(0, 0);

    if (distanceCm > (TARGET_CM + (2 * DEADBAND_CM))) {
      setMotorSpeed(FWD_SPEED, FWD_SPEED);
    } else {
      setMotorSpeed(FWD_SLOW, FWD_SLOW);
    }
  }
  else if (distanceCm < (TARGET_CM - DEADBAND_CM)) {
    // Too close -> move BACKWARD
    setPhaseBits(1, 1);

    if (distanceCm < (TARGET_CM - (2 * DEADBAND_CM))) {
      setMotorSpeed(BACK_SPEED, BACK_SPEED);
    } else {
      setMotorSpeed(BACK_SLOW, BACK_SLOW);
    }
  }
  else {
    // Close enough -> stop
    stopMotors();
  }

  delay(LOOP_DELAY_MS);
}

// -------------------- ULTRASONIC READ --------------------
float readDistanceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long us = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);

  if (us == 0) return -1.0;
  return us / 58.0;
}

// -------------------- MOTOR HELPERS --------------------
void setMotorSpeed(uint8_t a, uint8_t b) {
  analogWrite(PIN_AENBL, a);
  analogWrite(PIN_BENBL, b);
}

void stopMotors() {
  analogWrite(PIN_AENBL, 0);
  analogWrite(PIN_BENBL, 0);
}

// Set only P0/P1, keep the rest of the expander state unchanged
void setPhaseBits(bool p0, bool p1) {
  uint8_t v = expState;

  v &= 0b11111100;  // clear P0/P1

  if (p0) v |= 0b00000001;
  if (p1) v |= 0b00000010;

  expWrite(v);
}

// -------------------- EXPANDER WRITE --------------------
void expWrite(uint8_t v) {
  Wire.beginTransmission(EXP_ADDR);
  Wire.write(v);
  Wire.endTransmission();
  expState = v;
}