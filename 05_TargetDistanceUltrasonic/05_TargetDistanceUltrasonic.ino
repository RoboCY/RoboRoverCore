/*
  ============================================================
  Project: Ultrasonic Distance Control
  Board: Kypruino UNO+ v0.8
  Platform: RoboRover Core
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
      APHASE = P0, BPHASE = P1 (I2C expander @ 0x20)

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
#define ECHO_TIMEOUT_US 30000UL // 30000 Unsigned Long [μs]


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

  // All HIGH by default
  expWrite(0xFF);
}

void loop() {
  // -------- Read ultrasonic distance (cm) --------
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long us = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);  // [μs]

  // If timeout, stop for safety
  if (us == 0) {
    analogWrite(PIN_AENBL, 0);
    analogWrite(PIN_BENBL, 0);
    delay(LOOP_DELAY_MS);
    return;
  }

  float distanceCm = us / 58.0;

  // -------- Distance control --------
  if (distanceCm > (TARGET_CM + DEADBAND_CM)) {
    // Too far -> move FORWARD
    // FORWARD = P0=0, P1=0
    expWrite(expState & 0b11111100); // AND operator (&) - Leaves bits 7-2 unchanged, makes bits 1-0 (P1, P0) LOW (0)
    if (distanceCm > (TARGET_CM + (2*DEADBAND_CM))) {
      analogWrite(PIN_AENBL, FWD_SPEED);
      analogWrite(PIN_BENBL, FWD_SPEED);
    } else {
      analogWrite(PIN_AENBL, FWD_SLOW);
      analogWrite(PIN_BENBL, FWD_SLOW);
    }
    
  }
  else if (distanceCm < (TARGET_CM - DEADBAND_CM)) {
    // Too close -> move BACKWARD
    // YOUR BACKWARD = P0=1, P1=1
    expWrite(expState | 0b00000011);  //  OR operator (|) - Leaves bits 7-2 unchanged, makes bits 1-0 (P1, P0) HIGH (1)
    if (distanceCm < (TARGET_CM - (2*DEADBAND_CM))) {
      analogWrite(PIN_AENBL, BACK_SPEED);
      analogWrite(PIN_BENBL, BACK_SPEED);
    } else {
      analogWrite(PIN_AENBL, BACK_SLOW);
      analogWrite(PIN_BENBL, BACK_SLOW);
    }
    
  }
  else {
    // Close enough -> stop
    analogWrite(PIN_AENBL, 0);
    analogWrite(PIN_BENBL, 0);
  }

  delay(LOOP_DELAY_MS);
}


// -------------------- EXPANDER WRITE --------------------
void expWrite(uint8_t v) {  // Sending to expander using I2C protocol
  Wire.beginTransmission(EXP_ADDR);
  Wire.write(v);
  Wire.endTransmission();
  expState = v;
}