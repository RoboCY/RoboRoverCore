/*
  ============================================================
  Project: Obstacle Avoidance (IR)
  Board: Kypruino UNO+ v0.8
  Platform: RoboRover Core 2
  ============================================================

  Description:
  The robot moves forward slowly until an obstacle is detected
  by the IR obstacle sensors.

  Updated Behaviour (LEFT priority):
    - If LEFT sensor detects obstacle:
        1) Reverse briefly
        2) Spin RIGHT
        3) Continue forward
    - Else if RIGHT sensor detects obstacle:
        1) Reverse briefly
        2) Spin LEFT
        3) Continue forward
    - Else -> move forward

  Hardware:
  - Motors: DRV8836 MODE=1 (PHASE/ENABLE)
      ENABLE (speed):  AENBL = D11, BENBL = D10 (PWM)
      PHASE  (dir):    APHASE = P0, BPHASE = P1 (I2C expander)
  - Obstacle IR sensors (digital, via expander):
      LEFT  = P2
      RIGHT = P3
  - I2C expander address: 0x20

  Direction mapping:
    FORWARD  = P0=0, P1=0
    BACKWARD = P0=1, P1=1

  Sensor logic:
    REVERSED obstacle output:
      0 = obstacle detected
      1 = clear
  ============================================================
*/

#include <Wire.h>

// -------------------- MOTOR PINS --------------------
#define PIN_AENBL 11
#define PIN_BENBL 10

// -------------------- SPEED / TIMING --------------------
#define FWD_SPEED   110
#define REV_SPEED   95
#define SPIN_SPEED  140

#define REV_MS      250
#define SPIN_MS     550
#define PAUSE_MS    120

// -------------------- I2C EXPANDER --------------------
#define EXP_ADDR 0x20

#define P_APHASE 0
#define P_BPHASE 1
#define P_OIR_L  2
#define P_OIR_R  3

// Cached expander state (keep input bits HIGH)
uint8_t expState = 0xFF;

// -------------------- EXPANDER I/O --------------------
void expWrite(uint8_t v) {
  Wire.beginTransmission(EXP_ADDR);
  Wire.write(v);
  Wire.endTransmission();
  expState = v;
}

uint8_t expRead() {
  Wire.requestFrom(EXP_ADDR, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  } else {
    return 0xFF;
  }
}

void setup() {
  pinMode(PIN_AENBL, OUTPUT);
  pinMode(PIN_BENBL, OUTPUT);

  analogWrite(PIN_AENBL, 0);
  analogWrite(PIN_BENBL, 0);

  Wire.begin();

  // All HIGH at start so P2/P3 can be read as inputs
  expWrite(0xFF);
}

void loop() {
  // Keep sensor bits HIGH (input mode) before reading
  expWrite(expState | (1 << P_OIR_L) | (1 << P_OIR_R));
  uint8_t v = expRead();

  // REVERSED obstacle detection:
  // 0 = obstacle detected, 1 = clear
  int leftObstacle  = (bitRead(v, P_OIR_L) == 0);
  int rightObstacle = (bitRead(v, P_OIR_R) == 0);

  if (leftObstacle) {
    // -------- 1) Reverse briefly --------
    expWrite(expState | 0b00000011);  // BACKWARD = P0=1, P1=1
    analogWrite(PIN_AENBL, REV_SPEED);
    analogWrite(PIN_BENBL, REV_SPEED);
    delay(REV_MS);

    // Stop a moment
    analogWrite(PIN_AENBL, 0);
    analogWrite(PIN_BENBL, 0);
    delay(PAUSE_MS);

    // -------- 2) Spin RIGHT --------
    // Spin RIGHT = P0=1, P1=0
    expWrite((expState & 0b11111100) | 0b00000001);
    analogWrite(PIN_AENBL, SPIN_SPEED);
    analogWrite(PIN_BENBL, SPIN_SPEED);
    delay(SPIN_MS);

    // Stop a moment
    analogWrite(PIN_AENBL, 0);
    analogWrite(PIN_BENBL, 0);
    delay(PAUSE_MS);
  }
  else if (rightObstacle) {
    // -------- 1) Reverse briefly --------
    expWrite(expState | 0b00000011);  // BACKWARD = P0=1, P1=1
    analogWrite(PIN_AENBL, REV_SPEED);
    analogWrite(PIN_BENBL, REV_SPEED);
    delay(REV_MS);

    // Stop a moment
    analogWrite(PIN_AENBL, 0);
    analogWrite(PIN_BENBL, 0);
    delay(PAUSE_MS);

    // -------- 2) Spin LEFT --------
    // Spin LEFT = P0=0, P1=1
    expWrite((expState & 0b11111100) | 0b00000010);
    analogWrite(PIN_AENBL, SPIN_SPEED);
    analogWrite(PIN_BENBL, SPIN_SPEED);
    delay(SPIN_MS);

    // Stop a moment
    analogWrite(PIN_AENBL, 0);
    analogWrite(PIN_BENBL, 0);
    delay(PAUSE_MS);
  }
  else {
    // No obstacle -> move forward
    expWrite(expState & 0b11111100);  // FORWARD = P0=0, P1=0
    analogWrite(PIN_AENBL, FWD_SPEED);
    analogWrite(PIN_BENBL, FWD_SPEED);
    delay(20);
  }
}