/*
  ============================================================
  Project: Hello Robot!
  Board: Kypruino UNO+ v0.8
  Platform: RoboRover Core 2 (DRV8836 MODE=1: PHASE/ENABLE)
  ============================================================

  Description:
  A first motor test project for RoboRover Core 2.
  The robot repeats:
    Forward -> Stop -> Backward -> Stop -> Spin Right -> Stop -> Spin Left -> Stop

  Each movement function takes a duration (in milliseconds).

  Motor direction mapping via PCF8574:
    FORWARD  = P0=0, P1=0
    BACKWARD = P0=1, P1=1
    Spin R   = P0=1, P1=0
    Spin L   = P0=0, P1=1

  Notes:
  - D11 = AENBL PWM
  - D10 = BENBL PWM
  - PCF8574 P0 = M1_PHASE
  - PCF8574 P1 = M2_PHASE
  - Other expander bits are left HIGH so they behave as inputs / inactive
  ============================================================
*/

#include <Wire.h>

// -------------------- PINS --------------------
#define PIN_AENBL 11
#define PIN_BENBL 10

// -------------------- SPEED SETTINGS --------------------
#define SPEED_FWD  160
#define SPEED_BACK 160
#define SPEED_SPIN 160

// -------------------- I2C EXPANDER --------------------
#define EXP_ADDR 0x20

// -------------------- PCF8574 BIT DEFINITIONS --------------------
#define BIT_M1_PHASE 0   // P0
#define BIT_M2_PHASE 1   // P1

// Keep all other bits HIGH by default
uint8_t expanderState = 0xFF;

void setup() {
  pinMode(PIN_AENBL, OUTPUT);
  pinMode(PIN_BENBL, OUTPUT);

  analogWrite(PIN_AENBL, 0);
  analogWrite(PIN_BENBL, 0);

  Wire.begin();

  // Set expander to default safe state
  writeExpander(expanderState);

  // Stop at startup
  stopMotors(500);
}

void loop() {
  moveForward(1200);
  stopMotors(600);

  moveBackward(1200);
  stopMotors(600);

  spinRight(800);
  stopMotors(600);

  spinLeft(800);
  stopMotors(600);
}

// -------------------- BASIC ACTIONS --------------------
void stopMotors(unsigned long ms) {
  analogWrite(PIN_AENBL, 0);
  analogWrite(PIN_BENBL, 0);
  delay(ms);
}

void moveForward(unsigned long ms) {
  setMotorPhases(0, 0);   // P0=0, P1=0
  analogWrite(PIN_AENBL, SPEED_FWD);
  analogWrite(PIN_BENBL, SPEED_FWD);
  delay(ms);
}

void moveBackward(unsigned long ms) {
  setMotorPhases(1, 1);   // P0=1, P1=1
  analogWrite(PIN_AENBL, SPEED_BACK);
  analogWrite(PIN_BENBL, SPEED_BACK);
  delay(ms);
}

void spinRight(unsigned long ms) {
  setMotorPhases(1, 0);   // P0=1, P1=0
  analogWrite(PIN_AENBL, SPEED_SPIN);
  analogWrite(PIN_BENBL, SPEED_SPIN);
  delay(ms);
}

void spinLeft(unsigned long ms) {
  setMotorPhases(0, 1);   // P0=0, P1=1
  analogWrite(PIN_AENBL, SPEED_SPIN);
  analogWrite(PIN_BENBL, SPEED_SPIN);
  delay(ms);
}

// -------------------- EXPANDER HELPERS --------------------
void setMotorPhases(uint8_t m1Phase, uint8_t m2Phase) {
  bitWrite(expanderState, BIT_M1_PHASE, m1Phase);
  bitWrite(expanderState, BIT_M2_PHASE, m2Phase);
  writeExpander(expanderState);
}

void writeExpander(uint8_t value) {
  Wire.beginTransmission(EXP_ADDR);
  Wire.write(value);
  Wire.endTransmission();
}