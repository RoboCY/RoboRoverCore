/*
  ============================================================
  Project: Hello Robot!
  Board: Kypruino UNO+ v0.8
  Platform: RoboRover Core (DRV8836 MODE=1: PHASE/ENABLE)
  ============================================================

  Description:
  A first motor test project for RoboRover Core.
  The robot repeats:
    Forward -> Stop -> Backward -> Stop -> Spin Right -> Stop -> Spin Left -> Stop

  Each movement function takes a duration (in milliseconds).

  Wiring:
    FORWARD  = P0=0, P1=0
    BACKWARD = P0=1, P1=1
    Spin R   = P0=1, P1=0
    Spin L   = P0=0, P1=1

  ============================================================
*/

#include <Wire.h>

// -------------------- PINS --------------------
#define PIN_AENBL 11    // PWM
#define PIN_BENBL 10    // PWM

// -------------------- SPEED SETTINGS --------------------
#define SPEED_FWD  160  // 0-255
#define SPEED_BACK 160
#define SPEED_SPIN 160

// -------------------- I2C EXPANDER --------------------
#define EXP_ADDR 0x20

void setup() {
  pinMode(PIN_AENBL, OUTPUT);
  pinMode(PIN_BENBL, OUTPUT);

  Wire.begin();

  // stop at start
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

// P0 = APHASE, P1 = BPHASE
void moveForward(unsigned long ms) {
  expWrite(0xFC);                    // P0=0, P1=0  (FORWARD) binary: (111111[00]) (P7,P6,...,[P1,P0]), or in hexadecimal: FC
  analogWrite(PIN_AENBL, SPEED_FWD);
  analogWrite(PIN_BENBL, SPEED_FWD);
  delay(ms);
}

void moveBackward(unsigned long ms) {
  expWrite(0xFF);                    // P0=1, P1=1  (BACKWARD) binary: 111111[11], or 0xFF
  analogWrite(PIN_AENBL, SPEED_BACK);
  analogWrite(PIN_BENBL, SPEED_BACK);
  delay(ms);
}

void spinRight(unsigned long ms) {
  expWrite(0xFD);                    // P0=1, P1=0 (RIGHT BACKWARD, LEFT FORWARD) 0b11111101, or 0xFD
  analogWrite(PIN_AENBL, SPEED_SPIN);
  analogWrite(PIN_BENBL, SPEED_SPIN);
  delay(ms);
}

void spinLeft(unsigned long ms) {
  expWrite(0xFE);                    // P0=0, P1=1 (RIGHT FORWARD, LEFT BACKWARD) 0b11111110, or 0xFE 
  analogWrite(PIN_AENBL, SPEED_SPIN);
  analogWrite(PIN_BENBL, SPEED_SPIN);
  delay(ms);
}

void expWrite(uint8_t v) {  // Send byte to expander
  Wire.beginTransmission(EXP_ADDR);
  Wire.write(v);
  Wire.endTransmission();
}
