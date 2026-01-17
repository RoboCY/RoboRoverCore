/*
  ============================================================
  Project: Obstacle Avoidance (IR)
  Board: Kypruino UNO+ v0.8
  Platform: RoboRover Core
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
    ACTIVE-HIGH obstacle output:
      1 = obstacle detected
      0 = clear

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

#define REV_MS      250   // reverse time before turning
#define SPIN_MS     550   // turn time
#define PAUSE_MS    120   // small pause between actions

// -------------------- I2C EXPANDER --------------------
#define EXP_ADDR 0x20

#define P_APHASE 0
#define P_BPHASE 1
#define P_OIR_L  2
#define P_OIR_R  3

// Cached expander state (keep input bits HIGH)
uint8_t expState = 0xFF;

// -------------------- EXPANDER I/O --------------------
void expWrite(uint8_t v) {  // Sending to expander using I2C protocol
  Wire.beginTransmission(EXP_ADDR);
  Wire.write(v);
  Wire.endTransmission();
  expState = v;
}

uint8_t expRead() { // Requesting readings from sensors on epxander using I2C protocol
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
  expWrite(expState | (1 << P_OIR_L) | (1 << P_OIR_R)); // "<<"" operator shifs "1" by the required amount of bits to target the desired bit equivalent to the target sensor. eg. 0001 << 2 = 0100, 0001 << 3 = 1000
                                                        // "|" operator is used to ensure the target bits are kept HIGH(1). eg. 0101 | 1000 = 1101, 0101 | 0100 = 0101
  uint8_t v = expRead();

  // ACTIVE-HIGH obstacle detection
  int leftObstacle  = bitRead(v, P_OIR_L);  // if obstacle detected (bit = HIGH(1)), leftObstacle = TRUE(1), otherwise FALSE(0)
  int rightObstacle = bitRead(v, P_OIR_R);  // if obstacle detected (bit = HIGH(1)), rightObstacle = TRUE(1), otherwise FALSE(0)

  if (leftObstacle) {
    // -------- 1) Reverse briefly (BACKWARD = P0=1, P1=1) --------
    expWrite(expState | 0b00000011);  // Set motors to reverse
    analogWrite(PIN_AENBL, REV_SPEED);
    analogWrite(PIN_BENBL, REV_SPEED);
    delay(REV_MS);

    // Stop a moment
    analogWrite(PIN_AENBL, 0);
    analogWrite(PIN_BENBL, 0);
    delay(PAUSE_MS);

    // -------- 2) Spin RIGHT --------
    // For your wiring: spin RIGHT uses P0=1, P1=0
    expWrite((expState & 0b11111100) | 0b00000001); // Start with bits P0 and P1 LOW(0), then turn P0 HIGH(1)
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
    expWrite(expState | 0b00000011);
    analogWrite(PIN_AENBL, REV_SPEED);
    analogWrite(PIN_BENBL, REV_SPEED);
    delay(REV_MS);

    // Stop a moment
    analogWrite(PIN_AENBL, 0);
    analogWrite(PIN_BENBL, 0);
    delay(PAUSE_MS);

    // -------- 2) Spin LEFT --------
    // For your wiring: spin LEFT uses P0=0, P1=1
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
    // No obstacle -> move forward (FORWARD = P0=0, P1=0)
    expWrite(expState & 0b11111100);
    analogWrite(PIN_AENBL, FWD_SPEED);
    analogWrite(PIN_BENBL, FWD_SPEED);
    delay(20);
  }
}
