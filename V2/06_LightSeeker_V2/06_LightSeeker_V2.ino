/*
  ============================================================
  Project: Light Seeker (Spin Only)
  Board: Kypruino UNO+ v0.8
  Platform: RoboRover Core 2
  ============================================================

  Description:
  The robot spins on the spot to face a bright light source.
  It uses two LDR sensors (left and right).

  Behaviour:
    - If LEFT sees more light -> spin LEFT
    - If RIGHT sees more light -> spin RIGHT
    - If small difference -> stop

  Hardware:
  - LDR Left  = A6
  - LDR Right = A7
  - Motors: DRV8836 MODE=1 (PHASE/ENABLE)
      AENBL = D11 (PWM), BENBL = D10 (PWM)
      APHASE = P0, BPHASE = P1 (I2C expander @ 0x20)

  Notes:
  - This sketch does NOT move forward.
  - It is intentionally simple and robust.

  ============================================================
*/

#include <Wire.h>

// -------------------- LDR PINS --------------------
#define PIN_L_LDR A6
#define PIN_R_LDR A7

// -------------------- MOTOR PINS --------------------
#define PIN_AENBL 11
#define PIN_BENBL 10

// -------------------- I2C EXPANDER --------------------
#define EXP_ADDR 0x20
uint8_t expState = 0xFF;

// -------------------- BEHAVIOUR SETTINGS --------------------
#define SPIN_SPEED     120   // how fast to spin
#define DIFF_DEADBAND   40   // define "small" differences in LDR readings
#define LOOP_MS         40


void setup() {
  pinMode(PIN_AENBL, OUTPUT);
  pinMode(PIN_BENBL, OUTPUT);

  analogWrite(PIN_AENBL, 0);
  analogWrite(PIN_BENBL, 0);

  Wire.begin();

  // Default expander state HIGH, inputs safe
  expWrite(0xFF);
}

void loop() {
  // Read LDR sensors
  int left  = analogRead(PIN_L_LDR);
  int right = analogRead(PIN_R_LDR);

  // Difference between sensors
  int diff = left - right;

  if (diff > DIFF_DEADBAND) {
    // LEFT is brighter -> spin LEFT
    // Spin LEFT = P0=0, P1=1
    expWrite((expState & 0b11111100) | 0b00000010); // makes P0 and P1 LOW(0) without affecting other bits (& operator), then ensures P1 is set to HIGH(1) (| operator), and writes to the expander
    analogWrite(PIN_AENBL, SPIN_SPEED);
    analogWrite(PIN_BENBL, SPIN_SPEED);
  }
  else if (diff < -DIFF_DEADBAND) {
    // RIGHT is brighter -> spin RIGHT
    // Spin RIGHT = P0=1, P1=0
    expWrite((expState & 0b11111100) | 0b00000001); // makes P0 and P1 LOW(0) without affecting other bits (& operator), then ensures P0 is set to HIGH(1) (| operator), and writes to the expander
    analogWrite(PIN_AENBL, SPIN_SPEED);
    analogWrite(PIN_BENBL, SPIN_SPEED);
  }
  else {
    // Light roughly centred -> stop
    analogWrite(PIN_AENBL, 0);
    analogWrite(PIN_BENBL, 0);
  }

  delay(LOOP_MS);
}


// -------------------- EXPANDER WRITE --------------------
void expWrite(uint8_t v) {  // Sending to expander using I2C protocol
  Wire.beginTransmission(EXP_ADDR);
  Wire.write(v);
  Wire.endTransmission();
  expState = v;
}