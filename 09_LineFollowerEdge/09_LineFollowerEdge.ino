/*
  ============================================================
  Project: Line Edge Follower (1-IR)
  Board: Kypruino UNO+ v0.8
  Platform: RoboRover Core (DRV8836 MODE=1: PHASE/ENABLE)
  ============================================================

  Description:
  A simple line follower using a single sensor to follow the right edge of a dark line.
  Tuning needs to be done manually.

  IMPORTANT:
  - Motor direction mapping:
      FORWARD  = P0=0, P1=0
      BACKWARD = P0=1, P1=1
  - Motor mapping:
      Motor A (D11) = RIGHT motor
      Motor B (D10) = LEFT motor

  Hardware:
  - IR line sensor: A1 (Center)
  - Motors:
      AENBL = D11 (RIGHT PWM)
      BENBL = D10 (LEFT  PWM)
      APHASE/BPHASE via expander @0x20 (P0/P1)
  ============================================================
*/

#include <Wire.h>

// -------------------- SENSOR PINS --------------------
#define PIN_IR_C A1

// -------------------- MOTOR PINS --------------------
#define PIN_AENBL 11          // RIGHT motor PWM
#define PIN_BENBL 10          // LEFT motor PWM
#define EXP_ADDR  0x20


// -------------------- EXPANDER DEFAULT STATE --------------------
uint8_t expState = 0xFF;

// -------- Extras --------
#define BUZZER_PIN 9

#define threshC 120 // Needs tuning!


// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);

  // Motor PWM pins
  pinMode(PIN_AENBL, OUTPUT);  // A = RIGHT motor
  pinMode(PIN_BENBL, OUTPUT);  // B = LEFT motor
  // Stop motors
  analogWrite(PIN_AENBL, 0);
  analogWrite(PIN_BENBL, 0);

  // I2C expander
  Wire.begin();
  pinExpander(0xFF);

  // For tuning purposes, uncomment and test output values in Serial Monitor
  /*
  for (int i = 0; i < 100; i++) {
    Serial.println(analogRead(PIN_IR_C));
    delay(100);
  }
  */

  // Set forward direction
  pinExpander(expState & 0b11111100);
}


// -------------------- MAIN LOOP --------------------
void loop() {
  // Read sensors
  int vC = analogRead(PIN_IR_C);  // Center value

  if (vC < threshC) {
    // Line detected in center - turn right (away from line)
    analogWrite(PIN_BENBL, 150);  // Left motor
    analogWrite(PIN_AENBL, 100);  // Right motor
  } else {
    // Lost line - turn left (towards line)
    analogWrite(PIN_BENBL, 100);
    analogWrite(PIN_AENBL, 150);
  }

  delay(25);
}

// Expander helper
void pinExpander(uint8_t v) {
  Wire.beginTransmission(EXP_ADDR);
  Wire.write(v);
  Wire.endTransmission();
  expState = v;
}