/*
  ============================================================
  Project: IR Remote-Controlled Rover
  Board: Kypruino UNO+ v0.8
  Platform: RoboRover Core 2 (DRV8836 MODE=1: PHASE/ENABLE)
  ============================================================

  Description:
  Use an IR remote (NEC protocol) to control RoboRover Core 2.
  Movement buttons control the robot's movements (keeps moving until unpressed).
  Number buttons set NeoPixel colours/patterns.
  * and # play short low/high beeps (implemented WITHOUT tone()).

  Hardware:
  - IR Receiver: A3
  - Motors:
      AENBL = D11 (PWM)
      BENBL = D10 (PWM)
      M1_PHASE = P0, M2_PHASE = P1 via PCF8574 @ 0x20
    Wiring:
      FORWARD  = P0=0, P1=0
      BACKWARD = P0=1, P1=1
  - NeoPixels: D8
  - Buzzer: D9
  ============================================================
*/

#include <Wire.h>
#include <Adafruit_NeoPixel.h>

#define IR_RECEIVE_PIN A3
#include <TinyIRReceiver.hpp>

// -------------------- PINS --------------------
#define PIN_AENBL    11
#define PIN_BENBL    10
#define PIN_NEOPIXEL 8
#define PIN_BUZZER   9

// -------------------- I2C --------------------
#define EXP_ADDR     0x20   // PCF8574 on RoboRover Core 2

// -------------------- NEOPIXELS --------------------
#define LED_COUNT    6

// -------------------- SPEEDS --------------------
#define SPEED_MOVE  200
#define SPEED_SPIN  150

#define HOLD_TIMEOUT_MS 150
unsigned long lastArrowMs = 0;

// -------------------- STATE --------------------
Adafruit_NeoPixel pixels(LED_COUNT, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// Cache expander outputs so only P0/P1 are changed
uint8_t expState = 0xFF;

// Current commanded motor values
int curA = 0;
int curB = 0;

void setup() {
  pinMode(PIN_AENBL, OUTPUT);
  pinMode(PIN_BENBL, OUTPUT);

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  pinMode(IR_RECEIVE_PIN, INPUT);

  Wire.begin();

  // default HIGH on expander
  expWrite(0xFF);

  // stop motors at startup
  applyMotors(0, 0);

  pixels.begin();
  pixels.setBrightness(255);
  pixels.clear();
  pixels.show();

  initPCIInterruptForTinyReceiver();
  enablePCIInterruptForTinyReceiver();
}

void loop() {
  unsigned long now = millis();

  // release-to-stop
  if (now - lastArrowMs > HOLD_TIMEOUT_MS) {
    curA = 0;
    curB = 0;
  }

  if (TinyReceiverDecode()) {
    uint8_t cmd = TinyIRReceiverData.Command;

    // ---- Movement ----
    if (cmd == 0x18) {           // Forward
      curA = +SPEED_MOVE;
      curB = +SPEED_MOVE;
      lastArrowMs = now;
    }
    else if (cmd == 0x52) {      // Backward
      curA = -SPEED_MOVE;
      curB = -SPEED_MOVE;
      lastArrowMs = now;
    }
    else if (cmd == 0x5A) {      // Spin right
      curA = -SPEED_SPIN;
      curB = +SPEED_SPIN;
      lastArrowMs = now;
    }
    else if (cmd == 0x08) {      // Spin left
      curA = +SPEED_SPIN;
      curB = -SPEED_SPIN;
      lastArrowMs = now;
    }
    else if (cmd == 0x1C) {      // OK button
      hornBeepBeep();
    }

    // ---- LEDs ----
    else if (cmd == 0x45) ledsAll(60, 0, 0);   // 1: red
    else if (cmd == 0x46) ledsAll(0, 60, 0);   // 2: green
    else if (cmd == 0x47) ledsAll(0, 0, 60);   // 3: blue
    else if (cmd == 0x44) ledsAll(60, 0, 60);  // 4: magenta
    else if (cmd == 0x40) ledsAll(60, 60, 0);  // 5: yellow
    else if (cmd == 0x43) ledsAll(0, 60, 60);  // 6: cyan

    else if (cmd == 0x07) { // 7: rainbow-ish fixed pattern
      uint32_t colors[] = {
        pixels.Color(60, 0, 0),
        pixels.Color(60, 20, 0),
        pixels.Color(60, 60, 0),
        pixels.Color(0, 60, 0),
        pixels.Color(0, 0, 60),
        pixels.Color(60, 0, 60),
        pixels.Color(60, 0, 0),
        pixels.Color(60, 20, 0),
        pixels.Color(60, 60, 0)
      };

      for (int i = 0; i < LED_COUNT; i++) {
        pixels.setPixelColor(i, colors[i % (sizeof(colors) / sizeof(colors[0]))]);
      }
      pixels.show();
    }

    else if (cmd == 0x15) { // 8: all one random colour
      uint8_t r = random(0, 80);
      uint8_t g = random(0, 80);
      uint8_t b = random(0, 80);
      ledsAll(r, g, b);
    }
    else if (cmd == 0x09) { // 9: each LED random
      for (int i = 0; i < LED_COUNT; i++) {
        pixels.setPixelColor(
          i,
          pixels.Color(
            (uint8_t)random(0, 80),
            (uint8_t)random(0, 80),
            (uint8_t)random(0, 80)
          )
        );
      }
      pixels.show();
    }
    else if (cmd == 0x19) { // 0: off
      pixels.clear();
      pixels.show();
    }

    // ---- Beeps ----
    else if (cmd == 0x16) shortBeep(350, 90);   // *
    else if (cmd == 0x0D) shortBeep(1200, 90);  // #
  }

  applyMotors(curA, curB);

  delay(10);
}

// ============================================================
//                    I2C EXPANDER HELPERS
// ============================================================
void expWrite(uint8_t v) {
  Wire.beginTransmission(EXP_ADDR);
  Wire.write(v);
  Wire.endTransmission();
  expState = v;
}

void setPhaseBits(bool p0, bool p1) {
  uint8_t v = expState;

  // clear P0 and P1
  v &= 0b11111100;

  if (p0) v |= 0b00000001;
  if (p1) v |= 0b00000010;

  expWrite(v);
}

// ============================================================
//                      MOTOR CONTROL
// ============================================================
void applyMotors(int a, int b) {
  a = constrain(a, -255, 255);
  b = constrain(b, -255, 255);

  // FORWARD  = P0=0, P1=0
  // BACKWARD = P0=1, P1=1
  bool p0 = (a < 0);
  bool p1 = (b < 0);

  setPhaseBits(p0, p1);

  analogWrite(PIN_AENBL, (uint8_t)abs(a));
  analogWrite(PIN_BENBL, (uint8_t)abs(b));
}

// ============================================================
//                        SIMPLE BEEP
// ============================================================
void shortBeep(uint16_t freqHz, uint16_t ms) {
  unsigned long periodUs = 1000000UL / freqHz;
  unsigned long halfUs   = periodUs / 2;

  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    digitalWrite(PIN_BUZZER, HIGH);
    delayMicroseconds(halfUs);
    digitalWrite(PIN_BUZZER, LOW);
    delayMicroseconds(halfUs);
  }
  delay(150);
}

void hornBeepBeep() {
  shortBeep(900, 80);
  shortBeep(900, 80);
  delay(200);
}

// ============================================================
//                        LED HELPERS
// ============================================================
void ledsAll(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < LED_COUNT; i++) {
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }
  pixels.show();
}