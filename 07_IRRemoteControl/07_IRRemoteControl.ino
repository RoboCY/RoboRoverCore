/*
  ============================================================
  Project: IR Remote-Controlled Rover
  Board: Kypruino UNO+ v0.8
  Platform: RoboRover Core (DRV8836 MODE=1: PHASE/ENABLE)
  ============================================================

  Description:
  Use an IR remote (NEC protocol) to control RoboRover.
  Movement buttons control the robot's movements (keeps moving until unpressed).
  Number buttons set NeoPixel colours/patterns.
  * and # play short low/high beeps (implemented WITHOUT tone()).

  Hardware:
  - IR Receiver: A3
  - Motors:
      AENBL = D11 (PWM)
      BENBL = D10 (PWM)
      APHASE = P0, BPHASE = P1 via I2C expander @ 0x20
    Wiring:
      FORWARD  = P0=0, P1=0
      BACKWARD = P0=1, P1=1
  - NeoPixels: D8 (6 LEDs)
  - Buzzer: D9

  IR codes:
    FWD  0xE718FF00
    BACK 0xAD52FF00
    RIGHT 0xA55AFF00
    LEFT  0xF708FF00
    OK/STOP 0xE31CFF00
    1..0,*,# etc... defined later in sketch

  ============================================================
*/

#include <Wire.h>
#include <Adafruit_NeoPixel.h>

// -------------------- PINS --------------------
#define PIN_AENBL    11
#define PIN_BENBL    10

#define EXP_ADDR     0x20   // I2C expander address

#define PIN_NEOPIXEL 8
#define LED_COUNT    6

#define PIN_BUZZER   9

// IR receiver pin for TinyIRReceiver
#define IR_RECEIVE_PIN A3

// Use the Tiny receiver (NEC) from IRremote
#include <TinyIRReceiver.hpp>

// -------------------- SPEEDS --------------------
#define SPEED_MOVE  200   // forward/back
#define SPEED_SPIN  150   // slow-ish spin

unsigned long lastArrowMs = 0;
#define HOLD_TIMEOUT_MS 150

// -------------------- STATE --------------------
Adafruit_NeoPixel pixels(LED_COUNT, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// Cache last expander outputs so we only change bits we need (P0/P1).
uint8_t expState = 0xFF;

// Current commanded motor values (kept until button release)
int curA = 0;
int curB = 0;

void setup() {
  pinMode(PIN_AENBL, OUTPUT);
  pinMode(PIN_BENBL, OUTPUT);

  pinMode(PIN_NEOPIXEL, OUTPUT);

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  Wire.begin();
  expWrite(0xFF);          // default HIGH on expander
  applyMotors(0, 0);       // stop

  pixels.begin();
  pixels.setBrightness(255);
  pixels.clear();
  pixels.show();

  // Start Tiny IR receiver (NEC) on IR_RECEIVE_PIN
  initPCIInterruptForTinyReceiver();
  enablePCIInterruptForTinyReceiver();  // recommended by Tiny receiver docs :contentReference[oaicite:2]{index=2}
}

void loop() {
  unsigned long now = millis();

  // If no arrow command has been received recently, stop (button released)
  if (now - lastArrowMs > HOLD_TIMEOUT_MS) {
    curA = 0;
    curB = 0;
  }

  if (TinyReceiverDecode()) {
    uint8_t cmd = TinyIRReceiverData.Command; // Assigning signal from remote to a variable

    // ---- Movement (ONLY while held) ----
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

    else if (cmd == 0x1C) {   // OK button
      hornBeepBeep();
    }


    // ---- LEDs ----
    else if (cmd == 0x45) ledsAll(60, 0, 0);  // 1: All Red
    else if (cmd == 0x46) ledsAll(0, 60, 0);  // 2: All Green
    else if (cmd == 0x47) ledsAll(0, 0, 60);  // 3: All Blue
    else if (cmd == 0x44) ledsAll(60, 0, 60); // 4: All Magenta
    else if (cmd == 0x40) ledsAll(60, 60, 0); // 5: All Yellow
    else if (cmd == 0x43) ledsAll(0, 60, 60); // 6: All Cyan

    else if (cmd == 0x07) { // 7: R-O-Y-G-B-P (Rainbow)
      uint32_t c[6] = {
        pixels.Color(60, 0, 0),    // Red
        pixels.Color(60, 20, 0),   // Orange
        pixels.Color(60, 60, 0),   // Yellow
        pixels.Color(0, 60, 0),    // Green
        pixels.Color(0, 0, 60),    // Blue
        pixels.Color(60, 0, 60)    // Purple/Magenta
      };
      for (int i = 0; i < 6; i++) pixels.setPixelColor(i, c[i]);
      pixels.show();
    }

    else if (cmd == 0x15) { // 8: all one random colour
      uint8_t r = random(0, 80), g = random(0, 80), b = random(0, 80);
      ledsAll(r, g, b);
    }
    else if (cmd == 0x09) { // 9: each LED random
      for (int i = 0; i < 6; i++) {
        pixels.setPixelColor(i, pixels.Color((uint8_t)random(0, 80), (uint8_t)random(0, 80), (uint8_t)random(0, 80)));
      } 
      pixels.show();
    }
    else if (cmd == 0x19) { // 0: off
      pixels.clear();
      pixels.show();
    }

    // ---- Beeps ----
    else if (cmd == 0x16) shortBeep(350, 90);
    else if (cmd == 0x0D) shortBeep(1200, 90);
  }

  // Always keep applying the last commanded motor speeds
  applyMotors(curA, curB);

  delay(10);
}


// -------------------- I2C EXPANDER WRITE --------------------
void expWrite(uint8_t v) {  // Sending to expander using I2C protocol
  Wire.beginTransmission(EXP_ADDR);
  Wire.write(v);
  Wire.endTransmission();
  expState = v;
}

// Set P0/P1 (APHASE/BPHASE) only, keep other expander bits unchanged.
void setPhaseBits(bool p0, bool p1) {
  uint8_t v = expState;

  // Clear P0 and P1 first (0)
  v &= 0b11111100;

  // Put new values into P0/P1
  if (p0) v |= 0b00000001;  // if TRUE(1) (p0 == 1), change p0 only to 1 (otherwise stays at 0) (OR operator will keep other bits as their previous state, ensuring p0 turns 1)
  if (p1) v |= 0b00000010;  

  expWrite(v);
}

// Apply motor command (-255..255). Sign sets direction via P0/P1.
void applyMotors(int a, int b) {
  a = constrain(a, -255, 255);
  b = constrain(b, -255, 255);

  // Wiring:
  // FORWARD  = P0=0, P1=0
  // BACKWARD = P0=1, P1=1
  //
  // So: a>=0 means "forward direction" -> phase bit LOW (0)
  //     a<0  means "backward direction" -> phase bit HIGH (1)
  bool p0 = (a < 0);
  bool p1 = (b < 0);
  setPhaseBits(p0, p1); // Writes to the expander, setting up directions for the motors

  analogWrite(PIN_AENBL, (uint8_t)abs(a));  // Motor speed (absolute value)
  analogWrite(PIN_BENBL, (uint8_t)abs(b));
}

// -------------------- SIMPLE BEEP (NO tone()) --------------------
// This avoids Timer2 side-effects when using the IR receiver (remote control)
void shortBeep(uint16_t freqHz, uint16_t ms) {
  // Square wave period (microseconds)
  unsigned long periodUs = 1000000UL / freqHz;
  unsigned long halfUs   = periodUs / 2;

  unsigned long t0 = millis();
  while (millis() - t0 < ms) {  // for the duration of "ms", create the square sound wave
    digitalWrite(PIN_BUZZER, HIGH);
    delayMicroseconds(halfUs);
    digitalWrite(PIN_BUZZER, LOW);
    delayMicroseconds(halfUs);
  }
  delay(150);
}

void hornBeepBeep() {
  shortBeep(900, 80);   // first beep (function incorporates a 150 ms delay at the end)
  shortBeep(900, 80);   // second beep
  delay(200);
}


// -------------------- LED HELPERS --------------------
void ledsAll(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < LED_COUNT; i++) {
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }
  pixels.show();
}




