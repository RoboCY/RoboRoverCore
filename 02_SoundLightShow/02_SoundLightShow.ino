/*
  ============================================================
  Project: Sound & Light Show (Button Triggered)
  Board: Kypruino UNO+ v0.8
  Platform: RoboRover Core
  ============================================================

  Description:
  Press one of the onboard buttons to trigger a light and sound
  animation. Each press triggers the animation once.

  Buttons:
    A (D7): Police siren (red/blue)
    B (D6): Green chase with scale up/down
    C (D4): Random colours + random beeps
    D (D2): Smooth rainbow + smooth sound sweep

  Hardware:
    - NeoPixels: D8 (6 LEDs)
    - Buzzer: D9
    - Buttons: D7, D6, D4, D2 (INPUT_PULLUP)

  ============================================================
*/

#include <Adafruit_NeoPixel.h>

// -------------------- PINS --------------------
#define PIN_NEOPIXEL 8
#define PIN_BUZZER   9

#define PIN_BTN_A 7
#define PIN_BTN_B 6
#define PIN_BTN_C 4
#define PIN_BTN_D 2

// -------------------- LED SETTINGS --------------------
#define LED_COUNT  6

Adafruit_NeoPixel pixels(LED_COUNT, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// -------------- SETUP --------------
void setup() {
  pinMode(PIN_BUZZER, OUTPUT);
  noTone(PIN_BUZZER);

  pinMode(PIN_NEOPIXEL, OUTPUT);

  pinMode(PIN_BTN_A, INPUT_PULLUP);
  pinMode(PIN_BTN_B, INPUT_PULLUP);
  pinMode(PIN_BTN_C, INPUT_PULLUP);
  pinMode(PIN_BTN_D, INPUT_PULLUP);

  pixels.begin();
  pixels.setBrightness(100);
  pixels.clear();
  pixels.show();
}

// -------------- MAIN LOOP --------------
void loop() {
  if (digitalRead(PIN_BTN_A) == LOW) {
    patternA_police();
  } else if (digitalRead(PIN_BTN_B) == LOW) {
    patternB_chaseScale();
  } else if (digitalRead(PIN_BTN_C) == LOW) {
    patternC_random();
  } else if (digitalRead(PIN_BTN_D) == LOW) {
    patternD_rainbowSweep();
  }

  delay(20); // simple debounce
}


// Color wheel helper
uint32_t wheel(byte pos) {
  pos = 255 - pos;
  if (pos < 85) return pixels.Color(255 - pos * 3, 0, pos * 3);
  if (pos < 170) {
    pos -= 85;
    return pixels.Color(0, pos * 3, 255 - pos * 3);
  }
  pos -= 170;
  return pixels.Color(pos * 3, 255 - pos * 3, 0);
}

// -------------------- PATTERN A --------------------
void patternA_police() {
  int cycles = 4;
  for (int c = 0; c < cycles; c++) {
    pixels.clear();
    for (int i = 0; i < LED_COUNT; i++) { // Loop through LEDs
      if (i % 2 == 0) { // Even numbered LEDs
        pixels.setPixelColor(i, pixels.Color(100, 0, 0));  // Red
      }
      else {  // Odd numbered LEDs
        pixels.setPixelColor(i, pixels.Color(0, 0, 100));  // Blue
      }           
    }
    pixels.show();
    tone(PIN_BUZZER, 950);
    delay(250);

    pixels.clear();
    for (int i = 0; i < LED_COUNT; i++) {
      if (i % 2 == 0) {
        pixels.setPixelColor(i, pixels.Color(0, 0, 100));  // Blue
      }
      else {
        pixels.setPixelColor(i, pixels.Color(100, 0, 0));  // Red
      }
    }
    pixels.show();
    tone(PIN_BUZZER, 750);
    delay(250);
  }
  noTone(PIN_BUZZER);
  pixels.clear();
  pixels.show();
  while (digitalRead(PIN_BTN_A) == LOW) {
    // stay here if button is still pressed
    delay(20);
  }
}

// -------------------- PATTERN B --------------------
void patternB_chaseScale() {
  int notes[6] = {262, 294, 330, 349, 392, 440};

  for (int i = 0; i < LED_COUNT; i++) { // Counting LEDs up
    pixels.clear();
    pixels.setPixelColor(i, pixels.Color(0, 100, 0));  // Green
    pixels.show();
    tone(PIN_BUZZER, notes[i]);
    delay(100);
  }

  for (int i = LED_COUNT - 1; i >= 0; i--) {  // Counting LEDs down
    pixels.clear();
    pixels.setPixelColor(i, pixels.Color(0, 100, 0));  // Green
    pixels.show();
    tone(PIN_BUZZER, notes[i]);
    delay(100);
  }

  noTone(PIN_BUZZER);
  pixels.clear();
  pixels.show();
  while (digitalRead(PIN_BTN_B) == LOW) {
    // stay here if button is still pressed
    delay(20);
  }
}

// -------------------- PATTERN C --------------------
void patternC_random() {
  for (int r = 0; r < 3; r++) {
    for (int i = 0; i < LED_COUNT; i++) {
      pixels.setPixelColor(i, pixels.Color(random(0, 100), random(0, 100), random(0, 100)));
    }
    pixels.show();
    tone(PIN_BUZZER, random(250, 1200));
    delay(150);
  }
  noTone(PIN_BUZZER);
  pixels.clear();
  pixels.show();
  while (digitalRead(PIN_BTN_C) == LOW) {
    // stay here if button is still pressed
    delay(20);
  }
}

// -------------------- PATTERN D --------------------
void patternD_rainbowSweep() {
  for (int t = 0; t <= 255; t++) {
    uint32_t c = wheel(t);
    for (int i = 0; i < LED_COUNT; i++) {
      pixels.setPixelColor(i, c);
    }
    pixels.show();
    tone(PIN_BUZZER, 200 + (t * 4));
    delay(10);
  }
  noTone(PIN_BUZZER);
  pixels.clear();
  pixels.show();
  while (digitalRead(PIN_BTN_D) == LOW) {
    // stay here if button is still pressed
    delay(20);
  }
}