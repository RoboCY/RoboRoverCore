// 01_LEDs_V3 - colours on the eight RGB LEDs.
// Nothing moves.

#include <RoboRoverCore3.h>
#include <Adafruit_NeoPixel.h>

RoboRoverCore3 rover;
Adafruit_NeoPixel leds(RR_NEOPIXEL_COUNT, RR_PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

void allLeds(uint8_t r, uint8_t g, uint8_t b) {
  leds.fill(leds.Color(r, g, b));
  leds.show();
}

void setup() {
  // Also stops the wheels, in case the last sketch left them turning.
  rover.begin();

  leds.begin();
  leds.setBrightness(80);
}

void loop() {
  allLeds(80, 0, 0);    // red
  delay(500);

  allLeds(0, 80, 0);    // green
  delay(500);

  allLeds(0, 0, 80);    // blue
  delay(500);

  // One colour per LED. They are numbered 0 to 7.
  leds.clear();
  leds.setPixelColor(0, 80, 0, 0);
  leds.setPixelColor(1, 80, 40, 0);
  leds.setPixelColor(2, 80, 80, 0);
  leds.setPixelColor(3, 0, 80, 0);
  leds.setPixelColor(4, 0, 80, 80);
  leds.setPixelColor(5, 0, 0, 80);
  leds.setPixelColor(6, 60, 0, 80);
  leds.setPixelColor(7, 80, 0, 40);
  leds.show();
  delay(800);

  leds.clear();
  leds.show();
  delay(500);
}
