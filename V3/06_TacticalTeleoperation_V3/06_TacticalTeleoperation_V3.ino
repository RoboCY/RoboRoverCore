// 06_TacticalTeleoperation_V3 - drive the rover with the IR remote.
// The rover moves: give it floor space, or put it on blocks.
//
//   arrows     hold to drive or spin, let go to stop
//   1 2 3      red, green, blue lights
//   OK         lights off
//   *          rainbow
//   #          random colours

#include <RoboRoverCore3.h>

// The IR receiver pin must be set before this include.
#define IR_RECEIVE_PIN RR_PIN_IR_RECV
#include <TinyIRReceiver.hpp>

#include <Adafruit_NeoPixel.h>

RoboRoverCore3    rover;
Adafruit_NeoPixel leds(RR_NEOPIXEL_COUNT, RR_PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// Remote buttons
const uint8_t UP    = 0x18;
const uint8_t DOWN  = 0x52;
const uint8_t LEFT  = 0x08;
const uint8_t RIGHT = 0x5A;
const uint8_t OK    = 0x1C;
const uint8_t STAR  = 0x16;
const uint8_t HASH  = 0x0D;
const uint8_t NUM_1 = 0x45;
const uint8_t NUM_2 = 0x46;
const uint8_t NUM_3 = 0x47;

// Wheel speeds in ticks per second; 574 ticks is one turn of the wheel.
const int MOVE_SPEED = 2500;
const int SPIN_SPEED = 1500;

// A held button repeats about nine times a second. If nothing arrives for
// this long, the button was let go.
const unsigned long HOLD_TIMEOUT_MS = 150;

unsigned long lastDriveButtonMs = 0;
unsigned long lastDriveSentMs = 0;
int leftSpeed = 0;
int rightSpeed = 0;

void allLeds(uint8_t r, uint8_t g, uint8_t b) {
  leds.fill(leds.Color(r, g, b));
  leds.show();
}

void setup() {
  leds.begin();
  leds.clear();
  leds.show();

  rover.begin();
  initPCIInterruptForTinyReceiver();
}

void loop() {
  unsigned long now = millis();

  if (TinyReceiverDecode()) {
    uint8_t button = TinyIRReceiverData.Command;

    if (button == UP) {
      leftSpeed = MOVE_SPEED;
      rightSpeed = MOVE_SPEED;
      lastDriveButtonMs = now;
    } else if (button == DOWN) {
      leftSpeed = -MOVE_SPEED;
      rightSpeed = -MOVE_SPEED;
      lastDriveButtonMs = now;
    } else if (button == LEFT) {
      leftSpeed = -SPIN_SPEED;
      rightSpeed = SPIN_SPEED;
      lastDriveButtonMs = now;
    } else if (button == RIGHT) {
      leftSpeed = SPIN_SPEED;
      rightSpeed = -SPIN_SPEED;
      lastDriveButtonMs = now;
    } else if (button == NUM_1) {
      allLeds(60, 0, 0);
    } else if (button == NUM_2) {
      allLeds(0, 60, 0);
    } else if (button == NUM_3) {
      allLeds(0, 0, 60);
    } else if (button == OK) {
      allLeds(0, 0, 0);
    } else if (button == STAR) {
      for (int i = 0; i < RR_NEOPIXEL_COUNT; i++) {
        leds.setPixelColor(i, leds.ColorHSV(i * 65536L / RR_NEOPIXEL_COUNT, 255, 60));
      }
      leds.show();
    } else if (button == HASH) {
      for (int i = 0; i < RR_NEOPIXEL_COUNT; i++) {
        leds.setPixelColor(i, random(0, 80), random(0, 80), random(0, 80));
      }
      leds.show();
    }
  }

  if (now - lastDriveButtonMs > HOLD_TIMEOUT_MS) {
    leftSpeed = 0;
    rightSpeed = 0;
  }

  // Send the speeds about 20 times a second, so a button press or release
  // takes effect quickly. The same speeds sent again keep the wheels in
  // step, so an arrow held down drives straight.
  if (now - lastDriveSentMs >= 50) {
    lastDriveSentMs = now;
    rover.setSpeed(leftSpeed, rightSpeed);
  }
}
