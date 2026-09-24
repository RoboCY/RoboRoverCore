// IRRemoteDriving - drive with the arrow buttons on the remote.
// The wheels turn. Hold an arrow to drive; let go to stop.
// Needs the IRremote library (for TinyIRReceiver).

#include <RoboRoverCore3.h>

// The IR receiver pin must be set before this include.
#define IR_RECEIVE_PIN RR_PIN_IR_RECV
#include <TinyIRReceiver.hpp>

RoboRoverCore3 rover;

// Remote buttons
const uint8_t UP    = 0x18;
const uint8_t DOWN  = 0x52;
const uint8_t LEFT  = 0x08;
const uint8_t RIGHT = 0x5A;

// Wheel speeds in ticks per second; 574 ticks is one turn of the wheel.
const int MOVE_SPEED = 2500;
const int SPIN_SPEED = 1500;

// A held button repeats about nine times a second. If nothing arrives for
// this long, the button was let go.
const unsigned long HOLD_TIMEOUT_MS = 150;

unsigned long lastButtonMs = 0;
int leftSpeed = 0;
int rightSpeed = 0;

void setup() {
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
      lastButtonMs = now;
    } else if (button == DOWN) {
      leftSpeed = -MOVE_SPEED;
      rightSpeed = -MOVE_SPEED;
      lastButtonMs = now;
    } else if (button == LEFT) {
      leftSpeed = -SPIN_SPEED;
      rightSpeed = SPIN_SPEED;
      lastButtonMs = now;
    } else if (button == RIGHT) {
      leftSpeed = SPIN_SPEED;
      rightSpeed = -SPIN_SPEED;
      lastButtonMs = now;
    }
  }

  if (now - lastButtonMs > HOLD_TIMEOUT_MS) {
    leftSpeed = 0;
    rightSpeed = 0;
  }

  // Sending the same speeds again keeps the wheels in step, so UP drives
  // straight however long it is held.
  rover.setSpeed(leftSpeed, rightSpeed);
  delay(20);
}
