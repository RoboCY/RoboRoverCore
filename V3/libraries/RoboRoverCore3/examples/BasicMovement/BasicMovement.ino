// BasicMovement - drive a distance, turn, and drive at a speed.
// The wheels turn: put the rover on the floor with some space around it.

#include <RoboRoverCore3.h>

RoboRoverCore3 rover;

void setup() {
  rover.begin();
}

void loop() {
  rover.forwardAndWait(300);      // 300 mm forward, waits until it arrives
  rover.turnRightAndWait(90);     // 90 degrees on the spot

  rover.setSpeed(1000, 1000);     // left, right: wheel ticks per second
  delay(1000);                    // it keeps going until told otherwise
  rover.stop();

  delay(1000);
}
