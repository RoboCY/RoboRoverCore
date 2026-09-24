// 05_PrecisionDistanceLock_V3 - stay 10 cm away from whatever is in front.
// The rover moves: give it floor space. Hold your hand or a book in front of
// it, then move it closer and further away. The rover follows.

#include <RoboRoverCore3.h>

RoboRoverCore3 rover;

const int TARGET_MM       = 100;   // the distance to keep: 10 cm
const int CLOSE_ENOUGH_MM = 10;    // within 1 cm of the target, stay still
const int IGNORE_MM       = 500;   // anything further than 50 cm is ignored

// Speeds are in wheel ticks per second; 574 ticks is one turn of the wheel.
const int SPEED_PER_MM    = 20;    // how much faster to go for each mm of error
const int MIN_SPEED       = 300;   // the slowest it creeps towards the target
const int MAX_SPEED       = 1500;

void setup() {
  rover.begin();

  delay(2000);                // time to put it down and step back
}

void loop() {
  rover.poll();

  int speed = 0;

  if (rover.sensors.hasEcho() && rover.distanceMm() < IGNORE_MM) {
    // Positive error: too far away, drive forwards.
    // Negative error: too close, drive backwards.
    int error = rover.distanceMm() - TARGET_MM;

    if (abs(error) > CLOSE_ENOUGH_MM) {
      // The further off, the faster it moves.
      speed = constrain(error * SPEED_PER_MM, -MAX_SPEED, MAX_SPEED);

      // Never slower than MIN_SPEED, so it does not take forever to close
      // the last few centimetres.
      if (speed > 0 && speed < MIN_SPEED)  speed = MIN_SPEED;
      if (speed < 0 && speed > -MIN_SPEED) speed = -MIN_SPEED;
    }
  }

  // Both wheels at the same speed, so it drives straight towards the target.
  // A speed of 0 brakes.
  rover.setSpeed(speed, speed);

  delay(50);
}
