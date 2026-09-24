// 04_BasicMovement_V3 - a routine that uses every way the rover can move:
// exact distances and turns, then speeds for curves, spins and pivots.
// The rover moves: give it about a metre of floor space in front. Each part
// of the routine ends where it started, so the rover stays in one place.

#include <RoboRoverCore3.h>

RoboRoverCore3 rover;

void setup() {
  rover.begin();
  delay(1000);                    // time to put it down and step back
}

void loop() {
  // 1. A square, 30 cm a side.
  //    forwardAndWait() drives an exact distance in millimetres, and
  //    turnRightAndWait() spins on the spot by an exact angle in degrees.
  //    Both wait until the rover has arrived. It counts each wheel's turns
  //    to get them right, and keeps the two wheels in step so the sides are
  //    straight. If the corners are not square on your floor, run the
  //    library's TurnCalibration example once.
  for (int side = 0; side < 4; side++) {
    rover.forwardAndWait(300);
    rover.turnRightAndWait(90);
  }
  delay(1000);

  // 2. A weave, forwards and then back along the same path.
  //    setSpeed(left, right) drives at a speed instead of a distance: wheel
  //    ticks per second, where 574 ticks is one turn of the wheel. A speed
  //    holds until you change it, so delay() decides how long it lasts.
  //    A faster left wheel curves right; negative speeds drive backwards.
  rover.setSpeed(1600, 800);      // curve right
  delay(1000);
  rover.setSpeed(800, 1600);      // curve left
  delay(1000);
  rover.setSpeed(-800, -1600);    // back along the left curve
  delay(1000);
  rover.setSpeed(-1600, -800);    // back along the right curve
  delay(1000);
  rover.stop();                   // brakes: the wheels hold still
  delay(1000);

  // 3. Spins: opposite speeds turn the wheels opposite ways, so the rover
  //    turns on the spot.
  rover.setSpeed(1500, -1500);    // spin right
  delay(1000);
  rover.setSpeed(-1500, 1500);    // spin left, back to where it faced
  delay(1000);
  rover.stop();
  delay(1000);

  // 4. Pivots: a speed of 0 holds one wheel still, and the rover swings
  //    round it.
  rover.setSpeed(1500, 0);        // swing round the right wheel
  delay(1000);
  rover.setSpeed(-1500, 0);       // and back
  delay(1000);
  rover.stop();
  delay(2000);
}
