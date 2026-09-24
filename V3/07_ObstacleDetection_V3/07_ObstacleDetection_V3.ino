// 07_ObstacleDetection_V3 - drive forwards, stop when something is in the way.
// The rover moves: give it floor space. Open the Serial Monitor at 115200 baud.
// Start with nothing in front of the rover: the obstacle sensors learn what
// the floor looks like when the program starts.
//
// Three sensors watch the way ahead: the left and right obstacle sensors at
// the front corners, and the ultrasonic sensor in the middle.

#include <RoboRoverCore3.h>

RoboRoverCore3 rover;

const int STOP_CM     = 18;
const int DRIVE_SPEED = 1500;   // wheel ticks per second, about 36 cm/s

void setup() {
  Serial.begin(115200);
  rover.begin();
}

void loop() {
  rover.poll();

  bool leftObstacle  = rover.obstacleLeft();
  bool rightObstacle = rover.obstacleRight();
  // The ultrasonic sensor also sees dark things the obstacle sensors miss.
  bool aheadObstacle = rover.sensors.hasEcho() &&
                       rover.distanceMm() < STOP_CM * 10;

  Serial.print("Left: ");
  Serial.print(leftObstacle);
  Serial.print(" Right: ");
  Serial.print(rightObstacle);
  Serial.print(" Ahead: ");
  Serial.println(aheadObstacle);

  if (leftObstacle || rightObstacle || aheadObstacle) {
    rover.stop();
  } else {
    rover.setSpeed(DRIVE_SPEED, DRIVE_SPEED);   // equal speeds drive straight
  }

  delay(20);
}
