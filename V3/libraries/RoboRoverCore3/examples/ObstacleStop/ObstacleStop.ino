// ObstacleStop - drive forward, stop when something is in the way.
// The wheels turn. Start with nothing in front of the rover: the obstacle
// sensors learn what the floor looks like when the program starts.
// Open the Serial Monitor at 115200 baud.

#include <RoboRoverCore3.h>

RoboRoverCore3 rover;

void setup() {
  Serial.begin(115200);
  rover.begin();
}

void loop() {
  rover.poll();

  bool leftObstacle  = rover.obstacleLeft();
  bool rightObstacle = rover.obstacleRight();
  // The ultrasonic sensor also sees dark things the obstacle sensors miss.
  bool aheadObstacle = rover.sensors.hasEcho() && rover.distanceMm() < 180;

  Serial.print("Left: ");
  Serial.print(leftObstacle);
  Serial.print(" Right: ");
  Serial.print(rightObstacle);
  Serial.print(" Ahead: ");
  Serial.println(aheadObstacle);

  if (leftObstacle || rightObstacle || aheadObstacle) {
    rover.stop();
  } else {
    rover.setSpeed(800, 800);
  }

  delay(20);
}
