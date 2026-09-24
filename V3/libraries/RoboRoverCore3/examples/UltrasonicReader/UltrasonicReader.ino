// UltrasonicReader - distance to whatever is in front.
// Nothing moves. Open the Serial Monitor at 115200 baud.

#include <RoboRoverCore3.h>

RoboRoverCore3 rover;

void setup() {
  Serial.begin(115200);
  rover.begin();
}

void loop() {
  rover.poll();

  if (rover.sensors.hasEcho()) {
    Serial.print("Distance cm: ");
    Serial.println(rover.distanceMm() / 10.0, 1);
  } else {
    Serial.println("Distance: nothing in range");
  }

  delay(250);
}
