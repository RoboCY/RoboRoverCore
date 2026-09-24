// LightSensorReader - the two light sensors on top.
// Nothing moves. Brighter light gives a higher number, 0 to 1023.
// Open the Serial Monitor at 115200 baud.

#include <RoboRoverCore3.h>

RoboRoverCore3 rover;

void setup() {
  Serial.begin(115200);
  rover.begin();
}

void loop() {
  rover.poll();

  Serial.print("Light left: ");
  Serial.print(rover.lightLeft());
  Serial.print(" right: ");
  Serial.println(rover.lightRight());

  delay(250);
}
