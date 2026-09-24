// LineSensorReader - the five line sensors underneath.
// Nothing moves. White reads high, a black line low.
// Open the Serial Monitor at 115200 baud.

#include <RoboRoverCore3.h>

RoboRoverCore3 rover;

void setup() {
  Serial.begin(115200);
  rover.begin();
}

void loop() {
  // The line sensors are read on their own, only when you ask.
  if (rover.line.poll()) {
    Serial.print("Line (right to left): ");
    for (int i = 0; i < rover.line.channels(); i++) {
      Serial.print(rover.line.raw(i));
      Serial.print(' ');
    }
    Serial.println();
  } else {
    Serial.println("Line sensors not found - is the board plugged in?");
  }

  delay(100);
}
