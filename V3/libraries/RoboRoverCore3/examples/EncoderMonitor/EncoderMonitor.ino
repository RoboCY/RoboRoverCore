// EncoderMonitor - how far each wheel has turned.
// Nothing moves. Turn a wheel by hand and watch its count change.
// Open the Serial Monitor at 115200 baud.

#include <RoboRoverCore3.h>

RoboRoverCore3 rover;

void setup() {
  Serial.begin(115200);
  rover.begin();              // also sets both counts to 0

  Serial.println("Turn a wheel by hand.");
  Serial.println("574 ticks is one full turn; forward counts up.");
}

void loop() {
  rover.poll();               // fetch the latest counts

  Serial.print("Left ticks: ");
  Serial.print(rover.leftTicks());
  Serial.print(" | Right ticks: ");
  Serial.println(rover.rightTicks());

  delay(250);
}
