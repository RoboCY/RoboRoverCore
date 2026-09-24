// BatteryAndAccelMonitor - battery level and which way is down.
// Nothing moves. Open the Serial Monitor at 115200 baud.

#include <RoboRoverCore3.h>

RoboRoverCore3        rover;
RoboRoverCore3Battery battery;
RoboRoverCore3Accel   accel;

void setup() {
  Serial.begin(115200);
  rover.begin();
  battery.begin();
  accel.begin();
}

void loop() {
  battery.poll();
  accel.poll();

  Serial.print("Battery: ");
  Serial.print(battery.millivolts() / 1000.0, 2);
  Serial.print(" V, ");
  Serial.print(battery.percent());
  Serial.print("% | Accel: ");      // milli-g: about 1000 straight down
  Serial.print(accel.x());
  Serial.print(' ');
  Serial.print(accel.y());
  Serial.print(' ');
  Serial.println(accel.z());

  delay(500);
}
