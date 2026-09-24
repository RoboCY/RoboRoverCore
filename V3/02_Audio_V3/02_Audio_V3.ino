// 02_Audio_V3 - beeps and a short tune.
// Nothing moves.

#include <RoboRoverCore3.h>

RoboRoverCore3 rover;

void setup() {
  // Also stops the wheels, in case the last sketch left them turning.
  rover.begin();

  pinMode(RR_PIN_BUZZER, OUTPUT);
}

void loop() {
  // tone(pin, frequency in Hz, length in ms)
  tone(RR_PIN_BUZZER, 1000, 200);
  delay(500);

  tone(RR_PIN_BUZZER, 2000, 200);
  delay(500);

  // A rising sweep: higher frequency, higher note.
  for (int hz = 400; hz <= 2400; hz += 100) {
    tone(RR_PIN_BUZZER, hz, 30);
    delay(40);
  }
  delay(500);

  // A short tune: C, E, G, high C.
  tone(RR_PIN_BUZZER, 523, 150);
  delay(200);
  tone(RR_PIN_BUZZER, 659, 150);
  delay(200);
  tone(RR_PIN_BUZZER, 784, 150);
  delay(200);
  tone(RR_PIN_BUZZER, 1047, 400);
  delay(450);

  noTone(RR_PIN_BUZZER);
  delay(2000);
}
