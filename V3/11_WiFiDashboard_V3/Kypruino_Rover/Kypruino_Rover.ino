// 11_WiFiDashboard_V3 - drive the rover from your phone, and decide what the
// buttons on the phone do.
// The rover moves: give it floor space, or put it on blocks.
//
// How to use it
//   1. The WiFi module already runs WiFiModule_V3. Leave it as it is.
//   2. Upload this sketch to the Kypruino.
//   3. On your phone, join the WiFi network "RoboRover-XXXX", then open
//      http://192.168.4.1 in Chrome (Android) or Safari (iPhone).
//
// The joystick and the STOP button already work - WiFiLink.h takes care of
// them. Everything else on the phone is up to you:
//
//   Lights, Dance, Beep,              ->  onButton()
//   Custom A, Custom B buttons
//   (Piano opens a keyboard on the phone that plays the rover's buzzer)
//   words typed in the Console        ->  onConsole()
//   messages from the internet        ->  onCloudMessage()   (optional)
//
// Things you can use in your code:
//   say("Hello")                  write a line in the phone's Console
//   say(F("A long sentence"))     the same, F() keeps long text out of the small memory
//   say("Distance", 25)           the same, with a number after it
//   driveFor(left, right, ms)     drive for a while, speeds -1000..1000
//   waitFor(ms)                   wait - use this instead of delay()
//   same(word, "RED")             true when two words are the same
//   wifiDistanceCm()              distance to what is in front, -1 = nothing
//                                 (the phone shows it in its top bar too)
//   wifiForwardBlocked()          true when the obstacle guard (Avoid) says
//                                 the rover must not drive forwards
//   wifiSpeedCms()                speed in cm/s, from the wheel encoders
//   wifiTripCm()                  how far the rover has driven, in cm
//                                 (both show in the phone's top bar too)
//   publish("battery", 87)        send a value to the internet
//
// Pressing STOP, or any button while something is running, ends it.
//
// The rover's screen shows how to connect, and once a phone is connected:
// the motors, what the rover is doing, the distance, the battery and the
// last thing it said. That is in Screen.h - change it if you like.
//
// Libraries: RoboRoverCore3, Adafruit NeoPixel, U8g2.

#include <RoboRoverCore3.h>
#include <Adafruit_NeoPixel.h>
#include "WiFiLink.h"

RoboRoverCore3        rover;
RoboRoverCore3Battery battery;
Adafruit_NeoPixel     leds(RR_NEOPIXEL_COUNT, RR_PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

bool haveBattery = false;
int lightStep = 0;                 // which colour the Lights button shows
unsigned long lastPublishMs = 0;

#include "Screen.h"

// ---------------------------------------------------------------------------
//  Little helpers
// ---------------------------------------------------------------------------

void allLeds(uint8_t r, uint8_t g, uint8_t b) {
  leds.fill(leds.Color(r, g, b));
  leds.show();
}

void rainbow() {
  for (int i = 0; i < RR_NEOPIXEL_COUNT; i++) {
    leds.setPixelColor(i, leds.ColorHSV(i * 65536L / RR_NEOPIXEL_COUNT, 255, 60));
  }
  leds.show();
}

void sayDistance() {
  int cm = wifiDistanceCm();
  if (cm >= 0) {
    say(F("Distance in cm:"), cm);
  } else {
    say(F("Nothing in front of me"));
  }
}

void sayBattery() {
  if (!haveBattery) {
    say(F("No battery gauge found"));
    return;
  }
  battery.poll();
  say(F("Battery %:"), battery.percent());
}

// A short tune. waitFor() keeps listening to the phone, so STOP still works.
void playTune() {
  int notes[] = { 523, 659, 784, 1046, 784, 659, 523 };   // C E G C G E C
  for (int i = 0; i < 7; i++) {
    tone(RR_PIN_BUZZER, notes[i]);
    if (!waitFor(180)) break;        // stopped? then leave the loop
  }
  noTone(RR_PIN_BUZZER);
}

// Spin one way, then the other, with lights and sound.
// driveFor() gives false if you pressed STOP, a button, or moved the
// joystick - then we leave the dance straight away.
void dance() {
  say(F("Let's dance!"));
  for (int round = 0; round < 2; round++) {
    allLeds(60, 0, 0);
    tone(RR_PIN_BUZZER, 1047, 150);
    if (!driveFor(700, -700, 600)) break;     // spin right

    allLeds(0, 0, 60);
    tone(RR_PIN_BUZZER, 1319, 150);
    if (!driveFor(-700, 700, 600)) break;     // spin left

    allLeds(0, 60, 0);
    tone(RR_PIN_BUZZER, 1568, 150);
    if (!driveFor(500, 500, 400)) break;      // a little forwards
    if (!driveFor(-500, -500, 400)) break;    // and back
  }
  allLeds(0, 0, 0);
}

// ---------------------------------------------------------------------------
//  The phone's buttons
//  button is one of: "LIGHTS", "DANCE", "BEEP", "A", "B"
//  ("PIANO" only comes from the internet remote: the phone opens a keyboard)
// ---------------------------------------------------------------------------

void onButton(const char* button) {
  if (same(button, "LIGHTS")) {
    // Each press shows the next colour: red, green, blue, rainbow, off
    lightStep = (lightStep + 1) % 5;
    if (lightStep == 1) allLeds(60, 0, 0);
    if (lightStep == 2) allLeds(0, 60, 0);
    if (lightStep == 3) allLeds(0, 0, 60);
    if (lightStep == 4) rainbow();
    if (lightStep == 0) allLeds(0, 0, 0);
  }
  else if (same(button, "BEEP")) {
    tone(RR_PIN_BUZZER, 1200, 150);
  }
  else if (same(button, "PIANO")) {
    playTune();
  }
  else if (same(button, "DANCE")) {
    dance();
  }
  else if (same(button, "A")) {
    // Custom A - try changing what it does!
    sayDistance();
  }
  else if (same(button, "B")) {
    // Custom B - try changing what it does!
    sayBattery();
  }
}

// ---------------------------------------------------------------------------
//  Words typed in the phone's Console (open it with the >_ button)
// ---------------------------------------------------------------------------

void onConsole(const char* text) {
  if (same(text, "hello")) {
    say(F("Hi! I am your RoboRover."));
  }
  else if (same(text, "red"))   { allLeds(60, 0, 0); }
  else if (same(text, "green")) { allLeds(0, 60, 0); }
  else if (same(text, "blue"))  { allLeds(0, 0, 60); }
  else if (same(text, "off"))   { allLeds(0, 0, 0); }
  else if (same(text, "distance")) {
    sayDistance();
  }
  else if (same(text, "battery")) {
    sayBattery();
  }
  else if (same(text, "trip")) {
    // Tap the speed in the phone's top bar to start from 0 again
    say(F("Driven so far in cm:"), wifiTripCm());
  }
  else if (same(text, "tune")) {
    playTune();
  }
  else if (same(text, "calibrate")) {
    // The obstacle sensors measure the floor at switch-on. On a lighter floor
    // they then see "something" everywhere, so measure this floor again.
    say(rover.sensors.calibrateObstacles() ? F("Obstacle sensors calibrated") : F("Calibration failed"));
  }
  else if (same(text, "link")) {
    // Joystick messages the serial link misheard and the rover threw away
    say(F("Damaged messages ignored:"), wifiBadLines());
  }
  else if (same(text, "spin")) {
    driveFor(600, -600, 800);
  }
  else if (startsWith(text, "beep ")) {
    // "beep 440" plays 440 Hz - the number comes after the word
    int hz = atoi(text + 5);
    tone(RR_PIN_BUZZER, hz, 300);
    say(F("Beep at"), hz);
  }
  else if (same(text, "help")) {
    say(F("Try: hello, red, green, blue, off, distance, battery, trip, tune, spin, beep 440, calibrate, link"));
  }
  else {
    say("I don't know this word:", text);
  }
}

// ---------------------------------------------------------------------------
//  Messages from the internet (optional)
//
//  Needs "Dashboard + Internet" in the phone's Network tab. Anything sent to
//  <rover topic>/in/<name> arrives here as topic <name>. The rover topic is
//  shown in the Network tab, e.g. roborover/3f2a1c.
// ---------------------------------------------------------------------------

void onCloudMessage(const char* topic, const char* data) {
  if (same(topic, "say")) {
    say("Internet says:", data);
  }
  else if (same(topic, "console")) {
    onConsole(data);    // handle it as if it was typed in the Console
  }
}

// ---------------------------------------------------------------------------

void setup() {
  leds.begin();
  allLeds(0, 0, 0);

  rover.begin();
  rover.sensors.enable(RR_SEN_OBSTACLE | RR_SEN_ULTRASONIC | RR_SEN_LDR);
  haveBattery = battery.begin() == RR_OK;

  screenBegin();

  // joystickMaxSpeed = 600;   // uncomment for a gentler joystick (0..1000)
  // turnStrength = 30;        // turning speed in percent (40 is the default)

  wifiBegin();
  say(F("RoboRover ready. Type help in the Console."));
}

void loop() {
  wifiUpdate();   // always first: listens to the phone

  // Your own code can go here too - keep it quick, and use waitFor(), not delay().

  // Example: send the battery level to the internet every 30 seconds.
  if (haveBattery && millis() - lastPublishMs >= 30000) {
    lastPublishMs = millis();
    battery.poll();
    publish("battery", battery.percent());
  }
}
