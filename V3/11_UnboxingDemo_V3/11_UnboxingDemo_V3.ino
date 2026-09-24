// 11_UnboxingDemo_V3 - the program the rover ships with. Switch it on and
// drive it straight away: with the IR remote from the box, or from a phone
// over WiFi. Both work at the same time, and do the same things.
// The rover moves: give it floor space.
//
// Phone
//   1. Join the WiFi network "RoboRover-XXXX".
//   2. Open http://192.168.4.1 in Chrome (Android) or Safari (iPhone).
//   3. Drive with the half-circle joystick - the green strip in the middle
//      drives dead straight. Reverse, Avoid, Lights, Dance, Beep, Piano,
//      Custom A / B, speed and distance driven are on the page too. The
//      Console (>_) has more: type help. A gamepad or tilting the phone can
//      drive as well (switch them on in the Console).
//
// IR remote
//   up / down    hold: drive forwards / backwards, dead straight
//   left / right hold: spin on the spot
//   OK           STOP everything
//   1 Lights (next colour)   2 Beep                   3 Dance
//   4 Custom A (distance)    5 Tune                   6 Custom B (battery)
//   7 Avoid on / off         8 say distance driven    9 Piano: 1-8 play notes, 9 ends
//   * slower                 0 distance driven to 0   # faster
//   While Dance or Tune runs, any button stops it.
//
// The rover's screen shows how to connect. Once a phone is connected or the
// remote is used: the motors, what the rover is doing, the distance ahead,
// the speed, the battery and the last thing it said.
//
// This sketch is 08_CommandFusionHub_V3's Kypruino_Rover plus the IR remote.
// WiFiLink.h is the same file as there; Screen.h adds the remote.
// RoverSerial is Arduino's SoftwareSerial, trimmed so the IR receiver can
// share the chip's pin-change interrupts (see the note in RoverSerial.cpp).
//
// Libraries: RoboRoverCore3, Adafruit NeoPixel, U8g2, IRremote.

#include <RoboRoverCore3.h>
#include <Adafruit_NeoPixel.h>

// The WiFi link uses RoverSerial instead of SoftwareSerial (see above)
#include "RoverSerial.h"
#define WIFI_SERIAL_CLASS RoverSerial
#include "WiFiLink.h"

// The IR receiver pin must be set before this include.
#define IR_RECEIVE_PIN RR_PIN_IR_RECV
#include <TinyIRReceiver.hpp>

RoboRoverCore3        rover;
RoboRoverCore3Battery battery;
Adafruit_NeoPixel     leds(RR_NEOPIXEL_COUNT, RR_PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

bool haveBattery = false;
int lightStep = 0;                 // which colour the Lights button shows
unsigned long lastPublishMs = 0;

// The screen shows the driving view for a minute after the last remote button
unsigned long irLastKeyMs = 0;
bool remoteInUse() { return irLastKeyMs != 0 && millis() - irLastKeyMs < 60000; }

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

// Tap the speed in the phone's top bar, or remote button 0, to start from 0
void sayTrip() {
  say(F("Driven so far in cm:"), wifiTripCm());
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
//  The phone's buttons - and the remote's, which run the same ones
//  button is one of: "LIGHTS", "DANCE", "BEEP", "A", "B",
//  "TUNE" (remote button 5), "PIANO" (only from the internet remote: the
//  phone opens a keyboard)
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
  else if (same(button, "TUNE") || same(button, "PIANO")) {
    playTune();
  }
  else if (same(button, "DANCE")) {
    dance();
  }
  else if (same(button, "A")) {
    // Custom A - try changing what it does! (remote button 4)
    sayDistance();
  }
  else if (same(button, "B")) {
    // Custom B - try changing what it does! (remote button 6)
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
    sayTrip();
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
  else if (same(text, "remote")) {
    say(F("IR: arrows drive, OK stop, 1 lights, 2 beep, 3 dance, 4 A, 5 tune, 6 B, "
          "7 avoid, 8 trip, 9 piano, 0 trip to 0, * slower, # faster"));
  }
  else if (same(text, "help")) {
    say(F("Try: hello, red, green, blue, off, distance, battery, trip, tune, spin, beep 440, calibrate, link, remote"));
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
//  IR remote - it drives and presses buttons the same way the phone does:
//  joystickDrive(), runButton() and stopAll() come from WiFiLink.h, so the
//  obstacle guard, the straight hold and the phone's display all apply.
// ---------------------------------------------------------------------------

const uint8_t IR_UP = 0x18, IR_DOWN = 0x52, IR_LEFT = 0x08, IR_RIGHT = 0x5A, IR_OK = 0x1C;
const uint8_t IR_1 = 0x45, IR_2 = 0x46, IR_3 = 0x47, IR_4 = 0x44, IR_5 = 0x40, IR_6 = 0x43;
const uint8_t IR_7 = 0x07, IR_8 = 0x15, IR_9 = 0x09, IR_0 = 0x19, IR_STAR = 0x16, IR_HASH = 0x0D;

// A held button repeats about nine times a second. If nothing arrives for
// this long, the button was let go.
const unsigned long IR_HOLD_MS = 180;

int irSpeed = 700;                 // like the phone's speed slider: 300..1000, * and # change it
bool irDriving = false;            // an arrow is held
bool irPiano = false;              // buttons 1..8 play notes
unsigned long irHeldMs = 0;

// While a dance or tune runs, any new remote button stops it
// (WiFiLink.h calls this during driveFor() and waitFor()).
void remoteDuringAction() {
  if (TinyReceiverDecode() && !(TinyIRReceiverData.Flags & IRDATA_FLAGS_IS_REPEAT)) {
    irLastKeyMs = millis();
    stopAction();
  }
}

// The buttons that act once per press
void remoteButton(uint8_t button) {
  // Piano: 1..8 are notes, 9 (or OK) ends it
  if (irPiano && button == IR_9) {
    irPiano = false;
    say(F("Piano off"));
    return;
  }

  if (button == IR_OK) {
    irPiano = false;
    stopAll();                              // the same as the phone's STOP
  }
  else if (button == IR_1) runButton("LIGHTS");
  else if (button == IR_2) runButton("BEEP");
  else if (button == IR_3) runButton("DANCE");
  else if (button == IR_4) runButton("A");
  else if (button == IR_5) runButton("TUNE");
  else if (button == IR_6) runButton("B");
  else if (button == IR_7) {
    setObstacleGuard(!obstacleGuardOn());   // the phone's Avoid button follows
    tone(RR_PIN_BUZZER, obstacleGuardOn() ? 1600 : 600, 120);   // high = on, low = off
    say(obstacleGuardOn() ? F("Avoid on") : F("Avoid off"));
  }
  else if (button == IR_8) sayTrip();
  else if (button == IR_9) {
    irPiano = true;
    say(F("Piano: 1-8 play, 9 ends"));
  }
  else if (button == IR_0) {
    resetTrip();
    tone(RR_PIN_BUZZER, 900, 80);
    say(F("Distance driven: 0"));
  }
  else if (button == IR_STAR || button == IR_HASH) {
    irSpeed = constrain(irSpeed + (button == IR_HASH ? 100 : -100), 300, 1000);
    tone(RR_PIN_BUZZER, 400 + irSpeed, 80);                    // higher = faster
    say(F("Remote speed %:"), irSpeed / 10);
  }
}

// Piano: buttons 1..8 play one octave, C5 to C6, like the phone's keyboard.
// Which note a button plays, 0 = not a note button.
int pianoNote(uint8_t button) {
  switch (button) {
    case IR_1: return 523;    // C
    case IR_2: return 587;    // D
    case IR_3: return 659;    // E
    case IR_4: return 698;    // F
    case IR_5: return 784;    // G
    case IR_6: return 880;    // A
    case IR_7: return 988;    // B
    case IR_8: return 1047;   // C
  }
  return 0;
}

void checkRemote() {
  unsigned long now = millis();

  if (TinyReceiverDecode()) {
    uint8_t button = TinyIRReceiverData.Command;
    bool repeat = TinyIRReceiverData.Flags & IRDATA_FLAGS_IS_REPEAT;
    irLastKeyMs = now;

    // Arrows: drive while held, forwards and backwards dead straight on
    // the encoders, left and right spin a little slower than they drive
    int fwd = 0, turn = 0;
    if (button == IR_UP)         fwd = irSpeed;
    else if (button == IR_DOWN)  fwd = -irSpeed;
    else if (button == IR_LEFT)  turn = -irSpeed * 3 / 5;
    else if (button == IR_RIGHT) turn = irSpeed * 3 / 5;

    int hz = irPiano ? pianoNote(button) : 0;
    if (fwd || turn) {
      joystickDrive(fwd, turn);
      irDriving = true;
      irHeldMs = now;
    }
    else if (hz) {
      // A held note keeps sounding: every repeat plays it on a little longer
      tone(RR_PIN_BUZZER, hz, 200);
    }
    else if (!repeat) {
      remoteButton(button);                 // once per press, not again while held
    }
  }

  // Arrow let go: stop, once - so the phone's joystick is not disturbed
  if (irDriving && now - irHeldMs > IR_HOLD_MS) {
    irDriving = false;
    joystickDrive(0, 0);
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

  initPCIInterruptForTinyReceiver();
  whileBusy = remoteDuringAction;   // the remote can stop a dance or tune

  wifiBegin();
  say(F("RoboRover ready. Type help in the Console."));
}

void loop() {
  wifiUpdate();   // always first: listens to the phone
  checkRemote();  // then the IR remote

  // Your own code can go here too - keep it quick, and use waitFor(), not delay().

  // Example: send the battery level to the internet every 30 seconds.
  if (haveBattery && millis() - lastPublishMs >= 30000) {
    lastPublishMs = millis();
    battery.poll();
    publish("battery", battery.percent());
  }
}
