// WiFiLink.h - the connection between the Kypruino and the WiFi module.
//
// You do not need to change anything in this file. It already handles:
//   - the joystick and the STOP button on the phone
//   - driving dead straight while the joystick is in the phone's green strip
//   - the speed and the distance driven, shown in the phone's top bar
//   - stopping the rover when the phone disconnects or goes quiet
//   - calling onButton(), onConsole() and onCloudMessage() in your sketch
//   - calling screenTick() often, so the screen stays up to date
//
// The WiFi module runs WiFiModule_V3 (in arduino-examples) and talks to the
// Kypruino over a serial link, one line of text per message (19200 baud,
// D12 / D13).

#pragma once
#include <Arduino.h>
#include <RoboRoverCore3.h>

// The serial class for the WiFi module. A sketch can define WIFI_SERIAL_CLASS
// before including this file to use another one (12_UnboxingDemo does, so
// the IR remote can share the pin-change interrupts).
#ifndef WIFI_SERIAL_CLASS
#include <SoftwareSerial.h>
#define WIFI_SERIAL_CLASS SoftwareSerial
#endif

// ---- your sketch writes these ----------------------------------------------
extern RoboRoverCore3 rover;
void onButton(const char* button);
void onConsole(const char* text);
void onCloudMessage(const char* topic, const char* data);
void screenTick();   // in Screen.h

// ---- setting your sketch may change ----------------------------------------
// Top speed of the joystick, 0..1000. The phone's speed slider scales it down.
int joystickMaxSpeed = 1000;
// How strongly the joystick steers while driving along, in percent. 100 = as
// the phone sends it, which turns very quickly. Like a car, the rover steers
// less the faster it goes: at full speed only half of this is used.
// Spinning on the spot always gets full power.
int turnStrength = 40;

// ---- wiring and timing ------------------------------------------------------
const uint8_t WIFI_RX_PIN = 12;  // Kypruino D12 <- WiFi module TX
const uint8_t WIFI_TX_PIN = 13;  // Kypruino D13 -> WiFi module RX
const long    WIFI_BAUD   = 19200;  // must match UART_BAUD in WiFiModule_V3. Software
                                 // serial misreads bits at 57600; 19200 has 3x the margin

const unsigned long JOYSTICK_TIMEOUT_MS = 800;   // no joystick update -> stop
const unsigned long LINK_TIMEOUT_MS     = 2000;  // phone silent -> end the action
const unsigned long DRIVE_REPEAT_MS     = 50;    // how often the joystick is checked against the guard
const unsigned long DRIVE_REFRESH_MS    = 500;   // an unchanged speed is told to the motors again
const unsigned long STATUS_MS           = 1000;  // "rover online" heartbeat to the phone
const unsigned long DISTANCE_MS         = 250;   // front distance for the phone's top bar
const unsigned long GUARD_DISTANCE_MS   = 100;   // measured faster while the guard is on
const int           GUARD_CM            = 15;    // the guard stops forwards closer than this
const unsigned long TRIP_MS             = 500;   // speed and distance driven, from the encoders
const unsigned long TRIP_SEND_MS        = 2000;  // ...told to the phone at least this often

// The joystick drives on the encoders: joystick speed 1000 is this many
// ticks/s, about what the motors reach with no load. On the floor the motion
// chip slows both wheels to the pace the weaker one can hold.
const long JOY_FULL_TICKS = 5000;
// In the green strip, speed changes smaller than this (ticks/s) are ignored,
// so a trembling finger does not restart the motion chip's straight hold.
const long STRAIGHT_BAND  = 300;

WIFI_SERIAL_CLASS wifiSerial(WIFI_RX_PIN, WIFI_TX_PIN);

// ---- internal state ---------------------------------------------------------
char wlLine[90];           // line being received
uint8_t wlLen = 0;
char wlArg[64];            // copy handed to your functions
unsigned long wlLastLineMs = 0;
unsigned long wlPhoneMs = 0;         // last time the phone said anything
bool wlPhoneSeen = false;

int wlJoyLeft = 0, wlJoyRight = 0;   // joystick speeds, -1000..1000
long wlJoyFwd = 0, wlJoyTurn = 0;   // the same, as forwards + turning
bool wlGuard = false;               // obstacle guard, switched from the phone
bool wlBlocked = false;             // the guard is holding the rover back right now
unsigned int wlBadLines = 0;       // joystick lines thrown away as damaged
int wlShowLeft = 0, wlShowRight = 0; // what the motors are doing now (for the screen)
long wlStraightTicks = 0;           // green strip: the speed held straight, 0 = not in it
long wlSentLeft = 0, wlSentRight = 0; // the speeds the motion chip has now
unsigned long wlSentMs = 0;
bool wlResend = true;               // something else drove the motors: send again
unsigned long wlJoyMs = 0, wlDriveSentMs = 0, wlStatusMs = 0;

uint8_t wlBusy = 0;             // > 0 while one of your functions is running
bool wlInAction = false;        // an action the phone started (button / console / cloud)
bool wlOwnsMotors = false;      // driveFor() is steering, not the joystick
bool wlStop = false;            // the running action should end now
const char* wlMode = "0";       // shown on the phone: "0" = teleop, "2" = dance
const char* wlActivity = nullptr;

int wlDistanceCm = -1;          // ultrasonic, -1 = nothing in range
int wlDistanceSent = -2;
bool wlObstacleLeft = false, wlObstacleRight = false;   // IR obstacle sensors
uint8_t wlSidesSent = 255;
unsigned long wlDistanceMs = 0, wlDistanceSentMs = 0;

// Wheel size, for encoder ticks -> cm. wifiBegin() asks the motion chip.
// Whole numbers only: floating point would cost 2 kB of the Kypruino's flash.
uint16_t wlWheelCirc100 = 13823;  // wheel circumference, hundredths of a mm
uint16_t wlTicksPerRev = 574;
long wlEncLeft = 0, wlEncRight = 0;
bool wlEncValid = false;
unsigned long wlTripTicks2 = 0; // distance driven, forwards or back, in half ticks
int wlSpeedCms = 0;             // + forwards, - backwards
int wlSpeedSent = 0;
long wlTripSent = -1;
unsigned long wlTripMs = 0, wlTripSentMs = 0;


char wlSaid[24] = "";           // the last thing say() wrote
char wlHomeIp[16] = "";         // IP on the home WiFi, "" when not joined
bool wlCloud = false;           // connected to the internet broker

// SoftwareSerial cannot listen while it talks: a line the WiFi module starts
// meanwhile arrives garbled. So before talking, wait for a quiet moment -
// nothing arriving for 2 ms - but never longer than 25 ms.
void wlTalk() {
  unsigned long start = millis(), quietUs = micros();
  int seen = wifiSerial.available();
  while (millis() - start < 25) {
    int n = wifiSerial.available();
    if (n != seen || digitalRead(WIFI_RX_PIN) == LOW) { seen = n; quietUs = micros(); }
    else if (micros() - quietUs >= 2000) return;
  }
}

// ---- small helpers for your sketch -----------------------------------------

// true when the two words are exactly the same
bool same(const char* a, const char* b) { return strcmp(a, b) == 0; }

// true when text begins with word, e.g. startsWith("beep 440", "beep ")
bool startsWith(const char* text, const char* word) {
  return strncmp(text, word, strlen(word)) == 0;
}

// The same two, for the link's own words: PSTR("...") keeps them in flash,
// which leaves more of the Kypruino's small RAM for your sketch.
bool wlIs(const char* text, PGM_P word) { return strcmp_P(text, word) == 0; }
bool wlStarts(const char* text, PGM_P word) { return strncmp_P(text, word, strlen_P(word)) == 0; }

// Show a line in the phone's Console (and on the rover's screen).
void wlRemember(const char* label, const char* rest) {
  strncpy(wlSaid, label, sizeof(wlSaid) - 1);
  wlSaid[sizeof(wlSaid) - 1] = '\0';
  if (rest) {
    uint8_t n = strlen(wlSaid);
    if (n < sizeof(wlSaid) - 2) {
      wlSaid[n] = ' ';
      strncpy(wlSaid + n + 1, rest, sizeof(wlSaid) - n - 2);
    }
  }
}
void say(const char* text) {
  wlRemember(text, nullptr);
  wlTalk();
  wifiSerial.print(F("LOG "));
  wifiSerial.print(text);
  wifiSerial.print('\n');
}
void say(const __FlashStringHelper* text) {
  strncpy_P(wlSaid, (const char*)text, sizeof(wlSaid) - 1);
  wlSaid[sizeof(wlSaid) - 1] = '\0';
  wlTalk();
  wifiSerial.print(F("LOG "));
  wifiSerial.print(text);
  wifiSerial.print('\n');
}
void say(const char* label, long number) {
  char n[12];
  ltoa(number, n, 10);
  wlRemember(label, n);
  wlTalk();
  wifiSerial.print(F("LOG "));
  wifiSerial.print(label);
  wifiSerial.print(' ');
  wifiSerial.print(n);
  wifiSerial.print('\n');
}
void say(const __FlashStringHelper* label, long number) {
  char n[12];
  ltoa(number, n, 10);
  strncpy_P(wlSaid, (const char*)label, sizeof(wlSaid) - 1);
  wlSaid[sizeof(wlSaid) - 1] = '\0';
  uint8_t at = strlen(wlSaid);
  if (at < sizeof(wlSaid) - 2) {
    wlSaid[at] = ' ';
    strncpy(wlSaid + at + 1, n, sizeof(wlSaid) - at - 2);
  }
  wlTalk();
  wifiSerial.print(F("LOG "));
  wifiSerial.print(label);
  wifiSerial.print(' ');
  wifiSerial.print(n);
  wifiSerial.print('\n');
}
void say(const __FlashStringHelper* label, int number)           { say(label, (long)number); }
void say(const __FlashStringHelper* label, unsigned int number)  { say(label, (long)number); }
void say(const char* label, int number)           { say(label, (long)number); }
void say(const char* label, unsigned int number)  { say(label, (long)number); }
void say(const char* label, unsigned long number) { say(label, (long)number); }
void say(const char* label, const char* text) {
  wlRemember(label, text);
  wlTalk();
  wifiSerial.print(F("LOG "));
  wifiSerial.print(label);
  wifiSerial.print(' ');
  wifiSerial.print(text);
  wifiSerial.print('\n');
}

// Send a value to the internet as <rover topic>/<topic>.
// Only works in "Dashboard + Internet" mode with a broker set (Network tab);
// otherwise the WiFi module ignores it.
void publish(const char* topic, long value) {
  wlTalk();
  wifiSerial.print(F("PUB "));
  wifiSerial.print(topic);
  wifiSerial.print(' ');
  wifiSerial.print(value);
  wifiSerial.print('\n');
}
void publish(const char* topic, int value)           { publish(topic, (long)value); }
void publish(const char* topic, unsigned int value)  { publish(topic, (long)value); }
void publish(const char* topic, unsigned long value) { publish(topic, (long)value); }
void publish(const char* topic, const char* text) {
  wlTalk();
  wifiSerial.print(F("PUB "));
  wifiSerial.print(topic);
  wifiSerial.print(' ');
  wifiSerial.print(text);
  wifiSerial.print('\n');
}

// ---- what the link knows, for the screen -----------------------------------

// true while a phone (or the internet remote) is connected
bool wifiPhoneConnected() { return wlPhoneSeen && millis() - wlPhoneMs < LINK_TIMEOUT_MS; }

// motor speeds right now, -1000..1000
int wifiLeftSpeed()  { return wlShowLeft; }
int wifiRightSpeed() { return wlShowRight; }

// what the rover is doing: "Ready", "Driving", or the button being run
const char* wifiActivity() {
  if (wlActivity) return wlActivity;
  if (wlBlocked) return "Blocked ahead!";
  if (wlStraightTicks) return "Straight";
  if (wlShowLeft || wlShowRight) return "Driving";
  return "Ready";
}

// distance to whatever is in front, in cm; -1 when nothing is in range
int wifiDistanceCm() { return wlDistanceCm; }

// how many joystick messages arrived damaged and were thrown away
unsigned int wifiBadLines() { return wlBadLines; }

// speed in cm/s from the wheel encoders: + forwards, - backwards, 0 spinning
int wifiSpeedCms() { return wlSpeedCms; }

// encoder ticks -> cm, in whole numbers without overflowing
long wlTicksToCm(unsigned long ticks) {
  unsigned long mm100 = ticks / wlTicksPerRev * wlWheelCirc100 +
                        ticks % wlTicksPerRev * wlWheelCirc100 / wlTicksPerRev;
  return (mm100 + 500) / 1000;
}

// how far the rover has driven, forwards or back, in cm; spinning adds nothing
long wifiTripCm() { return wlTicksToCm(wlTripTicks2 / 2); }

// start counting the distance driven from 0 again (the phone does it when
// the speed display in its top bar is tapped)
void resetTrip() { wlTripTicks2 = 0; }

// true when the IR obstacle sensor on that side sees something close
bool wifiObstacleLeft()  { return wlObstacleLeft; }
bool wifiObstacleRight() { return wlObstacleRight; }

const char* wifiLastSaid()  { return wlSaid; }
const char* wifiHomeIp()    { return wlHomeIp; }    // "" when not on a home WiFi
bool        wifiCloudOnline() { return wlCloud; }

// ---- inside the link --------------------------------------------------------

// Obstacle guard (the "Avoid" button on the phone): true when the rover must
// not drive forwards - an IR sensor sees something, or the ultrasonic reads
// closer than GUARD_CM. Reversing and spinning are always allowed.
bool wifiForwardBlocked() {
  return wlGuard && (wlObstacleLeft || wlObstacleRight ||
                     (wlDistanceCm >= 0 && wlDistanceCm < GUARD_CM));
}

// Switch the obstacle guard from your own code (the unboxing demo does it from
// the IR remote). The phone's Avoid button shows whatever the rover has.
void setObstacleGuard(bool on) { wlGuard = on; wlSidesSent = 255; }
bool obstacleGuardOn() { return wlGuard; }

// End a running driveFor() / waitFor() action, like the phone's STOP does.
void stopAction() { wlStop = true; }

// Called again and again while driveFor() or waitFor() run, so a sketch can
// keep watching something else during an action - e.g. an IR remote button
// that calls stopAction(). Set it in setup(): whileBusy = myFunction;
void (*whileBusy)() = nullptr;

// Joystick -> motors, holding back the forwards part if the guard says so.
// setSpeed(): the motion chip holds each wheel at its speed and, while it is
// sent the same two speeds again and again, keeps the wheels in step (Motion
// 1.15). In the phone's green strip both speeds are equal - dead straight -
// and small changes are ignored there: every new speed starts a new hold, so
// a trembling finger would let the heading wander.
void wlApplyJoystick() {
  long fwd = wlJoyFwd;
  wlBlocked = fwd > 0 && wifiForwardBlocked();
  if (wlBlocked) fwd = 0;
  wlJoyLeft = wlShowLeft = (int)(fwd + wlJoyTurn);
  wlJoyRight = wlShowRight = (int)(fwd - wlJoyTurn);
  long left = wlJoyLeft * JOY_FULL_TICKS / 1000;
  long right = wlJoyRight * JOY_FULL_TICKS / 1000;
  if (wlJoyTurn == 0 && left != 0) {
    if ((left > 0) != (wlStraightTicks > 0) || wlStraightTicks == 0 ||
        abs(left - wlStraightTicks) > STRAIGHT_BAND) wlStraightTicks = left;
    left = right = wlStraightTicks;
  } else {
    wlStraightTicks = 0;
  }
  // The motion chip keeps a speed until it gets another, so only changes go
  // over the bus, plus a refresh now and then. A quieter bus means fewer
  // letters misheard on the WiFi link.
  unsigned long now = millis();
  if (left != wlSentLeft || right != wlSentRight || wlResend || now - wlSentMs >= DRIVE_REFRESH_MS) {
    rover.setSpeed(left, right);
    wlSentLeft = left;
    wlSentRight = right;
    wlSentMs = now;
    wlResend = false;
  }
}

void wlSendStatus() {
  wlStatusMs = millis();
  wlTalk();
  wifiSerial.print(F("T "));
  wifiSerial.print(wlMode);
  wifiSerial.print('\n');
}

void wlStopJoystick() {
  wlJoyFwd = wlJoyTurn = 0;
  wlBlocked = false;
  wlJoyLeft = wlJoyRight = 0;
  wlShowLeft = wlShowRight = 0;
  wlStraightTicks = 0;
  wlResend = true;
  rover.stop();
}

// What the phone's STOP does: halt the wheels, end the running action and
// the buzzer. A sketch can call it too (the unboxing demo's OK button does).
void stopAll() {
  wlStop = true;
  wlStopJoystick();
  noTone(RR_PIN_BUZZER);
}

// Reads what has arrived. Returns true once a whole line is in wlLine.
bool wlReadLine() {
  while (wifiSerial.available()) {
    char c = (char)wifiSerial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (wlLen == 0) continue;
      wlLine[wlLen] = '\0';
      wlLen = 0;
      wlLastLineMs = millis();
      return true;
    }
    // SoftwareSerial cannot listen while it is sending, so a byte that arrives
    // then comes out garbled. Garbled bytes are almost never plain text: drop them.
    if (c < ' ' || c > '~') continue;
    if (wlLen < sizeof(wlLine) - 1) wlLine[wlLen++] = c;  // longer lines are cut
  }
  return false;
}

// Status lines from the WiFi module about its own network
void wlNetLine(const char* line) {
  if (wlIs(line, PSTR("NET BOOT"))) {
    wlHomeIp[0] = '\0';
    wlCloud = false;
  } else if (wlStarts(line, PSTR("NET WIFI UP "))) {
    strncpy(wlHomeIp, line + 12, sizeof(wlHomeIp) - 1);
    wlHomeIp[sizeof(wlHomeIp) - 1] = '\0';
  } else if (wlIs(line, PSTR("NET WIFI DOWN"))) {
    wlHomeIp[0] = '\0';
    wlCloud = false;
  } else if (wlIs(line, PSTR("NET MQTT UP"))) {
    wlCloud = true;
  } else if (wlIs(line, PSTR("NET MQTT DOWN"))) {
    wlCloud = false;
  } else if (wlStarts(line, PSTR("NET STATUS "))) {
    // "NET STATUS internet wifi=up ip=192.168.1.23 mqtt=up"
    const char* ip = strstr_P(line, PSTR(" ip="));
    wlHomeIp[0] = '\0';
    if (ip && ip[4] != '-') {
      ip += 4;
      uint8_t i = 0;
      while (ip[i] && ip[i] != ' ' && i < sizeof(wlHomeIp) - 1) { wlHomeIp[i] = ip[i]; i++; }
      wlHomeIp[i] = '\0';
    }
    wlCloud = strstr_P(line, PSTR(" mqtt=up")) != nullptr;
  }
}

// Runs one of your functions, marked busy so a second one cannot start on top.
// kind: 'B' button, 'C' console, 'M' cloud message. fromPhone: the action ends
// if the phone goes silent (not for one started on the rover, e.g. by IR).
void wlRun(char kind, const char* arg, bool fromPhone = true) {
  strncpy(wlArg, arg, sizeof(wlArg) - 1);
  wlArg[sizeof(wlArg) - 1] = '\0';

  wlBusy++;
  wlInAction = fromPhone;
  wlStop = false;

  if (kind == 'B') {
    // The phone's button names, turned into friendlier ones
    const char* name = wlArg;
    if (wlIs(name, PSTR("LIGHTMODE"))) name = "LIGHTS";
    else if (wlIs(name, PSTR("G1"))) name = "A";
    else if (wlIs(name, PSTR("G2"))) name = "B";
    wlMode = wlIs(name, PSTR("DANCE")) ? "2" : "busy";
    wlActivity = name;
    wlSendStatus();
    onButton(name);
  } else if (kind == 'C') {
    wlMode = "busy";
    wlActivity = wlArg;
    onConsole(wlArg);
  } else {
    // "in/lights red" -> topic "lights", data "red"
    char* topic = wlArg;
    char* data = strchr(wlArg, ' ');
    if (data) *data++ = '\0'; else data = (char*)"";
    if (wlStarts(topic, PSTR("in/"))) topic += 3;
    wlMode = "busy";
    wlActivity = "Internet message";
    onCloudMessage(topic, data);
  }

  wlBusy--;
  wlInAction = false;
  if (wlStop) wlStopJoystick();   // the action was cut short: make sure we halt
  wlStop = false;
  wlMode = "0";
  wlActivity = nullptr;
  wlSendStatus();
}

// Where a joystick line starts. Normally at the beginning, but a line that
// arrived while the Kypruino was talking can carry junk in front
// ("jjRM -204 4 38") - then the check number decides if the rest is good.
const char* wlFindJoystick(const char* line) {
  for (const char* p = line; (p = strchr(p, 'M')) != nullptr; p++) {
    if (p[1] == ' ' || p[1] == '-' || isdigit(p[1])) return p;
  }
  return nullptr;
}

// Joystick: "M <left> <right> <check>", each -255..255. The check number
// lets us throw away a line the serial link misheard ("M 229 229" arriving
// as "M 29 229" would spin the rover). Lines without one come from the
// internet remote, which the WiFi module re-writes; they are only trusted
// when nothing was in front of them.
void wlJoystickLine(const char* m, bool salvaged) {
  char *mid, *end;
  long a = strtol(m + 1, &mid, 10);
  long b = strtol(mid, &end, 10);
  if (mid == m + 1 || end == mid) { wlBadLines++; return; }   // damaged, the next one follows
  if (*end != '\0' || salvaged) {
    char* last;
    long check = strtol(end, &last, 10);
    if (last == end || *last != '\0' || check != (a * 7 + b * 13 + 5100) % 97) { wlBadLines++; return; }
  }
  a = constrain(a, -255L, 255L) * joystickMaxSpeed / 255;
  b = constrain(b, -255L, 255L) * joystickMaxSpeed / 255;

  // driveFor() is steering: moving the joystick takes over and ends it
  if (wlOwnsMotors) {
    if (a || b) wlStop = true;
    return;
  }

  // The dashboard's L and R are the other way round from the V3 wheels
  // (its mixer was written for the V1 rover's wiring), so b is left, a right.
  // Split into forwards + turning.
  //   Spinning on the spot (knob along the flat edge): full power, the
  //   wheels need it to scrub round.
  //   Driving along (knob up in the dome): gentler steering, more so at speed.
  long forward = (a + b) / 2;
  long absF = abs(forward), absT = abs(b - a) / 2;
  long speedPct = absF * 100 / max(joystickMaxSpeed, 1);                 // 0..100
  long drivePct = turnStrength * (200 - speedPct) / 200;                  // full .. half
  long along = absF + absT ? min(100L, absF * 300 / (absF + absT)) : 0;  // 0 = spin, 100 = driving
  long turnPct = 100 - (100 - drivePct) * along / 100;
  wlJoyFwd = forward;
  wlJoyTurn = (b - a) / 2 * turnPct / 100;
  wlJoyMs = millis();
  wlDriveSentMs = wlJoyMs;
  wlApplyJoystick();
}

void wlHandleLine(const char* line) {
  // Status from the WiFi module itself
  if (wlStarts(line, PSTR("NET"))) {
    wlNetLine(line);
    return;
  }

  // STOP button, or the phone disconnected. Also when junk came in front of
  // it ("*PUdaJjjzSTOP"): stopping by mistake is harmless, missing one is not.
  if (strstr_P(line, PSTR("STOP"))) {
    stopAll();
    return;
  }

  // Everything else comes from a phone
  wlPhoneMs = millis();
  wlPhoneSeen = true;

  // Keep-alive, twice a second (also when a letter got lost: "PINGPING")
  if (wlStarts(line, PSTR("PIN")) || strstr_P(line, PSTR("PING"))) return;

  // Piano on the phone: "NOTE <Hz>" plays a note, "NOTE 0" stops. Each note
  // ends by itself after a second; the phone repeats it while a key is held.
  if (wlStarts(line, PSTR("NOTE "))) {
    int hz = atoi(line + 5);
    if (hz >= 100 && hz <= 5000) tone(RR_PIN_BUZZER, hz, 1000);
    else noTone(RR_PIN_BUZZER);
    return;
  }

  // Obstacle guard on / off: "SAFE 1" / "SAFE 0"
  if (wlStarts(line, PSTR("SAFE "))) {
    wlGuard = line[5] == '1';
    wlSidesSent = 255;   // report the new state to the phone straight away
    return;
  }

  // The speed display on the phone was tapped: "ODO 0" starts the distance again
  if (wlStarts(line, PSTR("ODO"))) {
    resetTrip();
    return;
  }

  if (wlStarts(line, PSTR("CALL "))) {
    if (wlBusy) { wlStop = true; return; }   // pressing a button again stops the action
    wlRun('B', line + 5);
    return;
  }

  if (wlStarts(line, PSTR("MSG "))) {
    if (!wlBusy) wlRun('M', line + 4);
    return;
  }

  const char* m = wlFindJoystick(line);
  if (m) {
    wlJoystickLine(m, m != line);
    return;
  }

  // Anything else was typed into the phone's Console. Typed words start with
  // a small letter or a digit; anything else is a damaged line - count it
  // (the "link" word shows the count) instead of answering it.
  if (!islower(line[0]) && !isdigit(line[0])) { wlBadLines++; return; }
  if (wlBusy) { say(F("busy - press STOP or a button first")); return; }
  wlRun('C', line);
}

// Everything the link does regularly. Called by wifiUpdate(), waitFor() and driveFor().
void wlService() {
  while (wlReadLine()) wlHandleLine(wlLine);

  unsigned long now = millis();

  // Joystick: re-check the obstacle guard 20 times a second, stop if it went quiet.
  // The motor chip has no failsafe of its own since Motion 1.12: this
  // dead-man and the STOP on disconnect are what stop the rover.
  if (!wlOwnsMotors && (wlJoyFwd || wlJoyTurn)) {
    if (now - wlJoyMs > JOYSTICK_TIMEOUT_MS) {
      wlStopJoystick();
    } else if (now - wlDriveSentMs >= DRIVE_REPEAT_MS) {
      wlDriveSentMs = now;
      wlApplyJoystick();
    }
  }

  // The phone went away in the middle of an action
  if (wlInAction && now - wlLastLineMs > LINK_TIMEOUT_MS) wlStop = true;

  if (now - wlStatusMs >= STATUS_MS) wlSendStatus();

  // Front distance and the left/right obstacle sensors: measure 4 times a
  // second (10 with the guard on), tell the phone when something changes and
  // once a second anyway, so it knows the reading is fresh.
  // "D 42 10 1" = 42 cm ahead, something on the left, right side clear, guard on.
  if (now - wlDistanceMs >= (wlGuard ? GUARD_DISTANCE_MS : DISTANCE_MS)) {
    wlDistanceMs = now;
    if (rover.sensors.poll()) {
      wlDistanceCm = rover.sensors.hasEcho() ? (rover.distanceMm() + 5) / 10 : -1;
      wlObstacleLeft = rover.obstacleLeft();
      wlObstacleRight = rover.obstacleRight();
    }
    uint8_t sides = (wlObstacleLeft ? 2 : 0) | (wlObstacleRight ? 1 : 0);
    bool changed = wlDistanceCm != wlDistanceSent || sides != wlSidesSent;
    if ((changed && now - wlDistanceSentMs >= 200) || wlSidesSent == 255 || now - wlDistanceSentMs >= 1000) {
      wlDistanceSent = wlDistanceCm;
      wlSidesSent = sides;
      wlDistanceSentMs = now;
      wlTalk();
      wifiSerial.print(F("D "));
      wifiSerial.print(wlDistanceCm);
      wifiSerial.print(wlObstacleLeft ? F(" 1") : F(" 0"));
      wifiSerial.print(wlObstacleRight ? '1' : '0');
      wifiSerial.print(wlGuard ? F(" 1\n") : F(" 0\n"));
    }
  }

  // Speed and distance driven, twice a second from the wheel encoders. The
  // phone is told when either changes, and every 2 seconds anyway.
  // "V 23 1234" = 23 cm/s forwards, 1234 cm driven since the last reset.
  if (now - wlTripMs >= TRIP_MS) {
    unsigned long ms = now - wlTripMs;
    wlTripMs = now;
    if (rover.motion.poll()) {
      long l = rover.leftTicks(), r = rover.rightTicks();
      long dl = l - wlEncLeft, dr = r - wlEncRight;
      wlEncLeft = l;
      wlEncRight = r;
      // A jump no wheel can make means the sketch zeroed the encoders: skip it
      if (wlEncValid && abs(dl) < 5000 && abs(dr) < 5000) {
        wlTripTicks2 += abs(dl + dr);
        long ticksPerSec = (dl + dr) * 500 / (long)ms;   // the average of the two wheels
        wlSpeedCms = ticksPerSec < 0 ? -(int)wlTicksToCm(-ticksPerSec) : (int)wlTicksToCm(ticksPerSec);
      }
      wlEncValid = true;
    }
    long trip = wifiTripCm();
    if (wifiPhoneConnected() &&
        (wlSpeedCms != wlSpeedSent || trip != wlTripSent || now - wlTripSentMs >= TRIP_SEND_MS)) {
      wlSpeedSent = wlSpeedCms;
      wlTripSent = trip;
      wlTripSentMs = now;
      wlTalk();
      wifiSerial.print(F("V "));
      wifiSerial.print(wlSpeedCms);
      wifiSerial.print(' ');
      wifiSerial.print(trip);
      wifiSerial.print('\n');
    }
  }

  if (wlBusy && whileBusy) whileBusy();

  screenTick();
}

// ---- the functions your sketch calls ---------------------------------------

void wifiBegin() {
  // The motion chip's own wheel size, for the speed and distance driven
  uint16_t circ = rover.motion.read16(RR_M_WHEEL_CIRC);
  uint16_t ticks = rover.motion.ticksPerRev();
  if (circ > 0 && ticks > 0) { wlWheelCirc100 = circ; wlTicksPerRev = ticks; }

  wifiSerial.begin(WIFI_BAUD);
  wlLastLineMs = millis();
  wlSendStatus();
  wlTalk();
  wifiSerial.print(F("NET\n"));   // ask the WiFi module how its network is doing
}

// Call this at the top of loop(). Keep the rest of loop() quick.
void wifiUpdate() {
  wlStop = false;
  wlService();
}

// Wait, while still listening to the phone. Use this instead of delay().
// Returns false if the action was stopped (STOP, another button, phone gone);
// after that every waitFor() and driveFor() returns straight away.
bool waitFor(unsigned long ms) {
  if (wlStop) return false;
  wlBusy++;
  unsigned long start = millis();
  while (millis() - start < ms && !wlStop) wlService();
  wlBusy--;
  return !wlStop;
}

// Drive for a while. Speeds are -1000 (full backwards) to 1000 (full forwards).
// Returns false if the action was stopped; the rover is stopped either way.
bool driveFor(int leftSpeed, int rightSpeed, unsigned long ms) {
  if (wlStop) return false;
  wlBusy++;
  wlOwnsMotors = true;
  wlJoyLeft = wlJoyRight = 0;
  wlShowLeft = leftSpeed;
  wlShowRight = rightSpeed;
  unsigned long start = millis();
  unsigned long lastSent = 0;
  while (millis() - start < ms && !wlStop) {
    if (lastSent == 0 || millis() - lastSent >= DRIVE_REFRESH_MS) {
      lastSent = millis();
      rover.drive(leftSpeed, rightSpeed);
    }
    wlService();
  }
  rover.stop();
  wlShowLeft = wlShowRight = 0;
  wlResend = true;
  wlOwnsMotors = false;
  wlBusy--;
  return !wlStop;
}

// ---- for driving from something else as well (the unboxing demo's IR remote) ----

// Drive as the phone's joystick does: forward and turn -1000..1000 (turn > 0
// = right). Same obstacle guard, same straight hold when turn is 0, same
// motor bars. Call it again at least every 0.8 s while driving, and with
// (0, 0) to stop. During driveFor() it ends that action instead.
void joystickDrive(int forward, int turn) {
  if (wlOwnsMotors) {
    if (forward || turn) wlStop = true;
    return;
  }
  wlJoyFwd = forward;
  wlJoyTurn = turn;
  wlJoyMs = millis();
  wlDriveSentMs = wlJoyMs;
  wlApplyJoystick();
}

// Run one of the phone's buttons ("LIGHTS", "DANCE", "A", ...) as if it was
// pressed on the phone: onButton() is called and the phone shows it running.
// While something is running it stops that instead, like a phone button.
void runButton(const char* name) {
  if (wlBusy) { wlStop = true; return; }
  wlRun('B', name, false);
}
