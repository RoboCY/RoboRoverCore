// Screen.h - what the rover's little screen (128 x 32 dots) shows.
// Change it if you like!
//
//   Nobody driving yet:          how to connect, and that the IR remote works
//   Phone joined or IR remote:   motor bars, what the rover is doing,
//                                distance ahead, speed, last thing said
//   Always:                      battery charge in the top right corner
//
// 08_CommandFusionHub_V3 has the same file without the IR remote parts.
//
// Text positions are in dots: x from the left (0..127), y from the top (0..31).
// Each line of text is 8 dots high, so there is room for 4 lines.

#pragma once
#include <U8g2lib.h>

// "_1_" draws the screen in 4 small strips, which needs much less memory
U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C screen(U8G2_R0);

int screenBattery = -1;       // percent, -1 = unknown
unsigned long screenDrawnMs = 0, screenBatteryMs = 0;

void screenBegin() {
  screen.begin();
  screen.setBusClock(100000);   // match the speed of the rover's other chips
  screen.setFont(u8g2_font_5x8_tf);
  screen.setFontPosTop();       // text is placed by its top-left corner
}

// A tall bar for one motor: fills up for forwards, down for backwards.
void drawSpeedBar(int x, int speed) {
  screen.drawFrame(x, 0, 6, 32);
  int h = abs(speed) * 15L / 1000;         // 0..15 dots
  if (speed > 0) screen.drawBox(x + 1, 16 - h, 4, h);
  if (speed < 0) screen.drawBox(x + 1, 16, 4, h);
  screen.drawHLine(x, 16, 6);              // middle line = stopped
}

// Text is written with setCursor(x, y) and then print(), like Serial.print().
// F("...") keeps the text in flash memory instead of the Kypruino's small RAM.

// Battery charge, top right: a small battery symbol that empties as the
// charge goes down, then the charge in percent (from the fuel gauge chip).
void drawBattery() {
  if (screenBattery < 0) return;
  int textW = screenBattery >= 100 ? 20 : screenBattery >= 10 ? 15 : 10;   // 5 dots per character
  int x = 128 - textW;
  screen.setCursor(x, 0);
  screen.print(screenBattery);
  screen.print('%');

  int bx = x - 15;                                  // the symbol: 12 x 7 dots
  screen.drawFrame(bx, 0, 12, 7);
  screen.drawBox(bx + 12, 2, 1, 3);                 // the + end
  screen.drawBox(bx + 1, 1, screenBattery / 10, 5); // fill: 0..10 dots
}

// Speed, right-aligned: "23cm/s". 5 dots per character.
void drawSpeed(int y) {
  int v = wifiSpeedCms();
  int chars = 4 + (v < 0) + (abs(v) >= 100 ? 3 : abs(v) >= 10 ? 2 : 1);
  screen.setCursor(128 - 5 * chars, y);
  screen.print(v);
  screen.print(F("cm/s"));
}

void drawScreen() {
  drawBattery();   // on both screens

  if (wifiPhoneConnected() || remoteInUse()) {
    // Motor bars on the left, like the ones on the phone
    drawSpeedBar(0, wifiLeftSpeed());
    drawSpeedBar(8, wifiRightSpeed());

    screen.setCursor(18, 0);
    screen.print(wifiPhoneConnected() ? F("Phone connected") : F("IR remote"));

    screen.setCursor(18, 8);
    screen.print(wifiActivity());

    screen.setCursor(18, 16);
    if (wifiDistanceCm() >= 0 && wifiDistanceCm() <= 100) {   // further than 1 m: "clear"
      screen.print(F("Front: "));
      screen.print(wifiDistanceCm());
      screen.print(F(" cm"));
    } else {
      screen.print(F("Front: clear"));
    }
    if (wifiSpeedCms()) {
      drawSpeed(16);
    } else if (wifiCloudOnline()) {
      screen.setCursor(103, 16);
      screen.print(F("cloud"));
    }

    screen.setCursor(18, 24);
    screen.print(wifiLastSaid());
  } else {
    // Waiting: show how to connect. The dots move so you can see it is alive.
    screen.setCursor(0, 0);
    screen.print(F("No phone yet"));
    int dots = (millis() / 400) % 4;
    for (int i = 0; i < dots; i++) screen.print('.');

    screen.setCursor(0, 8);
    screen.print(F("1. Join WiFi RoboRover-"));
    screen.setCursor(0, 16);
    screen.print(F("2. Open 192.168.4.1"));

    // On a home WiFi too? Then that address works as well.
    // Otherwise remind that the IR remote works without any phone.
    screen.setCursor(0, 24);
    if (wifiHomeIp()[0]) {
      screen.print(F("or http://"));
      screen.print(wifiHomeIp());
    } else {
      screen.print(F("Or use the IR remote"));
    }
  }
}

// A number that changes whenever anything shown on the screen changes.
// If you draw something new in drawScreen(), add it here too.
unsigned long screenLastState = 1;

unsigned long textSum(const char* s) {
  unsigned long sum = 0;
  while (*s) sum = sum * 31 + *s++;
  return sum;
}

unsigned long screenState() {
  unsigned long s = wifiPhoneConnected() * 2 + remoteInUse();
  s = s * 31 + abs(wifiLeftSpeed()) * 15L / 1000 * (wifiLeftSpeed() < 0 ? -1 : 1);
  s = s * 31 + abs(wifiRightSpeed()) * 15L / 1000 * (wifiRightSpeed() < 0 ? -1 : 1);
  s = s * 31 + wifiDistanceCm();
  s = s * 31 + wifiSpeedCms();
  s = s * 31 + screenBattery;
  s = s * 31 + wifiCloudOnline();
  s = s * 31 + (wifiPhoneConnected() ? 0 : (millis() / 400) % 4);   // the moving dots
  s = s * 31 + textSum(wifiActivity());
  s = s * 31 + textSum(wifiLastSaid());
  s = s * 31 + textSum(wifiHomeIp());
  return s;
}

// Called by WiFiLink.h many times a second - also while driveFor() and
// waitFor() are running, so the screen stays live during a dance.
void screenTick() {
  unsigned long now = millis();

  // The battery changes slowly: every 5 seconds is plenty
  if (haveBattery && (screenBatteryMs == 0 || now - screenBatteryMs >= 5000)) {
    screenBatteryMs = now;
    battery.poll();
    screenBattery = battery.percent();
  }

  // Redraw at most 4 times a second, and only when something changed - once
  // a second while the wheels turn. A picture is 512 bytes on the I2C bus, at
  // least 46 ms, and meanwhile the Kypruino does not answer the joystick or
  // the remote. A busy bus also makes the WiFi link more likely to mishear
  // a letter.
  unsigned long gap = (wifiLeftSpeed() || wifiRightSpeed()) ? 1000 : 250;
  if (now - screenDrawnMs < gap) return;
  unsigned long state = screenState();
  if (state == screenLastState) return;
  screenLastState = state;
  screenDrawnMs = now;
  screen.firstPage();
  do {
    drawScreen();
  } while (screen.nextPage());
}
