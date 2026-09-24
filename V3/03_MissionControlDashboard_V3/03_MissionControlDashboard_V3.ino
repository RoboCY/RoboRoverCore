// 03_MissionControlDashboard_V3 - every sensor on the screen, lights and driving on the remote.
// The rover moves while you hold an arrow: put it on blocks, or give it
// floor space. Needs the U8g2 library.
//
//   OK        next screen page: sensors, tilt, wheels, battery
//   arrows    hold to drive or spin, let go to stop
//   1 to 6    red, green, blue, magenta, yellow, cyan
//   7         rainbow colours
//   8         one random colour
//   9         a random colour for each light
//   0         lights off
//   *  #      low beep, high beep

#include <RoboRoverCore3.h>
#include <U8g2lib.h>
#include <Adafruit_NeoPixel.h>

// The IR receiver pin must be set before this include.
#define IR_RECEIVE_PIN RR_PIN_IR_RECV
#include <TinyIRReceiver.hpp>

RoboRoverCore3          rover;
RoboRoverCore3Accel     accel;
RoboRoverCore3Battery   battery;

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C screen(U8G2_R0);
Adafruit_NeoPixel leds(RR_NEOPIXEL_COUNT, RR_PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// Remote buttons
const uint8_t UP    = 0x18;
const uint8_t DOWN  = 0x52;
const uint8_t LEFT  = 0x08;
const uint8_t RIGHT = 0x5A;
const uint8_t OK    = 0x1C;
const uint8_t STAR  = 0x16;
const uint8_t HASH  = 0x0D;
const uint8_t NUM_0 = 0x19;
const uint8_t NUM_1 = 0x45;
const uint8_t NUM_2 = 0x46;
const uint8_t NUM_3 = 0x47;
const uint8_t NUM_4 = 0x44;
const uint8_t NUM_5 = 0x40;
const uint8_t NUM_6 = 0x43;
const uint8_t NUM_7 = 0x07;
const uint8_t NUM_8 = 0x15;
const uint8_t NUM_9 = 0x09;

// Wheel speeds in ticks per second; 574 ticks is one turn of the wheel.
const int MOVE_SPEED = 2500;
const int SPIN_SPEED = 1500;

// A held button repeats about nine times a second. If nothing arrives for
// this long, the button was let go.
const unsigned long HOLD_TIMEOUT_MS = 150;

// Screen pages
const int PAGE_SENSORS = 0;
const int PAGE_TILT    = 1;
const int PAGE_WHEELS  = 2;
const int PAGE_BATTERY = 3;
const int PAGES        = 4;

int page = PAGE_SENSORS;

int leftSpeed = 0;
int rightSpeed = 0;
unsigned long lastDriveButtonMs = 0;
unsigned long lastDriveSentMs = 0;
unsigned long lastDrawMs = 0;
unsigned long lastBatteryMs = 0;

bool haveLine, haveAccel, haveBattery;

// ---------------------------------------------------------------- lights

void allLeds(uint8_t r, uint8_t g, uint8_t b) {
  leds.fill(leds.Color(r, g, b));
  leds.show();
}

void rainbowLeds() {
  for (int i = 0; i < RR_NEOPIXEL_COUNT; i++) {
    leds.setPixelColor(i, leds.ColorHSV(i * 65536L / RR_NEOPIXEL_COUNT, 255, 60));
  }
  leds.show();
}

void randomLeds() {
  for (int i = 0; i < RR_NEOPIXEL_COUNT; i++) {
    leds.setPixelColor(i, random(0, 80), random(0, 80), random(0, 80));
  }
  leds.show();
}

// ---------------------------------------------------------------- remote

void checkRemote(unsigned long now) {
  if (!TinyReceiverDecode()) return;

  uint8_t button = TinyIRReceiverData.Command;
  bool held = TinyIRReceiverData.Flags & IRDATA_FLAGS_IS_REPEAT;

  // Arrows keep working while held; they are what makes the rover move.
  if (button == UP) {
    leftSpeed = MOVE_SPEED;
    rightSpeed = MOVE_SPEED;
    lastDriveButtonMs = now;
  } else if (button == DOWN) {
    leftSpeed = -MOVE_SPEED;
    rightSpeed = -MOVE_SPEED;
    lastDriveButtonMs = now;
  } else if (button == LEFT) {
    leftSpeed = -SPIN_SPEED;
    rightSpeed = SPIN_SPEED;
    lastDriveButtonMs = now;
  } else if (button == RIGHT) {
    leftSpeed = SPIN_SPEED;
    rightSpeed = -SPIN_SPEED;
    lastDriveButtonMs = now;
  }

  // Every other button acts once per press, so holding OK does not flick
  // through all the pages.
  else if (held) {
    return;
  } else if (button == OK) {
    page = (page + 1) % PAGES;
  } else if (button == NUM_1) {
    allLeds(60, 0, 0);
  } else if (button == NUM_2) {
    allLeds(0, 60, 0);
  } else if (button == NUM_3) {
    allLeds(0, 0, 60);
  } else if (button == NUM_4) {
    allLeds(60, 0, 60);
  } else if (button == NUM_5) {
    allLeds(60, 60, 0);
  } else if (button == NUM_6) {
    allLeds(0, 60, 60);
  } else if (button == NUM_7) {
    rainbowLeds();
  } else if (button == NUM_8) {
    allLeds(random(0, 80), random(0, 80), random(0, 80));
  } else if (button == NUM_9) {
    randomLeds();
  } else if (button == NUM_0) {
    allLeds(0, 0, 0);
  } else if (button == STAR) {
    tone(RR_PIN_BUZZER, 350, 90);
  } else if (button == HASH) {
    tone(RR_PIN_BUZZER, 1200, 90);
  }
}

// ---------------------------------------------------------------- drawing

// A bar that fills from the bottom: value from 0 to maxValue.
void drawBar(int x, int y, int w, int h, long value, long maxValue) {
  screen.drawFrame(x, y, w, h);
  int fill = constrain(value * (h - 2) / maxValue, 0, h - 2);
  if (fill > 0) screen.drawBox(x + 1, y + h - 1 - fill, w - 2, fill);
}

// A bar that fills left or right from the middle: value from -range to range.
void drawSignedBar(int x, int y, int w, int h, long value, long range) {
  screen.drawFrame(x, y, w, h);
  int middle = x + w / 2;
  screen.drawVLine(middle, y + 1, h - 2);

  int halfW = (w - 2) / 2;
  int fill = constrain(abs(value) * halfW / range, 0, halfW);
  if (value >= 0) screen.drawBox(middle + 1, y + 1, fill, h - 2);
  else            screen.drawBox(middle - fill, y + 1, fill, h - 2);
}

// A little sun, to mark the light sensor bars.
void drawSun(int cx, int cy) {
  screen.drawCircle(cx, cy, 2);
  screen.drawHLine(cx - 4, cy, 2);
  screen.drawHLine(cx + 3, cy, 2);
  screen.drawVLine(cx, cy - 4, 2);
  screen.drawVLine(cx, cy + 3, 2);
}

void drawCentred(const char *text, int xMin, int xMax, int y) {
  int x = xMin + (xMax - xMin - screen.getStrWidth(text)) / 2;
  screen.drawStr(x, y, text);
}

//  [L box]  (sun)      distance      (sun)  [R box]
//           light bar  4 3 2 1 0  light bar
void drawSensorsPage() {
  rover.sensors.poll();
  if (haveLine) rover.line.poll();

  const int boxW = 10, boxH = 10;
  const int lightW = 10;
  const int lineW = 6, lineGap = 2;
  const int lineGroupW = 5 * lineW + 4 * lineGap;
  const int barsTop = 14;

  int gap = (128 - (2 * boxW + 2 * lightW + lineGroupW)) / 4;
  int xBoxL   = 0;
  int xLightL = xBoxL + boxW + gap;
  int xLine   = xLightL + lightW + gap;
  int xLightR = xLine + lineGroupW + gap;
  int xBoxR   = xLightR + lightW + gap;

  // Obstacle sensors: an empty box is clear, a filled box sees something.
  screen.drawFrame(xBoxL, 0, boxW, boxH);
  screen.drawFrame(xBoxR, 0, boxW, boxH);
  if (rover.obstacleLeft())  screen.drawBox(xBoxL + 1, 1, boxW - 2, boxH - 2);
  if (rover.obstacleRight()) screen.drawBox(xBoxR + 1, 1, boxW - 2, boxH - 2);
  screen.drawStr(xBoxL + 3, 12, "L");
  screen.drawStr(xBoxR + 3, 12, "R");

  // Ultrasonic distance.
  char text[12];
  if (rover.sensors.hasEcho()) {
    snprintf(text, sizeof(text), "%dcm", (rover.distanceMm() + 5) / 10);
  } else {
    strcpy(text, "N/A");
  }
  drawCentred(text, xBoxL + boxW + 1, xBoxR - 1, 1);

  // Light sensors, 0 to 1023. More light fills the bar higher.
  drawSun(xLightL + lightW / 2, 4);
  drawSun(xLightR + lightW / 2, 4);
  drawBar(xLightL, barsTop, lightW, 32 - barsTop, rover.lightLeft(), 1023);
  drawBar(xLightR, barsTop, lightW, 32 - barsTop, rover.lightRight(), 1023);

  // Line sensors, 0 to 4095. White fills the bar, black leaves it empty.
  // Sensor 4 is on the far left and sensor 0 on the far right, so they are
  // drawn from 4 down to 0.
  for (int place = 0; place < 5; place++) {
    int sensor = 4 - place;
    int x = xLine + place * (lineW + lineGap);
    drawBar(x, barsTop, lineW, 11, haveLine ? rover.line.raw(sensor) : 0, 4095);
    char label[2] = { char('0' + sensor), 0 };
    screen.drawStr(x + 1, 25, label);
  }
}

void drawTiltPage() {
  if (haveAccel) accel.poll();

  screen.drawStr(0, 0, "Tilt (milli-g)");
  if (!haveAccel) {
    screen.drawStr(0, 12, "accelerometer not found");
    return;
  }

  // Lying flat, Z shows about +1000: that is gravity pulling down.
  const char *names[3] = { "X", "Y", "Z" };
  int values[3] = { accel.x(), accel.y(), accel.z() };
  for (int i = 0; i < 3; i++) {
    int y = 8 + i * 8;
    screen.drawStr(0, y, names[i]);
    drawSignedBar(10, y, 70, 7, values[i], 1000);
    screen.setCursor(86, y);
    screen.print(values[i]);
  }
}

void drawWheelsPage() {
  rover.motion.poll();

  screen.drawStr(0, 0, "Wheels (encoder ticks)");
  screen.drawStr(0, 10, "Left");
  screen.setCursor(40, 10);
  screen.print(rover.leftTicks());
  screen.drawStr(0, 19, "Right");
  screen.setCursor(40, 19);
  screen.print(rover.rightTicks());
}

void drawBatteryPage(unsigned long now) {
  // The battery changes slowly, so once a second is plenty.
  if (haveBattery && now - lastBatteryMs >= 1000) {
    lastBatteryMs = now;
    battery.poll();
  }

  screen.drawStr(0, 0, "Battery");
  if (!haveBattery) {
    screen.drawStr(0, 12, "fuel gauge not found");
    return;
  }

  char text[12];
  screen.setFont(u8g2_font_10x20_tf);
  snprintf(text, sizeof(text), "%d%%", battery.percent());
  screen.drawStr(0, 11, text);
  screen.setFont(u8g2_font_5x8_tf);

  snprintf(text, sizeof(text), "%d.%02d V", battery.millivolts() / 1000,
           battery.millivolts() % 1000 / 10);
  screen.drawStr(60, 12, text);

  // Negative current: the battery is powering the rover.
  snprintf(text, sizeof(text), "%d mA", battery.milliamps());
  screen.drawStr(60, 22, text);
}

// ---------------------------------------------------------------- main

void setup() {
  leds.begin();
  leds.clear();
  leds.show();
  pinMode(RR_PIN_BUZZER, OUTPUT);

  rover.begin();              // also sets both wheel counts to 0

  haveLine    = rover.line.channels() > 0;
  haveAccel   = accel.begin() == RR_OK;
  haveBattery = battery.begin() == RR_OK;

  screen.begin();
  screen.setBusClock(100000);   // match the speed of the rover's other chips
  screen.setFont(u8g2_font_5x8_tf);
  screen.setFontPosTop();       // text is placed by its top-left corner

  initPCIInterruptForTinyReceiver();
}

void loop() {
  unsigned long now = millis();

  checkRemote(now);

  if (now - lastDriveButtonMs > HOLD_TIMEOUT_MS) {
    leftSpeed = 0;
    rightSpeed = 0;
  }

  // Send the speeds about 20 times a second, so a button press or release
  // takes effect quickly. The same speeds sent again keep the wheels in
  // step, so an arrow held down drives straight.
  if (now - lastDriveSentMs >= 50) {
    lastDriveSentMs = now;
    rover.setSpeed(leftSpeed, rightSpeed);
  }

  // Redraw the screen ten times a second.
  if (now - lastDrawMs >= 100) {
    lastDrawMs = now;
    screen.clearBuffer();

    if (page == PAGE_SENSORS)     drawSensorsPage();
    else if (page == PAGE_TILT)   drawTiltPage();
    else if (page == PAGE_WHEELS) drawWheelsPage();
    else                          drawBatteryPage(now);

    screen.sendBuffer();
  }
}
