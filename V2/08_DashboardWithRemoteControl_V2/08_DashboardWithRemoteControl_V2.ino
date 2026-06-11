/*
  ============================================================
  Project: IR Remote + OLED Graph Dashboard (RoboRover Core 2)
  Board: Kypruino UNO+ v0.8
  Platform: RoboRover Core 2
  ============================================================
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>

#define IR_RECEIVE_PIN A3
#include <TinyIRReceiver.hpp>

// -------------------- OLED --------------------
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   32
#define OLED_RESET     -1
#define OLED_ADDR      0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// -------------------- I2C ADDRESSES --------------------
#define EXP_ADDR   0x20
#define TLA_ADDR   0x14
#define LIS_ADDR   0x19

// -------------------- TLA2528 --------------------
#define TLA_OP_WRITE_REG     0x08
#define TLA_REG_CHANNEL_SEL  0x11

// -------------------- PINS --------------------
#define PIN_TRIG      4
#define PIN_ECHO      5
#define PIN_L_LDR     A6
#define PIN_R_LDR     A7
#define PIN_BUZZER    9
#define PIN_NEOPIXEL  8
#define PIN_AENBL     11
#define PIN_BENBL     10

// -------------------- LED CHAIN --------------------
#define LED_COUNT       6

Adafruit_NeoPixel pixels(LED_COUNT, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// -------------------- EXPANDER BITS --------------------
#define EXP_OIR_L  2
#define EXP_OIR_R  3

// -------------------- TIMING --------------------
#define DASH_UPDATE_MS     50
#define ULTRASONIC_MS     200
#define ACCEL_MS          100
#define ECHO_TIMEOUT_US 12000UL

unsigned long tDash  = 0;
unsigned long tUS    = 0;
unsigned long tAccel = 0;

// -------------------- REMOTE CONTROL --------------------
#define SPEED_MOVE  200
#define SPEED_SPIN  150
#define HOLD_TIMEOUT_MS 150

unsigned long lastArrowMs = 0;
int curA = 0;
int curB = 0;

// -------------------- DISPLAY PAGE --------------------
enum DisplayPage {
  PAGE_SENSORS = 0,
  PAGE_ACCEL   = 1
};

DisplayPage currentPage = PAGE_SENSORS;

#define CMD_TOGGLE_PAGE 0x1C   // OK button
#define CMD_LEDS_OFF    0x19   // 0 button

// -------------------- STATE --------------------
uint8_t expState = 0xFF;

float distanceCm = -1.0;
int ir[5] = {0, 0, 0, 0, 0};
int ldrL = 0, ldrR = 0;
int oirL = 0, oirR = 0;
int16_t accX = 0, accY = 0, accZ = 0;

// ============================================================
//                    EXPANDER HELPERS
// ============================================================
void expWrite(uint8_t v) {
  Wire.beginTransmission(EXP_ADDR);
  Wire.write(v);
  Wire.endTransmission();
  expState = v;
}

uint8_t expRead() {
  Wire.requestFrom(EXP_ADDR, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0xFF;
}

void setPhaseBits(bool p0, bool p1) {
  uint8_t v = expState;
  v &= 0b11111100;
  if (p0) v |= 0b00000001;
  if (p1) v |= 0b00000010;
  expWrite(v);
}

// ============================================================
//                     TLA2528 HELPERS
// ============================================================
void tlaWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(TLA_ADDR);
  Wire.write(TLA_OP_WRITE_REG);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint16_t tlaReadChannel(uint8_t channel) {
  channel &= 0x07;

  tlaWriteReg(TLA_REG_CHANNEL_SEL, channel);
  delayMicroseconds(30);

  Wire.requestFrom(TLA_ADDR, (uint8_t)2);
  if (Wire.available() >= 2) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    uint16_t raw = ((uint16_t)msb << 8) | lsb;
    return (raw >> 4) & 0x0FFF;
  }

  return 0;
}

int tlaTo1023(uint16_t raw12) {
  return (raw12 * 1023UL) / 4095UL;
}

// ============================================================
//                    LIS2DH12 HELPERS
// ============================================================
void lisWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(LIS_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void initAccelerometer() {
  lisWriteReg(0x20, 0x57); // 100 Hz, XYZ enable
  lisWriteReg(0x23, 0x88); // BDU + high resolution
}

void readAccelerometer() {
  Wire.beginTransmission(LIS_ADDR);
  Wire.write(0x28 | 0x80);
  Wire.endTransmission(false);

  Wire.requestFrom(LIS_ADDR, (uint8_t)6);
  if (Wire.available() >= 6) {
    uint8_t xl = Wire.read();
    uint8_t xh = Wire.read();
    uint8_t yl = Wire.read();
    uint8_t yh = Wire.read();
    uint8_t zl = Wire.read();
    uint8_t zh = Wire.read();

    int16_t rawX = (int16_t)((xh << 8) | xl);
    int16_t rawY = (int16_t)((yh << 8) | yl);
    int16_t rawZ = (int16_t)((zh << 8) | zl);

    accX = rawX >> 4;
    accY = rawY >> 4;
    accZ = rawZ >> 4;
  }
}

// ============================================================
//                      MOTOR CONTROL
// ============================================================
void applyMotors(int a, int b) {
  a = constrain(a, -255, 255);
  b = constrain(b, -255, 255);

  bool p0 = (a < 0);
  bool p1 = (b < 0);
  setPhaseBits(p0, p1);

  analogWrite(PIN_AENBL, (uint8_t)abs(a));
  analogWrite(PIN_BENBL, (uint8_t)abs(b));
}

// ============================================================
//                        BUZZER
// ============================================================
void shortBeep(uint16_t freqHz, uint16_t ms) {
  unsigned long periodUs = 1000000UL / freqHz;
  unsigned long halfUs   = periodUs / 2;

  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    digitalWrite(PIN_BUZZER, HIGH);
    delayMicroseconds(halfUs);
    digitalWrite(PIN_BUZZER, LOW);
    delayMicroseconds(halfUs);
  }
  delay(80);
}

// ============================================================
//                         LED HELPERS
// ============================================================
void ledsAll(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < LED_COUNT; i++) {
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }
  pixels.show();
}

void ledsOff() {
  pixels.clear();
  pixels.show();
}

// ============================================================
//                     DASHBOARD HELPERS
// ============================================================
int clampi(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void drawSun(int cx, int cy) {
  display.drawCircle(cx, cy, 2, SSD1306_WHITE);
  display.drawLine(cx - 4, cy,     cx - 3, cy,     SSD1306_WHITE);
  display.drawLine(cx + 3, cy,     cx + 4, cy,     SSD1306_WHITE);
  display.drawLine(cx,     cy - 4, cx,     cy - 3, SSD1306_WHITE);
  display.drawLine(cx,     cy + 3, cx,     cy + 4, SSD1306_WHITE);
}

void drawVBar(int x, int y, int w, int h, int value0_1023) {
  display.drawRect(x, y, w, h, SSD1306_WHITE);

  int fillH = (value0_1023 * (h - 2)) / 1023;
  fillH = clampi(fillH, 0, h - 2);

  int fillY = y + (h - 1) - fillH;
  if (fillH > 0) display.fillRect(x + 1, fillY, w - 2, fillH, SSD1306_WHITE);
}

void drawCenteredText(const char* s, int xMin, int xMax, int y) {
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(s, 0, y, &x1, &y1, &w, &h);
  int x = xMin + ((xMax - xMin) - (int)w) / 2;
  display.setCursor(x, y);
  display.print(s);
}

void drawSignedBarRaw(int x, int y, int w, int h, int16_t value, int16_t range) {
  display.drawRect(x, y, w, h, SSD1306_WHITE);

  int cx = x + w / 2;
  display.drawLine(cx, y + 1, cx, y + h - 2, SSD1306_WHITE);

  value = constrain(value, -range, range);
  int halfW = (w - 2) / 2;
  int fill = ((long)abs(value) * halfW) / range;

  if (value >= 0) {
    if (fill > 0) display.fillRect(cx + 1, y + 1, fill, h - 2, SSD1306_WHITE);
  } else {
    if (fill > 0) display.fillRect(cx - fill, y + 1, fill, h - 2, SSD1306_WHITE);
  }
}

// ============================================================
//                      SENSOR READ
// ============================================================
void readFastSensors() {
  for (int i = 0; i < 5; i++) {
    uint16_t raw12 = tlaReadChannel(i);
    ir[i] = tlaTo1023(raw12);
  }

  ldrL = analogRead(PIN_L_LDR);
  ldrR = analogRead(PIN_R_LDR);

  uint8_t want = expState | (1 << EXP_OIR_L) | (1 << EXP_OIR_R);
  if (want != expState) expWrite(want);

  uint8_t v = expRead();

  // Reversed obstacle logic:
  // raw bit 0 = obstacle present
  // raw bit 1 = no obstacle
  oirL = (((v >> EXP_OIR_L) & 1) == 0);
  oirR = (((v >> EXP_OIR_R) & 1) == 0);
}

void readUltrasonic() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long us = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);
  distanceCm = (us == 0) ? -1.0 : (us / 58.0);
}

// ============================================================
//                     DRAW DASHBOARD PAGE 1
// ============================================================
void drawSensorDashboard() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  const int boxW = 10, boxH = 10;
  const int yBox = 0;
  const int ySun = 0;
  const int yDist = 1;
  const int yLRLabel = 12;

  const int barsTop = 14;
  const int barsBottom = 31;
  const int barsH = barsBottom - barsTop + 1;

  const int ldrW = 10;
  const int irBarW = 6;
  const int irGap  = 2;
  const int irGroupW = (5 * irBarW) + (4 * irGap);
  const int irLabelY = 26;
  const int irBarsH = irLabelY - barsTop;

  int used = boxW + ldrW + irGroupW + ldrW + boxW;
  int gaps = 4;
  int gapW = (SCREEN_WIDTH - used) / gaps;

  int xBoxL = 0;
  int xLdrL = xBoxL + boxW + gapW;
  int xIR   = xLdrL + ldrW + gapW;
  int xLdrR = xIR + irGroupW + gapW;
  int xBoxR = xLdrR + ldrW + gapW;

  display.drawRect(xBoxL, yBox, boxW, boxH, SSD1306_WHITE);
  if (oirL) display.fillRect(xBoxL + 1, yBox + 1, boxW - 2, boxH - 2, SSD1306_WHITE);

  display.drawRect(xBoxR, yBox, boxW, boxH, SSD1306_WHITE);
  if (oirR) display.fillRect(xBoxR + 1, yBox + 1, boxW - 2, boxH - 2, SSD1306_WHITE);

  display.setCursor(xBoxL + 2, yLRLabel); display.print("L");
  display.setCursor(xBoxR + 2, yLRLabel); display.print("R");

  drawSun(xLdrL + ldrW / 2, ySun + 4);
  drawSun(xLdrR + ldrW / 2, ySun + 4);

  char buf[12];
  if (distanceCm < 0) snprintf(buf, sizeof(buf), "N/A");
  else snprintf(buf, sizeof(buf), "%dcm", (int)(distanceCm + 0.5f));

  drawCenteredText(buf, xBoxL + boxW + 1, xBoxR - 1, yDist);

  drawVBar(xLdrL, barsTop, ldrW, barsH, ldrL);
  drawVBar(xLdrR, barsTop, ldrW, barsH, ldrR);

  // Reverse screen order: leftmost is IR4, rightmost is IR0
  for (int screenPos = 0; screenPos < 5; screenPos++) {
    int sensorIndex = 4 - screenPos;
    int x = xIR + screenPos * (irBarW + irGap);

    drawVBar(x, barsTop, irBarW, irBarsH, ir[sensorIndex]);
    display.setCursor(x + 1, irLabelY);
    display.print(sensorIndex);
  }

  display.display();
}

// ============================================================
//                     DRAW DASHBOARD PAGE 2
// ============================================================
void drawAccelDashboard() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.print("Accelerometer");

  const int16_t range = 2048;

  display.setCursor(0, 10); display.print("X");
  drawSignedBarRaw(12, 9, 70, 8, accX, range);
  display.setCursor(88, 9); display.print(accX);

  display.setCursor(0, 19); display.print("Y");
  drawSignedBarRaw(12, 18, 70, 8, accY, range);
  display.setCursor(88, 18); display.print(accY);

  display.setCursor(0, 28); display.print("Z");
  drawSignedBarRaw(12, 27, 70, 8, accZ, range);
  display.setCursor(88, 27); display.print(accZ);

  display.display();
}

// ============================================================
//                         SETUP
// ============================================================
void setup() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  pinMode(PIN_AENBL, OUTPUT);
  pinMode(PIN_BENBL, OUTPUT);
  analogWrite(PIN_AENBL, 0);
  analogWrite(PIN_BENBL, 0);

  pinMode(IR_RECEIVE_PIN, INPUT);

  Serial.begin(115200);
  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    while (1);
  }

  initAccelerometer();

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("RRC2 Remote+OLED");
  display.display();
  delay(700);

  pixels.begin();
  pixels.setBrightness(255);
  ledsOff();

  expWrite(0xFF);
  applyMotors(0, 0);

  initPCIInterruptForTinyReceiver();
  enablePCIInterruptForTinyReceiver();
}

// ============================================================
//                          LOOP
// ============================================================
void loop() {
  unsigned long now = millis();

  if (TinyReceiverDecode()) {
    uint8_t cmd = TinyIRReceiverData.Command;

    if (cmd == 0x18) {
      curA = +SPEED_MOVE;
      curB = +SPEED_MOVE;
      lastArrowMs = now;
    }
    else if (cmd == 0x52) {
      curA = -SPEED_MOVE;
      curB = -SPEED_MOVE;
      lastArrowMs = now;
    }
    else if (cmd == 0x5A) {
      curA = -SPEED_SPIN;
      curB = +SPEED_SPIN;
      lastArrowMs = now;
    }
    else if (cmd == 0x08) {
      curA = +SPEED_SPIN;
      curB = -SPEED_SPIN;
      lastArrowMs = now;
    }
    else if (cmd == CMD_TOGGLE_PAGE) {
      currentPage = (currentPage == PAGE_SENSORS) ? PAGE_ACCEL : PAGE_SENSORS;
    }
    else if (cmd == 0x45) ledsAll(60, 0, 0);
    else if (cmd == 0x46) ledsAll(0, 60, 0);
    else if (cmd == 0x47) ledsAll(0, 0, 60);
    else if (cmd == 0x44) ledsAll(60, 0, 60);
    else if (cmd == 0x40) ledsAll(60, 60, 0);
    else if (cmd == 0x43) ledsAll(0, 60, 60);
    else if (cmd == 0x07) {
      uint32_t c[9] = {
        pixels.Color(60, 0, 0),
        pixels.Color(60, 20, 0),
        pixels.Color(60, 60, 0),
        pixels.Color(0, 60, 0),
        pixels.Color(0, 0, 60),
        pixels.Color(60, 0, 60),
        pixels.Color(60, 0, 0),
        pixels.Color(60, 20, 0),
        pixels.Color(60, 60, 0)
      };
      for (int i = 0; i < LED_COUNT; i++) pixels.setPixelColor(i, c[i]);
      pixels.show();
    }
    else if (cmd == 0x15) {
      ledsAll((uint8_t)random(0, 80), (uint8_t)random(0, 80), (uint8_t)random(0, 80));
    }
    else if (cmd == 0x09) {
      for (int i = 0; i < LED_COUNT; i++) {
        pixels.setPixelColor(i, pixels.Color((uint8_t)random(0, 80),
                                             (uint8_t)random(0, 80),
                                             (uint8_t)random(0, 80)));
      }
      pixels.show();
    }
    else if (cmd == CMD_LEDS_OFF) {
      ledsOff();
    }
    else if (cmd == 0x16) {
      shortBeep(350, 90);
    }
    else if (cmd == 0x0D) {
      shortBeep(1200, 90);
    }
  }

  if (now - lastArrowMs > HOLD_TIMEOUT_MS) {
    curA = 0;
    curB = 0;
  }

  applyMotors(curA, curB);

  if (now - tUS >= ULTRASONIC_MS) {
    tUS = now;
    readUltrasonic();
  }

  if (now - tAccel >= ACCEL_MS) {
    tAccel = now;
    readAccelerometer();
  }

  if (now - tDash >= DASH_UPDATE_MS) {
    tDash = now;
    readFastSensors();

    if (currentPage == PAGE_SENSORS) drawSensorDashboard();
    else drawAccelDashboard();
  }
}