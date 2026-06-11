/*
  ============================================================
  Project: OLED Graph Dashboard (Live) - RoboRover Core 2
  Board: Kypruino UNO+ v0.8
  Platform: RoboRover Core 2
  ============================================================

  Description:
  Live sensor dashboard on the OLED and Serial Monitor.

  OLED layout:
    [Box]     [sun]  Ultrasonic distance  [sun]     [Box]
   L obstacle                                      R obstacle
             L LDR bar               R LDR bar
                    IR4 IR3 IR2 IR1 IR0

  Boxes:
    - outline only = no obstacle
    - filled       = obstacle detected

  Notes:
  - 5 line sensors are read through TLA2528 @ 0x14
  - obstacle IR digital outputs are read from PCF8574 @ 0x20
  - accelerometer (LIS2DH12 @ 0x19) is printed to Serial only
  - obstacle logic below is set for:
      raw bit 0 = obstacle present
      raw bit 1 = clear
  ============================================================
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// -------------------- OLED --------------------
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   32
#define OLED_RESET     -1
#define OLED_ADDR      0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// -------------------- PINS --------------------
#define PIN_TRIG   4
#define PIN_ECHO   5

#define PIN_L_LDR  A6
#define PIN_R_LDR  A7

// -------------------- I2C DEVICES --------------------
#define EXP_ADDR   0x20   // PCF8574
#define TLA_ADDR   0x14   // TLA2528
#define LIS_ADDR   0x19   // LIS2DH12

// -------------------- PCF8574 BITS --------------------
#define EXP_OIR_L  2
#define EXP_OIR_R  3

// -------------------- TLA2528 --------------------
#define TLA_OP_WRITE_REG     0x08
#define TLA_REG_CHANNEL_SEL  0x11

// -------------------- TIMING --------------------
#define UPDATE_MS         50
#define SERIAL_MS        200
#define ECHO_TIMEOUT_US 20000UL

unsigned long tUpdate = 0;
unsigned long tSerial = 0;

// -------------------- SENSOR VALUES --------------------
float distanceCm = -1.0;
int ir[5] = {0, 0, 0, 0, 0};
int ldrL = 0, ldrR = 0;
int oirL = 0, oirR = 0;
int16_t accX = 0, accY = 0, accZ = 0;

// ============================================================
//                         SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    while (1);
  }

  initAccelerometer();

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Graph Dashboard");
  display.display();
  delay(600);
}

// ============================================================
//                          LOOP
// ============================================================
void loop() {
  unsigned long now = millis();

  if (now - tUpdate >= UPDATE_MS) {
    tUpdate = now;
    readAllSensors();
    drawDashboard();
  }

  if (now - tSerial >= SERIAL_MS) {
    tSerial = now;
    serialDump();
  }
}

// ============================================================
//                    EXPANDER HELPERS
// ============================================================
void expWrite(uint8_t v) {
  Wire.beginTransmission(EXP_ADDR);
  Wire.write(v);
  Wire.endTransmission();
}

uint8_t expRead() {
  Wire.requestFrom(EXP_ADDR, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0xFF;
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
  Wire.write(0x28 | 0x80); // auto-increment from OUT_X_L
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
//                    SMALL DRAW HELPERS
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
  if (fillH > 0) {
    display.fillRect(x + 1, fillY, w - 2, fillH, SSD1306_WHITE);
  }
}

void drawCenteredText(const char* s, int xMin, int xMax, int y) {
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(s, 0, y, &x1, &y1, &w, &h);
  int x = xMin + ((xMax - xMin) - (int)w) / 2;
  display.setCursor(x, y);
  display.print(s);
}

// ============================================================
//                      SENSOR READ
// ============================================================
void readAllSensors() {
  // Ultrasonic
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long us = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);
  distanceCm = (us == 0) ? -1.0 : (us / 58.0);

  // 5 line sensors from TLA2528
  for (int i = 0; i < 5; i++) {
    uint16_t raw12 = tlaReadChannel(i);
    ir[i] = tlaTo1023(raw12);
  }

  // LDRs
  ldrL = analogRead(PIN_L_LDR);
  ldrR = analogRead(PIN_R_LDR);

  // Obstacle IR from expander
  expWrite(0xFF);   // keep inputs high
  uint8_t v = expRead();

  // Reversed logic:
  // raw bit 0 = obstacle present
  // raw bit 1 = clear
  oirL = (((v >> EXP_OIR_L) & 1) == 0);
  oirR = (((v >> EXP_OIR_R) & 1) == 0);

  // Accelerometer
  readAccelerometer();
}

// ============================================================
//                     SERIAL PRINT
// ============================================================
void serialDump() {
  Serial.print("US_cm: ");
  if (distanceCm < 0) Serial.print("N/A");
  else Serial.print(distanceCm, 1);

  Serial.print(" | IR: ");
  for (int i = 0; i < 5; i++) {
    Serial.print(ir[i]);
    Serial.print(" ");
  }

  Serial.print("| LDR: ");
  Serial.print(ldrL);
  Serial.print(" ");
  Serial.print(ldrR);

  Serial.print(" | OIR: ");
  Serial.print(oirL);
  Serial.print(" ");
  Serial.print(oirR);

  Serial.print(" | ACC: ");
  Serial.print(accX);
  Serial.print(" ");
  Serial.print(accY);
  Serial.print(" ");
  Serial.println(accZ);
}

// ============================================================
//                     DRAW DASHBOARD
// ============================================================
void drawDashboard() {
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

  // Obstacle boxes
  display.drawRect(xBoxL, yBox, boxW, boxH, SSD1306_WHITE);
  if (oirL) display.fillRect(xBoxL + 1, yBox + 1, boxW - 2, boxH - 2, SSD1306_WHITE);

  display.drawRect(xBoxR, yBox, boxW, boxH, SSD1306_WHITE);
  if (oirR) display.fillRect(xBoxR + 1, yBox + 1, boxW - 2, boxH - 2, SSD1306_WHITE);

  // L/R labels
  display.setCursor(xBoxL + 2, yLRLabel);
  display.print("L");
  display.setCursor(xBoxR + 2, yLRLabel);
  display.print("R");

  // Suns
  drawSun(xLdrL + ldrW / 2, ySun + 4);
  drawSun(xLdrR + ldrW / 2, ySun + 4);

  // Ultrasonic text
  char buf[12];
  if (distanceCm < 0) {
    snprintf(buf, sizeof(buf), "N/A");
  } else {
    int d = (int)(distanceCm + 0.5f);
    snprintf(buf, sizeof(buf), "%dcm", d);
  }
  drawCenteredText(buf, xBoxL + boxW + 1, xBoxR - 1, yDist);

  // LDR bars
  drawVBar(xLdrL, barsTop, ldrW, barsH, ldrL);
  drawVBar(xLdrR, barsTop, ldrW, barsH, ldrR);

  // IR bars reversed on screen:
  // leftmost = IR4, rightmost = IR0
  for (int screenPos = 0; screenPos < 5; screenPos++) {
    int sensorIndex = 4 - screenPos;
    int x = xIR + screenPos * (irBarW + irGap);

    drawVBar(x, barsTop, irBarW, irBarsH, ir[sensorIndex]);
    display.setCursor(x + 1, irLabelY);
    display.print(sensorIndex);
  }

  display.display();
}