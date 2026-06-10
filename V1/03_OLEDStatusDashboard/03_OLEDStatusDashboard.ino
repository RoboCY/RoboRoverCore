/*
  ============================================================
  Project: OLED Graph Dashboard (Live)
  Board: Kypruino UNO+ v0.8
  Platform: RoboRover Core
  ============================================================

  Description: This sketch outputs sensor readings on the
  OLED screen, as well as the Serial Monitor.

  Layout (128x32 OLED):
    [Box]     [sun]  Ultrasonic distance  [sun]     [Box]
   L obstacle                                      R obstacle
             L LDR bar                   R LDR bar
                          IR0 IR1 IR2 

  Boxes:
    - outline only = no obstacle
    - filled       = obstacle detected


  
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

#define PIN_IR0    A0
#define PIN_IR1    A1
#define PIN_IR2    A2

#define PIN_L_LDR  A6
#define PIN_R_LDR  A7

// -------------------- I2C EXPANDER (Obstacle IR) --------------------
#define EXP_ADDR   0x20
#define EXP_OIR_L  2  // Left Obstacle IR sensor
#define EXP_OIR_R  3  // Right Obstacle IR sensor

// -------------------- TIMING --------------------
#define UPDATE_MS         50  // How often to update on-board OLED screen
#define SERIAL_MS        200  // How often to updae Serial Monitor
#define ECHO_TIMEOUT_US 20000UL // 20000 Unsigned Long (to be used as [μs])

unsigned long tUpdate = 0;
unsigned long tSerial = 0;

// -------------------- SENSOR VALUES (deafault values) --------------------
float distanceCm = -1.0;
int ir0 = 0, ir1 = 0, ir2 = 0;
int ldrL = 0, ldrR = 0;
int oirL = 0, oirR = 0;


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

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Graph Dashboard");
  display.display();
  delay(600);
}

void loop() {
  unsigned long now = millis(); // Current time stamp [ms]

  if (now - tUpdate >= UPDATE_MS) { // If it is time to update
    tUpdate = now;  // Assigning current time as latest OLED Screen update time
    readAllSensors(); // Updates all variables with the latest sensor readings
    drawDashboard();
  }

  if (now - tSerial >= SERIAL_MS) {
    tSerial = now;  // Assigning current time as latest Serial update time
    serialDump();
  }
}


// -------------------- EXPANDER HELPERS --------------------
void expWrite(uint8_t v) {  // Sending to expander using I2C protocol
  Wire.beginTransmission(EXP_ADDR);
  Wire.write(v);
  Wire.endTransmission();
}

uint8_t expRead() { // Requesting readings from sensors on epxander using I2C protocol
  Wire.requestFrom(EXP_ADDR, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  } else {
    return 0xFF;
  }
}

// -------------------- SMALL DRAW HELPERS --------------------
int clampi(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Tiny "sun" icon (circle + 4 rays)
void drawSun(int cx, int cy) {
  display.drawCircle(cx, cy, 2, SSD1306_WHITE);
  display.drawLine(cx - 4, cy,     cx - 3, cy,     SSD1306_WHITE);
  display.drawLine(cx + 3, cy,     cx + 4, cy,     SSD1306_WHITE);
  display.drawLine(cx,     cy - 4, cx,     cy - 3, SSD1306_WHITE);
  display.drawLine(cx,     cy + 3, cx,     cy + 4, SSD1306_WHITE);
}

// Vertical bar inside outline, filled from bottom
void drawVBar(int x, int y, int w, int h, int value0_1023) {
  display.drawRect(x, y, w, h, SSD1306_WHITE);

  int fillH = (value0_1023 * (h - 2)) / 1023;
  fillH = clampi(fillH, 0, h - 2);

  int fillY = y + (h - 1) - fillH;
  if (fillH > 0) display.fillRect(x + 1, fillY, w - 2, fillH, SSD1306_WHITE);
}

// Center short text in a region
void drawCenteredText(const char* s, int xMin, int xMax, int y) {
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(s, 0, y, &x1, &y1, &w, &h);
  int x = xMin + ((xMax - xMin) - (int)w) / 2;
  display.setCursor(x, y);
  display.print(s);
}

// -------------------- SENSOR READ --------------------
void readAllSensors() { // Updates all variables with the latest sensor readings
  // Ultrasonic
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long us = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);
  distanceCm = (us == 0) ? -1.0 : (us / 58.0);

  // IR analog
  ir0 = analogRead(PIN_IR0);
  ir1 = analogRead(PIN_IR1);
  ir2 = analogRead(PIN_IR2);

  // LDR
  ldrL = analogRead(PIN_L_LDR);
  ldrR = analogRead(PIN_R_LDR);

  // Obstacle IR (expander)
  expWrite(0xFF);
  uint8_t v = expRead();  // Expander value
  oirL = (v >> EXP_OIR_L) & 1;  // Shifting read value to the bit of interest as the lowest bit (left obstacle IR sensor) and checking if TRUE(1, obstacle detected) or FALSE(0)
  oirR = (v >> EXP_OIR_R) & 1;  // Shifting read value to the bit of interest as the lowest bit (right obstacle IR sensor) and checking if TRUE(1, obstacle detected) or FALSE(0)
  // The "& (0b)1" operator returns 0b00000001 OR 0b00000000 (T/F)
}

// -------------------- SERIAL PRINT --------------------
void serialDump() {
  Serial.print("US_cm: ");
  if (distanceCm < 0) Serial.print("N/A");
  else Serial.print(distanceCm, 1);

  Serial.print(" | IR: ");
  Serial.print(ir0); Serial.print(" ");
  Serial.print(ir1); Serial.print(" ");
  Serial.print(ir2);

  Serial.print(" | LDR: ");
  Serial.print(ldrL); Serial.print(" ");
  Serial.print(ldrR);

  Serial.print(" | OIR: ");
  Serial.print(oirL); Serial.print(" ");
  Serial.println(oirR);
}

// -------------------- DRAW DASHBOARD --------------------
void drawDashboard() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  // ---- Geometry ----
  const int boxW = 10, boxH = 10;

  // Top row y positions
  const int yBox = 0;
  const int ySun = 0;      // suns on top line
  const int yDist = 1;     // ultrasonic text row
  const int yLRLabel = 12; // "L" and "R" below boxes

  // Bars region (go to bottom)
  const int barsTop = 14;
  const int barsBottom = 31;
  const int barsH = barsBottom - barsTop + 1;

  // IR bars (center group)
  const int irBarW = 8;
  const int irGap  = 4;
  const int irGroupW = (3 * irBarW) + (2 * irGap);
  const int irStartX = (SCREEN_WIDTH - irGroupW) / 2;

  // IR bar height goes down to just above labels
  const int irLabelY = 26;
  const int irBarsTop = barsTop;
  const int irBarsH = irLabelY - irBarsTop;

  // LDR bars same thickness as obstacle boxes
  const int ldrW = boxW;

  // ---- EVEN SPACING ACROSS TOP ----
  // Layout is: [box][gap][ldr][gap][IR group][gap][ldr][gap][box]
  int used = boxW + ldrW + irGroupW + ldrW + boxW; // fixed widths
  int gaps = 4;                                   // number of gaps between them
  int gapW = (SCREEN_WIDTH - used) / gaps;        // even spacing

  int xBoxL = 0;
  int xLdrL = xBoxL + boxW + gapW;
  int xIR   = xLdrL + ldrW + gapW;
  int xLdrR = xIR + irGroupW + gapW;
  int xBoxR = xLdrR + ldrW + gapW;

  // ---- Obstacle boxes ----
  display.drawRect(xBoxL, yBox, boxW, boxH, SSD1306_WHITE);
  if (oirL) display.fillRect(xBoxL + 1, yBox + 1, boxW - 2, boxH - 2, SSD1306_WHITE);

  display.drawRect(xBoxR, yBox, boxW, boxH, SSD1306_WHITE);
  if (oirR) display.fillRect(xBoxR + 1, yBox + 1, boxW - 2, boxH - 2, SSD1306_WHITE);

  // ---- L / R labels under boxes ----
  display.setCursor(xBoxL + 2, yLRLabel);
  display.print("L");
  display.setCursor(xBoxR + 2, yLRLabel);
  display.print("R");

  // ---- Suns on top line above LDR bars ----
  drawSun(xLdrL + ldrW / 2, ySun + 4);
  drawSun(xLdrR + ldrW / 2, ySun + 4);

  // ---- Ultrasonic in the middle (between the two boxes) ----
  char buf[12];
  if (distanceCm < 0) snprintf(buf, sizeof(buf), "N/A");
  else {
    int d = (int)(distanceCm + 0.5f);
    snprintf(buf, sizeof(buf), "%dcm", d);
  }
  drawCenteredText(buf, xBoxL + boxW + 1, xBoxR - 1, yDist);

  // ---- LDR bars (moved toward center by layout) ----
  drawVBar(xLdrL, barsTop, ldrW, barsH, ldrL);
  drawVBar(xLdrR, barsTop, ldrW, barsH, ldrR);

  // ---- IR bars ----
  drawVBar(xIR + 0 * (irBarW + irGap), irBarsTop, irBarW, irBarsH, ir0);
  drawVBar(xIR + 1 * (irBarW + irGap), irBarsTop, irBarW, irBarsH, ir1);
  drawVBar(xIR + 2 * (irBarW + irGap), irBarsTop, irBarW, irBarsH, ir2);

  // IR labels
  display.setCursor(xIR + 0 * (irBarW + irGap) + 2, irLabelY); display.print("0");
  display.setCursor(xIR + 1 * (irBarW + irGap) + 2, irLabelY); display.print("1");
  display.setCursor(xIR + 2 * (irBarW + irGap) + 2, irLabelY); display.print("2");

  display.display();
}