/*
  ============================================================
  Project: Bubble Level + Position-Reactive LEDs
  Board: Kypruino UNO+ v0.8
  Platform: RoboRover Core 2
  ============================================================

  Description:
  A bubble moves inside a box on the OLED according to tilt.

  LED behavior:
  - Up/down controls LED hue
  - Left/right controls where brightness is concentrated:
      far left  -> left LEDs brightest
      center    -> center LEDs brightest
      far right -> right LEDs brightest

  Hardware:
  - OLED: I2C 0x3C
  - Accelerometer: LIS2DH12 @ 0x19
  - NeoPixels: D8
  ============================================================
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

// -------------------- OLED --------------------
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   32
#define OLED_RESET     -1
#define OLED_ADDR      0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// -------------------- ACCEL --------------------
#define LIS_ADDR 0x19
int16_t accX = 0, accY = 0, accZ = 0;

// -------------------- NEOPIXELS --------------------
#define PIN_NEOPIXEL 8
#define LED_COUNT    6
Adafruit_NeoPixel pixels(LED_COUNT, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// -------------------- BUBBLE --------------------
float bubbleX = SCREEN_WIDTH / 2.0f;
float bubbleY = SCREEN_HEIGHT / 2.0f;
float velX = 0.0f;
float velY = 0.0f;

const float bubbleR = 3.0f;

// Physics
const float accelScale = 0.00045f;
const float damping    = 0.88f;
const float maxVel     = 1.8f;

// Play area box
const int boxX = 8;
const int boxY = 2;
const int boxW = 112;
const int boxH = 28;

// ============================================================
//                    ACCEL HELPERS
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
//                    UTILS
// ============================================================
float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

uint32_t wheelColor(uint8_t pos) {
  pos = 255 - pos;
  if (pos < 85) {
    return pixels.Color(255 - pos * 3, 0, pos * 3);
  }
  if (pos < 170) {
    pos -= 85;
    return pixels.Color(0, pos * 3, 255 - pos * 3);
  }
  pos -= 170;
  return pixels.Color(pos * 3, 255 - pos * 3, 0);
}

void scaleAndSetPixel(int i, uint32_t baseColor, float brightness01) {
  brightness01 = clampf(brightness01, 0.0f, 1.0f);

  uint8_t r = (baseColor >> 16) & 0xFF;
  uint8_t g = (baseColor >> 8) & 0xFF;
  uint8_t b = baseColor & 0xFF;

  r = (uint8_t)(r * brightness01);
  g = (uint8_t)(g * brightness01);
  b = (uint8_t)(b * brightness01);

  pixels.setPixelColor(i, pixels.Color(r, g, b));
}

// ============================================================
//                    LED HELPERS
// ============================================================
void updateLedsFromBubble() {
  float centerX = boxX + boxW / 2.0f;
  float centerY = boxY + boxH / 2.0f;

  float xSpan = (boxW / 2.0f) - bubbleR - 1.0f;
  float ySpan = (boxH / 2.0f) - bubbleR - 1.0f;

  float normX = (bubbleX - centerX) / xSpan;   // -1 left, +1 right
  float normY = (bubbleY - centerY) / ySpan;   // -1 up, +1 down

  normX = clampf(normX, -1.0f, 1.0f);
  normY = clampf(normY, -1.0f, 1.0f);

  // Hue from up/down
  float hue01 = (normY + 1.0f) * 0.5f;
  uint8_t hue = (uint8_t)(hue01 * 255.0f);
  uint32_t baseColor = wheelColor(hue);

  // Physical LED positions, based on your real layout
  float ledPos[6] = { 0.4f, -0.4f, -1.0f, -0.4f, 0.4f, 1.0f };

  const float sigma = 0.45f;

  for (int i = 0; i < LED_COUNT; i++) {
    float dist = ledPos[i] - normX;
    float brightness = exp(-(dist * dist) / (2.0f * sigma * sigma));

    float minBrightness = 0.06f;
    brightness = minBrightness + (1.0f - minBrightness) * brightness;

    scaleAndSetPixel(i, baseColor, brightness);
  }

  pixels.show();
}

// ============================================================
//                    BUBBLE HELPERS
// ============================================================
void resetBubble() {
  bubbleX = boxX + boxW / 2.0f;
  bubbleY = boxY + boxH / 2.0f;
  velX = 0.0f;
  velY = 0.0f;
}

void updateBubble() {
  readAccelerometer();

  // left/right already felt correct
  // forward/back corrected
  float ax =  accY * accelScale;
  float ay =  accX * accelScale;

  velX += ax;
  velY += ay;

  velX *= damping;
  velY *= damping;

  velX = clampf(velX, -maxVel, maxVel);
  velY = clampf(velY, -maxVel, maxVel);

  bubbleX += velX;
  bubbleY += velY;

  float minX = boxX + bubbleR + 1;
  float maxX = boxX + boxW - bubbleR - 2;
  float minY = boxY + bubbleR + 1;
  float maxY = boxY + boxH - bubbleR - 2;

  if (bubbleX < minX) {
    bubbleX = minX;
    velX = -velX * 0.45f;
  }
  if (bubbleX > maxX) {
    bubbleX = maxX;
    velX = -velX * 0.45f;
  }
  if (bubbleY < minY) {
    bubbleY = minY;
    velY = -velY * 0.45f;
  }
  if (bubbleY > maxY) {
    bubbleY = maxY;
    velY = -velY * 0.45f;
  }
}

void drawBubbleScreen() {
  display.clearDisplay();

  display.drawRect(boxX, boxY, boxW, boxH, SSD1306_WHITE);

  int cx = boxX + boxW / 2;
  int cy = boxY + boxH / 2;
  display.drawLine(cx - 5, cy, cx + 5, cy, SSD1306_WHITE);
  display.drawLine(cx, cy - 5, cx, cy + 5, SSD1306_WHITE);

  display.fillCircle((int)bubbleX, (int)bubbleY, (int)bubbleR, SSD1306_WHITE);

  display.display();
}

// ============================================================
//                        SETUP / LOOP
// ============================================================
void setup() {
  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    while (1);
  }

  initAccelerometer();

  pixels.begin();
  pixels.setBrightness(255);
  pixels.clear();
  pixels.show();

  resetBubble();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(18, 10);
  display.print("Bubble + LEDs");
  display.display();
  delay(800);
}

void loop() {
  updateBubble();
  updateLedsFromBubble();
  drawBubbleScreen();
  delay(20);
}