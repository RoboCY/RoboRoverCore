/*
  RoboRover Kypruino (ATmega328P) — WiFi Command + Telemetry
  - Receives line-based commands from ESP8266 over SoftwareSerial (D12/D13)
  - Drives motors via PWM (D11/D10) + PCF8574 (P0/P1) direction
  - Reads sensors: ultrasonic, line array, obstacle flags, LDRs
  - Implements actions: Light modes, Light-follow mode, beep, piano demo, generic placeholders

  Hardware reference:
  - PCF8574 @ 0x20
  - Motor direction mapping (working robot code):
    Forward:  P0=0, P1=0
    Backward: P0=1, P1=1
    Spin Right: P0=1, P1=0
    Spin Left:  P0=0, P1=1
*/

#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------------- Pins / addresses ----------------
static const uint8_t WIFI_RX_PIN = 12; // Kypruino RX  <- ESP TX
static const uint8_t WIFI_TX_PIN = 13; // Kypruino TX  -> ESP RX (via divider)
static const uint32_t WIFI_BAUD  = 57600;

static const uint8_t PWM_LEFT  = 11;
static const uint8_t PWM_RIGHT = 10;

static const uint8_t BUZZER_PIN = 9;
static const uint8_t NEO_PIN    = 8;  // Kypruino NeoPixels / RGB LEDs pin
static const uint8_t NEO_COUNT  = 6;  // adjust if needed

static const uint8_t US_TRIG = 4;
static const uint8_t US_ECHO = 5;

static const uint8_t LINE_L = A0;
static const uint8_t LINE_C = A1;
static const uint8_t LINE_R = A2;

static const uint8_t LDR_L = A6;
static const uint8_t LDR_R = A7;

static const uint8_t PCF_ADDR = 0x20;

// OLED configuration (128x32)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define OLED_ADDR 0x3C

// PCF bits
static const uint8_t PCF_P0 = 0; // left dir
static const uint8_t PCF_P1 = 1; // right dir
static const uint8_t PCF_P2 = 2; // obstacle left (input)
static const uint8_t PCF_P3 = 3; // obstacle right (input)

// Inputs + reserved should be released HIGH on PCF8574 (quasi-bidirectional)
static const uint8_t PCF_RELEASE_MASK = 0b11111100; // P2..P7 high

// ---------------- Comms ----------------
SoftwareSerial wifiSerial(WIFI_RX_PIN, WIFI_TX_PIN); // (RX, TX)

// ---------------- LEDs ----------------
Adafruit_NeoPixel pixels(NEO_COUNT, NEO_PIN, NEO_GRB + NEO_KHZ800);

enum class LedMode : uint8_t { Off=0, Solid=1, Police=2, Rainbow=3 };
static LedMode ledMode = LedMode::Off;
static uint32_t ledLastMs = 0;
static uint16_t rainbowHue = 0;
static bool policeFlip = false;

// ---------------- Robot modes ----------------
enum class RunMode : uint8_t { Teleop=0, LightFollow=1, Dance=2 };
static RunMode runMode = RunMode::Teleop;

// Dance sequence state
static uint8_t danceStep = 0;
static uint32_t danceStepStartMs = 0;

// ---------------- Motor config ----------------
static bool leftInvert  = false;
static bool rightInvert = false;
static bool obstacleInvert = false;

static uint8_t deadbandPWM = 10;         // below this -> 0
static uint16_t deadmanMs  = 1000;       // teleop safety stop if no M updates (increased for WiFi jitter)
static uint16_t autoCommsTimeoutMs = 2000; // in auto, stop if no comms for a while

static bool debugEnabled = false;        // enable verbose debug output via SET debug 1

static int lastCmdL = 0;
static int lastCmdR = 0;

static uint32_t lastMotorCmdMs = 0; // last time we received M
static uint32_t lastAnyCmdMs   = 0; // last time we received anything

// ---------------- PCF state ----------------
static uint8_t pcfOut = (PCF_RELEASE_MASK | 0b00000000); // default forward, inputs released

static void pcfWrite(uint8_t val) {
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(val);
  Wire.endTransmission();
}

static uint8_t pcfRead() {
  Wire.requestFrom((int)PCF_ADDR, 1);
  if (Wire.available()) return (uint8_t)Wire.read();
  return 0xFF;
}

static void pcfSetDirBits(bool leftBackward, bool rightBackward) {
  // forward mapping expects 0, backward expects 1 (per your working robot code)
  uint8_t v = PCF_RELEASE_MASK;
  if (leftBackward)  v |= (1 << PCF_P0);
  if (rightBackward) v |= (1 << PCF_P1);
  pcfOut = v;
  pcfWrite(pcfOut);
}

static void motorsStop() {
  analogWrite(PWM_LEFT, 0);
  analogWrite(PWM_RIGHT, 0);
  // keep direction bits in a safe “forward” state
  pcfSetDirBits(false, false);
  lastCmdL = 0; lastCmdR = 0;
}

static int clampi(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void applyMotors(int left255, int right255) {
  left255  = clampi(left255,  -255, 255);
  right255 = clampi(right255, -255, 255);

  if (leftInvert)  left255  = -left255;
  if (rightInvert) right255 = -right255;

  const bool leftBackward  = (left255 < 0);
  const bool rightBackward = (right255 < 0);

  int l = abs(left255);
  int r = abs(right255);

  if (l < deadbandPWM) l = 0;
  if (r < deadbandPWM) r = 0;

  pcfSetDirBits(leftBackward, rightBackward);

  // PWM values are latched by hardware and persist until changed
  // This ensures smooth motor operation between joystick packets (~30Hz)
  analogWrite(PWM_LEFT,  l);
  analogWrite(PWM_RIGHT, r);

  debugLog(String("PWM L=") + l + (leftBackward?"B":"F") + " R=" + r + (rightBackward?"B":"F"));

  lastCmdL = left255;
  lastCmdR = right255;
}

// ---------------- Ultrasonic ----------------
static int ultrasonicCM(uint32_t timeoutUs = 8000) { // Reduced from 25000 to prevent blocking
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);

  unsigned long dur = pulseIn(US_ECHO, HIGH, timeoutUs);
  if (dur == 0) return -1;
  // ~58us per cm (HC-SR04 class)
  return (int)(dur / 58UL);
}

// ---------------- Buzzer / melody (non-blocking) ----------------
struct MelodyPlayer {
  const uint16_t* freqs = nullptr;
  const uint16_t* durs  = nullptr;
  uint8_t len = 0;
  uint8_t idx = 0;
  bool active = false;
  uint32_t stepUntilMs = 0;

  void start(const uint16_t* f, const uint16_t* d, uint8_t n) {
    freqs = f; durs = d; len = n; idx = 0; active = true;
    stepUntilMs = 0;
  }

  void stop() {
    noTone(BUZZER_PIN);
    active = false;
  }

  void tick(uint32_t nowMs) {
    if (!active) return;
    if (nowMs < stepUntilMs) return;

    if (idx >= len) {
      stop();
      return;
    }

    uint16_t f = freqs[idx];
    uint16_t d = durs[idx];

    if (f == 0) noTone(BUZZER_PIN);
    else tone(BUZZER_PIN, f);

    stepUntilMs = nowMs + d;
    idx++;
  }
};

static MelodyPlayer melody;

static void beepShort() {
  static const uint16_t f[] = { 1200, 0, 1200 };
  static const uint16_t d[] = { 120, 60, 120 };
  melody.start(f, d, 3);
}

// “Piano demo” melody (very short)
static void pianoDemo() {
  static const uint16_t f[] = { 523, 659, 784, 1046, 0, 784, 659, 523 };
  static const uint16_t d[] = { 140, 140, 140, 220, 80, 140, 140, 260 };
  melody.start(f, d, 8);
}

// ---------------- LED patterns ----------------
static uint32_t wheel(uint8_t pos) {
  pos = 255 - pos;
  if (pos < 85)  return pixels.Color(255 - pos * 3, 0, pos * 3);
  if (pos < 170) { pos -= 85; return pixels.Color(0, pos * 3, 255 - pos * 3); }
  pos -= 170;
  return pixels.Color(pos * 3, 255 - pos * 3, 0);
}

static void setAll(uint32_t c) {
  for (uint8_t i=0; i<NEO_COUNT; i++) pixels.setPixelColor(i, c);
  pixels.show();
}

static void ledCycleMode() {
  uint8_t m = (uint8_t)ledMode;
  m = (m + 1) % 4;
  ledMode = (LedMode)m;
  ledLastMs = 0;
  rainbowHue = 0;
  policeFlip = false;
}

static void ledTick(uint32_t nowMs) {
  const uint16_t periodMs = 120;

  switch (ledMode) {
    case LedMode::Off:
      // Only refresh occasionally to avoid flicker
      if (ledLastMs == 0 || (nowMs - ledLastMs) > 500) {
        ledLastMs = nowMs;
        setAll(0);
      }
      break;

    case LedMode::Solid:
      if (ledLastMs == 0) {
        ledLastMs = nowMs;
        setAll(pixels.Color(0, 40, 120));
      }
      break;

    case LedMode::Police:
      if (ledLastMs == 0 || (nowMs - ledLastMs) > periodMs) {
        ledLastMs = nowMs;
        policeFlip = !policeFlip;
        uint32_t a = policeFlip ? pixels.Color(160, 0, 0) : pixels.Color(0, 0, 160);
        uint32_t b = policeFlip ? pixels.Color(0, 0, 160) : pixels.Color(160, 0, 0);
        if (NEO_COUNT >= 1) pixels.setPixelColor(0, a);
        if (NEO_COUNT >= 2) pixels.setPixelColor(1, b);
        if (NEO_COUNT >= 3) pixels.setPixelColor(2, a);
        pixels.show();
      }
      break;

    case LedMode::Rainbow:
      if (ledLastMs == 0 || (nowMs - ledLastMs) > 40) {
        ledLastMs = nowMs;
        rainbowHue++;
        for (uint8_t i=0; i<NEO_COUNT; i++) {
          pixels.setPixelColor(i, wheel((uint8_t)(rainbowHue + i*40)));
        }
        pixels.show();
      }
      break;
  }
}

// ---------------- Light-follow behavior ----------------
static void lightFollowTick() {
  // Simple proportional steering based on LDR difference
  // LDR readings: higher ADC usually means darker or brighter depending on divider orientation.
  // We’ll treat diff sign only; if reversed in practice, flip by swapping.
  int l = analogRead(LDR_L);
  int r = analogRead(LDR_R);

  int diff = (l - r); // positive => bias left side

  // Convert to turn term
  // Scale diff (0..1023) down to a useful turn range
  int turn = clampi(diff / 6, -120, 120);

  int base = 150; // default forward speed in auto
  int left  = base - turn;
  int right = base + turn;

  left  = clampi(left,  -200, 200);
  right = clampi(right, -200, 200);

  applyMotors(left, right);
}

// ---------------- Robot dance behavior ----------------
static void robotDanceTick(uint32_t nowMs) {
  // Fast, playful spinning dance - alternating 360° spins with upbeat music
  const uint16_t stepDuration = 300; // ms per step - quick and energetic!
  
  if (nowMs - danceStepStartMs < stepDuration) return;
  
  danceStepStartMs = nowMs;
  danceStep++;
  if (danceStep >= 20) danceStep = 0;
  
  // Alternating spin dance with rainbow chase lights and happy tones
  switch (danceStep) {
    case 0: // Fast spin right - start!
      applyMotors(255, -255);
      pixels.setPixelColor(0, pixels.Color(255, 0, 0));     // Red
      pixels.setPixelColor(1, pixels.Color(0, 0, 0));
      pixels.setPixelColor(2, pixels.Color(0, 0, 0));
      pixels.show();
      tone(BUZZER_PIN, 1047); // C6
      break;
      
    case 1: // Keep spinning right
      applyMotors(255, -255);
      pixels.setPixelColor(0, pixels.Color(0, 0, 0));
      pixels.setPixelColor(1, pixels.Color(255, 100, 0));   // Orange
      pixels.setPixelColor(2, pixels.Color(0, 0, 0));
      pixels.show();
      tone(BUZZER_PIN, 1175); // D6
      break;
      
    case 2: // Spin right continues
      applyMotors(255, -255);
      pixels.setPixelColor(0, pixels.Color(0, 0, 0));
      pixels.setPixelColor(1, pixels.Color(0, 0, 0));
      pixels.setPixelColor(2, pixels.Color(255, 255, 0));   // Yellow
      pixels.show();
      tone(BUZZER_PIN, 1319); // E6
      break;
      
    case 3: // Final spin right
      applyMotors(255, -255);
      pixels.setPixelColor(0, pixels.Color(0, 255, 0));     // Green
      pixels.setPixelColor(1, pixels.Color(0, 0, 0));
      pixels.setPixelColor(2, pixels.Color(0, 0, 0));
      pixels.show();
      tone(BUZZER_PIN, 1397); // F6
      break;
      
    case 4: // Quick pause - flash all
      applyMotors(0, 0);
      pixels.setPixelColor(0, pixels.Color(255, 255, 255)); // White flash
      pixels.setPixelColor(1, pixels.Color(255, 255, 255));
      pixels.setPixelColor(2, pixels.Color(255, 255, 255));
      pixels.show();
      tone(BUZZER_PIN, 1568); // G6
      break;
      
    case 5: // Fast spin left - go!
      applyMotors(-255, 255);
      pixels.setPixelColor(0, pixels.Color(0, 0, 0));
      pixels.setPixelColor(1, pixels.Color(0, 0, 0));
      pixels.setPixelColor(2, pixels.Color(0, 255, 255));   // Cyan
      pixels.show();
      tone(BUZZER_PIN, 1397); // F6
      break;
      
    case 6: // Keep spinning left
      applyMotors(-255, 255);
      pixels.setPixelColor(0, pixels.Color(0, 0, 0));
      pixels.setPixelColor(1, pixels.Color(0, 0, 255));     // Blue
      pixels.setPixelColor(2, pixels.Color(0, 0, 0));
      pixels.show();
      tone(BUZZER_PIN, 1319); // E6
      break;
      
    case 7: // Spin left continues
      applyMotors(-255, 255);
      pixels.setPixelColor(0, pixels.Color(255, 0, 255));   // Magenta
      pixels.setPixelColor(1, pixels.Color(0, 0, 0));
      pixels.setPixelColor(2, pixels.Color(0, 0, 0));
      pixels.show();
      tone(BUZZER_PIN, 1175); // D6
      break;
      
    case 8: // Final spin left
      applyMotors(-255, 255);
      pixels.setPixelColor(0, pixels.Color(0, 0, 0));
      pixels.setPixelColor(1, pixels.Color(0, 0, 0));
      pixels.setPixelColor(2, pixels.Color(255, 0, 150));   // Pink
      pixels.show();
      tone(BUZZER_PIN, 1047); // C6
      break;
      
    case 9: // Quick pause - flash
      applyMotors(0, 0);
      pixels.setPixelColor(0, pixels.Color(255, 200, 100)); // Warm white
      pixels.setPixelColor(1, pixels.Color(255, 200, 100));
      pixels.setPixelColor(2, pixels.Color(255, 200, 100));
      pixels.show();
      tone(BUZZER_PIN, 1568); // G6
      break;
      
    case 10: // Super fast spin right!
      applyMotors(255, -255);
      pixels.setPixelColor(0, pixels.Color(255, 50, 0));    // Red-orange
      pixels.setPixelColor(1, pixels.Color(255, 50, 0));
      pixels.setPixelColor(2, pixels.Color(0, 0, 0));
      pixels.show();
      tone(BUZZER_PIN, 1760); // A6 - high energy!
      break;
      
    case 11: // Continue fast right
      applyMotors(255, -255);
      pixels.setPixelColor(0, pixels.Color(0, 0, 0));
      pixels.setPixelColor(1, pixels.Color(255, 255, 0));   // Yellow
      pixels.setPixelColor(2, pixels.Color(255, 255, 0));
      pixels.show();
      tone(BUZZER_PIN, 1976); // B6
      break;
      
    case 12: // Still spinning right
      applyMotors(255, -255);
      pixels.setPixelColor(0, pixels.Color(0, 255, 100));
      pixels.setPixelColor(1, pixels.Color(0, 0, 0));
      pixels.setPixelColor(2, pixels.Color(0, 255, 100));
      pixels.show();
      tone(BUZZER_PIN, 2093); // C7 - super high!
      break;
      
    case 13: // Quick stop
      applyMotors(0, 0);
      pixels.setPixelColor(0, pixels.Color(255, 255, 255));
      pixels.setPixelColor(1, pixels.Color(255, 255, 255));
      pixels.setPixelColor(2, pixels.Color(255, 255, 255));
      pixels.show();
      tone(BUZZER_PIN, 1976); // B6
      break;
      
    case 14: // Super fast spin left!
      applyMotors(-255, 255);
      pixels.setPixelColor(0, pixels.Color(0, 200, 255));
      pixels.setPixelColor(1, pixels.Color(0, 200, 255));
      pixels.setPixelColor(2, pixels.Color(0, 0, 0));
      pixels.show();
      tone(BUZZER_PIN, 1760); // A6
      break;
      
    case 15: // Continue fast left
      applyMotors(-255, 255);
      pixels.setPixelColor(0, pixels.Color(0, 0, 0));
      pixels.setPixelColor(1, pixels.Color(200, 0, 255));
      pixels.setPixelColor(2, pixels.Color(200, 0, 255));
      pixels.show();
      tone(BUZZER_PIN, 1976); // B6
      break;
      
    case 16: // Still spinning left
      applyMotors(-255, 255);
      pixels.setPixelColor(0, pixels.Color(255, 0, 200));
      pixels.setPixelColor(1, pixels.Color(0, 0, 0));
      pixels.setPixelColor(2, pixels.Color(255, 0, 200));
      pixels.show();
      tone(BUZZER_PIN, 2093); // C7
      break;
      
    case 17: // Slow down - smaller spin right
      applyMotors(180, -180);
      pixels.setPixelColor(0, pixels.Color(255, 150, 0));
      pixels.setPixelColor(1, pixels.Color(255, 150, 0));
      pixels.setPixelColor(2, pixels.Color(255, 150, 0));
      pixels.show();
      tone(BUZZER_PIN, 1568); // G6
      break;
      
    case 18: // Slow spin left
      applyMotors(-180, 180);
      pixels.setPixelColor(0, pixels.Color(100, 255, 200));
      pixels.setPixelColor(1, pixels.Color(100, 255, 200));
      pixels.setPixelColor(2, pixels.Color(100, 255, 200));
      pixels.show();
      tone(BUZZER_PIN, 1319); // E6
      break;
      
    case 19: // Grand finale - all lights!
      applyMotors(0, 0);
      pixels.setPixelColor(0, pixels.Color(255, 100, 255)); // Purple
      pixels.setPixelColor(1, pixels.Color(255, 200, 0));   // Gold
      pixels.setPixelColor(2, pixels.Color(0, 255, 255));   // Cyan
      pixels.show();
      tone(BUZZER_PIN, 2093); // C7 - triumphant!
      break;
  }
}

// ---------------- Command parsing ----------------
static char rxBuf[128];
static uint8_t rxLen = 0;

static void sendLine(const String& s) {
  wifiSerial.print(s);
  wifiSerial.print('\n');
}

static void debugLog(const String& s) {
  if (debugEnabled) {
    Serial.print("DBG ");
    Serial.print(s);
    Serial.print('\n');
  }
}

static void handleCall(const String& id) {
  if (id == "LIGHTMODE") {
    ledCycleMode();
    return;
  }
  if (id == "DANCE") {
    if (runMode == RunMode::Teleop) {
      runMode = RunMode::Dance;
      danceStep = 0;
      danceStepStartMs = millis();
    } else {
      runMode = RunMode::Teleop;
      motorsStop();
      noTone(BUZZER_PIN); // Turn off buzzer when exiting dance
    }
    return;
  }
  if (id == "BEEP") {
    beepShort();
    return;
  }
  if (id == "PIANO") {
    pianoDemo();
    return;
  }
  if (id == "G1") {
    sendLine("LOG custom A (placeholder)");
    return;
  }
  if (id == "G2") {
    sendLine("LOG custom B (placeholder)");
    return;
  }

  sendLine(String("ERR unknown CALL: ") + id);
}

static void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  lastAnyCmdMs = millis();

  // Tokenize by spaces (simple)
  if (line == "STOP") {
    debugLog("RX STOP command");
    runMode = RunMode::Teleop;
    motorsStop();
    return;
  }

  if (line.startsWith("M ")) {
    // M <L> <R>
    int sp1 = line.indexOf(' ');
    int sp2 = line.indexOf(' ', sp1 + 1);
    if (sp2 < 0) { sendLine("ERR M needs 2 ints"); return; }

    int l = line.substring(sp1 + 1, sp2).toInt();
    int r = line.substring(sp2 + 1).toInt();

    debugLog(String("RX M L=") + l + " R=" + r);

    // Any non-zero joystick cancels auto modes (lightfollow or dance)
    if ((runMode == RunMode::LightFollow || runMode == RunMode::Dance) && (l != 0 || r != 0)) {
      if (runMode == RunMode::Dance) {
        noTone(BUZZER_PIN); // Turn off buzzer when dance is canceled
      }
      runMode = RunMode::Teleop;
    }

    // Update timestamp BEFORE applying motors so dead-man check sees fresh value
    lastMotorCmdMs = millis();
    applyMotors(l, r);
    return;
  }

  if (line.startsWith("CALL ")) {
    String id = line.substring(5);
    id.trim();
    handleCall(id);
    return;
  }

  if (line.startsWith("TXT ")) {
    // Echo back (and keep it visible in console)
    sendLine(String("LOG txt: ") + line.substring(4));
    return;
  }

  // Advanced users: allow raw "SET key value"
  if (line.startsWith("SET ")) {
    // SET deadmanMs 400, SET deadbandPWM 10, SET leftInvert 0/1, etc.
    int sp1 = line.indexOf(' ');
    int sp2 = line.indexOf(' ', sp1 + 1);
    if (sp2 < 0) { sendLine("ERR SET needs key value"); return; }
    String key = line.substring(sp1 + 1, sp2);
    String val = line.substring(sp2 + 1);
    key.trim(); val.trim();

    if (key == "deadmanMs") deadmanMs = (uint16_t)clampi(val.toInt(), 100, 2000);
    else if (key == "deadbandPWM") deadbandPWM = (uint8_t)clampi(val.toInt(), 0, 80);
    else if (key == "leftInvert") leftInvert = (val.toInt() != 0);
    else if (key == "rightInvert") rightInvert = (val.toInt() != 0);
    else if (key == "obstacleInvert") obstacleInvert = (val.toInt() != 0);
    else if (key == "debug") {
      debugEnabled = (val.toInt() != 0);
      Serial.println(debugEnabled ? "*** DEBUG MODE ENABLED ***" : "*** DEBUG MODE DISABLED ***");
    }
    else { sendLine(String("ERR unknown key: ") + key); return; }

    sendLine(String("LOG set ") + key + "=" + val);
    return;
  }

  sendLine(String("ERR unknown cmd: ") + line);
}

// ---------------- Telemetry ----------------
static uint32_t lastTelemMs = 0;

static void telemetryTick(uint32_t nowMs) {
  if (nowMs - lastTelemMs < 1000) return; // Reduced to 1 Hz to prevent WiFi overload
  lastTelemMs = nowMs;

  // Minimal telemetry - only mode (0=teleop, 1=lightfollow, 2=dance)
  String s = "T ";
  s += (runMode == RunMode::Teleop ? 0 : (runMode == RunMode::LightFollow ? 1 : 2));
  sendLine(s);
}

// ---------------- Setup / loop ----------------
void setup() {
  // Hardware serial for debugging at 115200 baud (USB connection)
  Serial.begin(115200);
  delay(200); // Longer delay for Serial stability
  Serial.println("=== Kypruino Debug Monitor ===");
  Serial.println("Debug mode: OFF (use web console: SET debug 1 to enable)");
  Serial.println("Baud: 115200");
  Serial.println("==============================");
  
  pinMode(PWM_LEFT, OUTPUT);
  pinMode(PWM_RIGHT, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);

  Wire.begin();
  
  // Initialize PCF with inputs released, direction forward
  pcfOut = (PCF_RELEASE_MASK | 0b00000000);
  pcfWrite(pcfOut);

  pixels.begin();
  pixels.setBrightness(70);
  setAll(0);

  wifiSerial.begin(WIFI_BAUD);

  motorsStop();
  lastMotorCmdMs = millis();
  lastAnyCmdMs = millis();

  Serial.println("Kypruino initialized and ready.");
}

void loop() {
  static uint32_t lastLoopMs = 0;
  const uint32_t nowMs = millis();
  const uint32_t loopDelta = nowMs - lastLoopMs;
  
  // Debug long loop times (>80ms indicates blocking)
  if (debugEnabled && lastLoopMs > 0 && loopDelta > 80) {
    Serial.print("DBG Loop delay: ");
    Serial.print(loopDelta);
    Serial.println("ms");
  }
  lastLoopMs = nowMs;

  // UART receive (line buffering)
  while (wifiSerial.available()) {
    char c = (char)wifiSerial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      rxBuf[rxLen] = '\0';
      handleLine(String(rxBuf));
      rxLen = 0;
    } else {
      if (rxLen < sizeof(rxBuf) - 1) rxBuf[rxLen++] = c;
    }
  }

  // Dead-man stop in teleop: only stop if motors are running AND timeout expires
  // This prevents premature stops that cause erratic behavior with ~30Hz joystick updates
  if (runMode == RunMode::Teleop) {
    const bool motorsActive = (lastCmdL != 0 || lastCmdR != 0);
    // Use fresh millis() to avoid underflow when lastMotorCmdMs was updated during this loop iteration
    const uint32_t currentMs = millis();
    const uint32_t timeSinceLastCmd = currentMs - lastMotorCmdMs;
    const bool timeoutExpired = timeSinceLastCmd > deadmanMs;
    
    if (motorsActive && timeoutExpired) {
      debugLog(String("Dead-man triggered: deltaT=") + timeSinceLastCmd + "ms (limit=" + deadmanMs + "ms)");
      motorsStop();
      sendLine("LOG dead-man stop");
    }
  }

  // Auto mode: stop if comms stale, else run light-follow behavior
  if (runMode == RunMode::LightFollow) {
    if ((nowMs - lastAnyCmdMs) > autoCommsTimeoutMs) {
      motorsStop();
      runMode = RunMode::Teleop;
    } else {
      lightFollowTick();
    }
  }

  // Dance mode: stop if comms stale, else run dance routine
  if (runMode == RunMode::Dance) {
    if ((nowMs - lastAnyCmdMs) > autoCommsTimeoutMs) {
      motorsStop();
      noTone(BUZZER_PIN); // Turn off buzzer on timeout
      runMode = RunMode::Teleop;
    } else {
      robotDanceTick(nowMs);
    }
  }

  melody.tick(nowMs);
  ledTick(nowMs);
  telemetryTick(nowMs);
}
