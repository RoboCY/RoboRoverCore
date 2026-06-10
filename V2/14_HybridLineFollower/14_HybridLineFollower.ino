/*
  ============================================================
  Project: Hybrid PD Line Follower (latched recovery)
  Board: Kypruino UNO+ v0.8
  Platform: RoboRover Core 2
  ============================================================

  Description:
  Hybrid line follower for RoboRover Core 2 using:
  - 5 analog line sensors through TLA2528
  - startup calibration
  - PD control for normal tracking
  - special-case logic for:
      * line breaks
      * latched forward-arc recovery after line loss
      * 90-degree turn handling
      * obstacle stop

  Important behavior:
  - PD only runs when the line is in a properly trackable state
  - if the line is lost to one side, the robot enters a recovery mode
    and stays there until the line is actually found again
  - recovery uses forward motion with heavy steering, not pure spin
  - 00000 must NEVER return directly to PD

  Sensor order encoded in this sketch:
    IR4  IR3  IR2  IR1  IR0
    left     center    right

  Internal PD weights:
    IR4=-4, IR3=-2, IR2=0, IR1=+2, IR0=+4
  ============================================================
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// -------------------- I2C ADDRESSES --------------------
#define EXP_ADDR   0x20
#define TLA_ADDR   0x14

// -------------------- TLA2528 --------------------
#define TLA_OP_WRITE_REG     0x08
#define TLA_REG_CHANNEL_SEL  0x11

// -------------------- MOTOR PINS --------------------
#define PIN_AENBL 11   // RIGHT motor PWM
#define PIN_BENBL 10   // LEFT motor PWM

// -------------------- ULTRASONIC --------------------
#define PIN_TRIG 4
#define PIN_ECHO 5
#define ECHO_TIMEOUT_US   30000UL
#define OBSTACLE_STOP_CM  20.0f

// -------------------- OLED --------------------
#define OLED_WIDTH   128
#define OLED_HEIGHT   32
#define OLED_ADDR    0x3C
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

// -------------------- CALIBRATION --------------------
#define CAL_SAMPLES  220
#define CAL_PWM      115

// -------------------- PD CONTROL --------------------
float Kp = 38.0f;
float Kd = 95.0f;

#define BASE_SPEED_MAX   140
#define BASE_SPEED_MIN    70
#define SPEED_REDUCTION   35.0f

#define MAX_CORRECTION   130.0f
#define PWM_MAX          220

// -------------------- SPECIAL CASE SPEEDS --------------------
#define BREAK_FORWARD_PWM     85

// forward-arc recovery
#define RECOVER_LEFT_LOW      60
#define RECOVER_LEFT_HIGH    180
#define RECOVER_RIGHT_LOW     60
#define RECOVER_RIGHT_HIGH   180

// 90-degree turn handling
#define TURN90_SPIN_PWM      120

// -------------------- SUPERVISORY TIMING --------------------
#define LOOP_DELAY_MS           10
#define LOST_LINE_THRESHOLD    280     // PD confidence only
#define TURN90_CONFIRM_LOOPS     3
#define LINE_BREAK_TIMEOUT_MS  180
#define TURN90_TIMEOUT_MS     1200

// -------------------- STATE --------------------
uint8_t expState = 0xFF;

int rawIR[5]      = {0, 0, 0, 0, 0};   // index 0..4 = IR0..IR4
int minIR[5]      = {4095, 4095, 4095, 4095, 4095};
int maxIR[5]      = {0, 0, 0, 0, 0};
int threshIR[5]   = {0, 0, 0, 0, 0};   // raw midpoint thresholds
int strengthIR[5] = {0, 0, 0, 0, 0};   // 0..1000, higher = more line

// weights correspond to IR0..IR4
const int weights[5] = {+4, +2, 0, -2, -4};

float error = 0.0f;
float prevError = 0.0f;
int lastSeenSign = 0;   // -1 = line last seen left, +1 = right, 0 = unknown

uint8_t state5 = 0;
uint8_t prevState5 = 0;

uint8_t strongLeftCount = 0;
uint8_t strongRightCount = 0;

enum FollowMode {
  MODE_PD = 0,
  MODE_LINE_BREAK,
  MODE_RECOVER_LEFT,
  MODE_RECOVER_RIGHT,
  MODE_TURN_90_LEFT,
  MODE_TURN_90_RIGHT,
  MODE_OBSTACLE_STOP
};

FollowMode mode = MODE_PD;
unsigned long modeStartMs = 0;

// ============================================================
//                           SETUP
// ============================================================
void setup() {
  pinMode(PIN_AENBL, OUTPUT);
  pinMode(PIN_BENBL, OUTPUT);
  stopMotors();

  pinMode(PIN_TRIG, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);

  Wire.begin();
  expWrite(0xFF);

  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.display();

  calibrateSensors();

  prevError = 0.0f;
  prevState5 = 0;
}

// ============================================================
//                            LOOP
// ============================================================
void loop() {
  float distanceCm = readUltrasonicCm();

  if (distanceCm > 0 && distanceCm < OBSTACLE_STOP_CM) {
    mode = MODE_OBSTACLE_STOP;
    stopMotors();
    oledShowObstacle(distanceCm);
    delay(LOOP_DELAY_MS);
    return;
  } else if (mode == MODE_OBSTACLE_STOP) {
    mode = MODE_PD;
  }

  readLineSensors();
  computeStrengths();
  state5 = computeBinaryState();
  updateLastSeenSign();   // only updates when line is confidently present

  handleModeTransitions();

  switch (mode) {
    case MODE_PD:
      runPDMode();
      break;

    case MODE_LINE_BREAK:
      runLineBreakMode();
      break;

    case MODE_RECOVER_LEFT:
      runRecoverLeftMode();
      break;

    case MODE_RECOVER_RIGHT:
      runRecoverRightMode();
      break;

    case MODE_TURN_90_LEFT:
      runTurn90LeftMode();
      break;

    case MODE_TURN_90_RIGHT:
      runTurn90RightMode();
      break;

    case MODE_OBSTACLE_STOP:
      stopMotors();
      break;
  }

  prevState5 = state5;
  delay(LOOP_DELAY_MS);
}

// ============================================================
//                    SUPERVISORY DECISIONS
// ============================================================
void handleModeTransitions() {
  // Already in a special mode? Stay there.
  // Only that mode's own reacquire logic is allowed to return to PD.
  if (mode != MODE_PD) {
    return;
  }

  if (isStrongLeftState(state5) && isCenteredishState(prevState5)) {
    strongLeftCount++;
  } else {
    strongLeftCount = 0;
  }

  if (isStrongRightState(state5) && isCenteredishState(prevState5)) {
    strongRightCount++;
  } else {
    strongRightCount = 0;
  }

  // 90-degree turn detection
  if (strongLeftCount >= TURN90_CONFIRM_LOOPS) {
    mode = MODE_TURN_90_LEFT;
    modeStartMs = millis();
    return;
  }

  if (strongRightCount >= TURN90_CONFIRM_LOOPS) {
    mode = MODE_TURN_90_RIGHT;
    modeStartMs = millis();
    return;
  }

  // Full loss of line
  if (state5 == 0b00000) {
    if (isCenteredishState(prevState5)) {
      mode = MODE_LINE_BREAK;
      modeStartMs = millis();
      return;
    }

    if (isLeftHeavyState(prevState5)) {
      mode = MODE_RECOVER_LEFT;
      modeStartMs = millis();
      return;
    }

    if (isRightHeavyState(prevState5)) {
      mode = MODE_RECOVER_RIGHT;
      modeStartMs = millis();
      return;
    }
  }

  // Edge-only states can also mean practical loss on tight turns
  if (isEdgeOnlyLeftState(state5) && isLeftHeavyState(prevState5)) {
    mode = MODE_RECOVER_LEFT;
    modeStartMs = millis();
    return;
  }

  if (isEdgeOnlyRightState(state5) && isRightHeavyState(prevState5)) {
    mode = MODE_RECOVER_RIGHT;
    modeStartMs = millis();
    return;
  }
}

// ============================================================
//                       MODE EXECUTION
// ============================================================
void runPDMode() {
  // PD only on properly trackable states
  if (!isTrackableState(state5)) {
    stopMotors();
    oledShowSpecial("WAIT ");
    return;
  }

  int totalStrength = 0;
  long weightedSum = 0;

  for (int i = 0; i < 5; i++) {
    totalStrength += strengthIR[i];
    weightedSum += (long)strengthIR[i] * weights[i];
  }

  if (totalStrength < LOST_LINE_THRESHOLD) {
    stopMotors();
    oledShowSpecial("WEAK ");
    return;
  }

  error = (float)weightedSum / (float)max(totalStrength, 1);

  float derivative = error - prevError;
  float correction = Kp * error + Kd * derivative;
  prevError = error;

  if (correction > MAX_CORRECTION) correction = MAX_CORRECTION;
  if (correction < -MAX_CORRECTION) correction = -MAX_CORRECTION;

  float baseSpeed = BASE_SPEED_MAX - SPEED_REDUCTION * abs(error);
  if (baseSpeed < BASE_SPEED_MIN) baseSpeed = BASE_SPEED_MIN;
  if (baseSpeed > BASE_SPEED_MAX) baseSpeed = BASE_SPEED_MAX;

  int leftPWM  = (int)(baseSpeed + correction);
  int rightPWM = (int)(baseSpeed - correction);

  leftPWM  = constrain(leftPWM,  0, PWM_MAX);
  rightPWM = constrain(rightPWM, 0, PWM_MAX);

  setForwardDir();
  setPWM(leftPWM, rightPWM);

  oledShowRun("PD   ", error, leftPWM, rightPWM);
}

void runLineBreakMode() {
  // ONLY return to PD if line is really reacquired
  if (isStrongReacquireState(state5)) {
    prevError = 0.0f;
    mode = MODE_PD;
    return;
  }

  // centered loss => likely gap, continue straight briefly
  setForwardDir();
  setPWM(BREAK_FORWARD_PWM, BREAK_FORWARD_PWM);
  oledShowSpecial("BREAK");

  // after short timeout, convert to side recovery
  if (millis() - modeStartMs > LINE_BREAK_TIMEOUT_MS) {
    if (lastSeenSign < 0) {
      mode = MODE_RECOVER_LEFT;
      modeStartMs = millis();
    } else if (lastSeenSign > 0) {
      mode = MODE_RECOVER_RIGHT;
      modeStartMs = millis();
    } else {
      // no side info: stay in break mode and keep trying forward
      modeStartMs = millis();
    }
  }
}

void runRecoverLeftMode() {
  // Stay here UNTIL line is truly reacquired
  if (isStrongReacquireState(state5)) {
    prevError = 0.0f;
    mode = MODE_PD;
    return;
  }

  recoverLeftArc();
  oledShowSpecial("REC L");
}

void runRecoverRightMode() {
  if (isStrongReacquireState(state5)) {
    prevError = 0.0f;
    mode = MODE_PD;
    return;
  }

  recoverRightArc();
  oledShowSpecial("REC R");
}

void runTurn90LeftMode() {
  // Only exit when line is really reacquired
  if (isStrongReacquireState(state5)) {
    stopMotors();
    prevError = 0.0f;
    mode = MODE_PD;
    return;
  }

  // true 90-degree handling: spin in place
  setLeft();
  setPWM(TURN90_SPIN_PWM, TURN90_SPIN_PWM);
  oledShowSpecial("L90  ");

  // after long turn timeout, fall into left recovery arc, not PD
  if (millis() - modeStartMs > TURN90_TIMEOUT_MS) {
    mode = MODE_RECOVER_LEFT;
    modeStartMs = millis();
  }
}

void runTurn90RightMode() {
  if (isStrongReacquireState(state5)) {
    stopMotors();
    prevError = 0.0f;
    mode = MODE_PD;
    return;
  }

  setRight();
  setPWM(TURN90_SPIN_PWM, TURN90_SPIN_PWM);
  oledShowSpecial("R90  ");

  if (millis() - modeStartMs > TURN90_TIMEOUT_MS) {
    mode = MODE_RECOVER_RIGHT;
    modeStartMs = millis();
  }
}

// ============================================================
//                    STATE CLASSIFICATION
// ============================================================
bool isTrackableState(uint8_t s) {
  return (s == 0b00100 ||
          s == 0b01100 ||
          s == 0b00110 ||
          s == 0b01110);
}

bool isStrongReacquireState(uint8_t s) {
  return (s == 0b00100 ||
          s == 0b01100 ||
          s == 0b00110 ||
          s == 0b01110);
}

bool isCenteredishState(uint8_t s) {
  return (s == 0b00100 ||
          s == 0b01100 ||
          s == 0b00110 ||
          s == 0b01110 ||
          s == 0b01000 ||
          s == 0b00010);
}

bool isStrongLeftState(uint8_t s) {
  return (s == 0b11100 ||
          s == 0b11000 ||
          s == 0b10000 ||
          s == 0b11110);
}

bool isStrongRightState(uint8_t s) {
  return (s == 0b00111 ||
          s == 0b00011 ||
          s == 0b00001 ||
          s == 0b01111);
}

bool isLeftHeavyState(uint8_t s) {
  return (s == 0b11100 ||
          s == 0b11000 ||
          s == 0b10000 ||
          s == 0b01100 ||
          s == 0b01000);
}

bool isRightHeavyState(uint8_t s) {
  return (s == 0b00111 ||
          s == 0b00011 ||
          s == 0b00001 ||
          s == 0b00110 ||
          s == 0b00010);
}

bool isEdgeOnlyLeftState(uint8_t s) {
  return (s == 0b01000 ||
          s == 0b10000 ||
          s == 0b11000);
}

bool isEdgeOnlyRightState(uint8_t s) {
  return (s == 0b00010 ||
          s == 0b00001 ||
          s == 0b00011);
}

// ============================================================
//                     SENSOR PROCESSING
// ============================================================
void readLineSensors() {
  for (int i = 0; i < 5; i++) {
    rawIR[i] = tlaReadChannel(i);
  }
}

void computeStrengths() {
  for (int i = 0; i < 5; i++) {
    int span = maxIR[i] - minIR[i];

    if (span < 10) {
      strengthIR[i] = 0;
      continue;
    }

    int raw = constrain(rawIR[i], minIR[i], maxIR[i]);

    // Assumption: black line gives LOWER reading than white floor
    int s = map(raw, minIR[i], maxIR[i], 1000, 0);

    // If opposite polarity, swap to:
    // int s = map(raw, minIR[i], maxIR[i], 0, 1000);

    strengthIR[i] = constrain(s, 0, 1000);
  }
}

uint8_t computeBinaryState() {
  // Binary supervisory logic uses RAW thresholds from calibration
  bool b0 = (rawIR[0] < threshIR[0]); // IR0
  bool b1 = (rawIR[1] < threshIR[1]); // IR1
  bool b2 = (rawIR[2] < threshIR[2]); // IR2
  bool b3 = (rawIR[3] < threshIR[3]); // IR3
  bool b4 = (rawIR[4] < threshIR[4]); // IR4

  // If opposite polarity, use > instead of <

  return (b4 << 4) | (b3 << 3) | (b2 << 2) | (b1 << 1) | b0;
}

void updateLastSeenSign() {
  // Only update while line is confidently present.
  int totalStrength = 0;
  long weightedSum = 0;

  for (int i = 0; i < 5; i++) {
    totalStrength += strengthIR[i];
    weightedSum += (long)strengthIR[i] * weights[i];
  }

  if (totalStrength >= LOST_LINE_THRESHOLD && state5 != 0b00000) {
    float e = (float)weightedSum / (float)totalStrength;

    if (e < -0.15f) lastSeenSign = -1;
    else if (e > 0.15f) lastSeenSign = +1;
    // else keep previous sign
  }
}

// ============================================================
//                        CALIBRATION
// ============================================================
void calibrateSensors() {
  for (int i = 0; i < 5; i++) {
    minIR[i] = 4095;
    maxIR[i] = 0;
  }

  for (int i = 0; i < CAL_SAMPLES; i++) {
    oledShowCalibrating(i + 1, CAL_SAMPLES);

    if (i < (CAL_SAMPLES / 4) || i > (3 * (CAL_SAMPLES / 4))) {
      setLeft();
    } else {
      setRight();
    }

    setPWM(CAL_PWM, CAL_PWM);

    for (int s = 0; s < 5; s++) {
      int v = tlaReadChannel(s);
      if (v < minIR[s]) minIR[s] = v;
      if (v > maxIR[s]) maxIR[s] = v;
    }

    delay(10);
  }

  stopMotors();

  for (int i = 0; i < 5; i++) {
    if (maxIR[i] - minIR[i] < 20) {
      maxIR[i] = minIR[i] + 20;
    }
    threshIR[i] = (minIR[i] + maxIR[i]) / 2;
  }

  oledShowCalDone();
  delay(500);
}

// ============================================================
//                     TLA2528 HELPERS
// ============================================================
void tlaWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission((uint8_t)TLA_ADDR);
  Wire.write(TLA_OP_WRITE_REG);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint16_t tlaReadChannel(uint8_t channel) {
  channel &= 0x07;

  tlaWriteReg(TLA_REG_CHANNEL_SEL, channel);
  delayMicroseconds(30);

  Wire.requestFrom((uint8_t)TLA_ADDR, (uint8_t)2);
  if (Wire.available() >= 2) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    uint16_t raw = ((uint16_t)msb << 8) | lsb;
    return (raw >> 4) & 0x0FFF;
  }

  return 0;
}

// ============================================================
//                     ULTRASONIC
// ============================================================
float readUltrasonicCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long us = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);
  if (us == 0) return -1.0f;
  return us / 58.0f;
}

// ============================================================
//                     EXPANDER / MOTORS
// ============================================================
void expWrite(uint8_t v) {
  Wire.beginTransmission((uint8_t)EXP_ADDR);
  Wire.write(v);
  Wire.endTransmission();
  expState = v;
}

void setForwardDir()  { expWrite(expState & 0b11111100); }
void setBackwardDir() { expWrite(expState | 0b00000011); }
void setRight()       { expWrite((expState & 0b11111100) | 0b00000001); }
void setLeft()        { expWrite((expState & 0b11111100) | 0b00000010); }

void setPWM(int leftPWM, int rightPWM) {
  leftPWM  = constrain(leftPWM,  0, 255);
  rightPWM = constrain(rightPWM, 0, 255);

  analogWrite(PIN_BENBL, leftPWM);
  analogWrite(PIN_AENBL, rightPWM);
}

void stopMotors() {
  analogWrite(PIN_AENBL, 0);
  analogWrite(PIN_BENBL, 0);
}

void recoverLeftArc() {
  setForwardDir();
  setPWM(RECOVER_LEFT_LOW, RECOVER_LEFT_HIGH);
}

void recoverRightArc() {
  setForwardDir();
  setPWM(RECOVER_RIGHT_HIGH, RECOVER_RIGHT_LOW);
}

// ============================================================
//                        OLED HELPERS
// ============================================================
void oledShowCalibrating(int i, int total) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Calibrating");

  int barY = OLED_HEIGHT - 8;
  int barW = OLED_WIDTH - 2;
  int filled = (int)((long)barW * i / total);

  display.drawRect(0, barY, barW, 7, SSD1306_WHITE);
  display.fillRect(1, barY + 1, filled, 5, SSD1306_WHITE);
  display.display();
}

void oledShowCalDone() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(28, 10);
  display.print("Cal done");
  display.display();
}

void oledShowObstacle(float d) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Obstacle");
  display.setCursor(0, 16);
  display.print(d, 1);
  display.print(" cm");
  display.display();
}

void oledShowSpecial(const char* modeText) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.print("Special");

  display.setCursor(0, 12);
  display.print(modeText);

  display.setCursor(0, 24);
  printStateBits(state5);

  display.display();
}

void oledShowRun(const char* modeText, float err, int leftPWM, int rightPWM) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.print(modeText);

  display.setCursor(38, 0);
  display.print("E:");
  display.print(err, 2);

  display.setCursor(0, 12);
  printStateBits(state5);

  display.setCursor(0, 24);
  display.print("L");
  display.print(leftPWM);
  display.print(" R");
  display.print(rightPWM);

  display.display();
}

void printStateBits(uint8_t s) {
  display.print((s & 0b10000) ? '1' : '0');
  display.print((s & 0b01000) ? '1' : '0');
  display.print((s & 0b00100) ? '1' : '0');
  display.print((s & 0b00010) ? '1' : '0');
  display.print((s & 0b00001) ? '1' : '0');
}