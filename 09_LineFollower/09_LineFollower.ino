/*
  ============================================================
  Project: Line Follower (3-IR, Simple State Table)
  Board: Kypruino UNO+ v0.8
  Platform: RoboRover Core (DRV8836 MODE=1: PHASE/ENABLE)
  ============================================================

  Description:
  A simple “state-machine-like” line follower using 3 IR sensors
  (Left, Center, Right) on a white floor with a black line.

  IMPORTANT:
  - Motor direction mapping:
      FORWARD  = P0=0, P1=0
      BACKWARD = P0=1, P1=1
  - Motor mapping:
      Motor A (D11) = RIGHT motor
      Motor B (D10) = LEFT motor

  Hardware:
  - IR line sensors: A0 (Left), A1 (Center), A2 (Right)
  - Motors:
      AENBL = D11 (RIGHT PWM)
      BENBL = D10 (LEFT  PWM)
      APHASE/BPHASE via expander @0x20 (P0/P1)
  ============================================================
*/

#include <Wire.h>

// -------------------- SENSOR PINS --------------------
#define PIN_IR_L A0
#define PIN_IR_C A1
#define PIN_IR_R A2

// -------------------- MOTOR PINS --------------------
#define PIN_AENBL 11          // RIGHT motor PWM
#define PIN_BENBL 10          // LEFT motor PWM
#define EXP_ADDR  0x20

// -------- Extras --------
#define BUZZER_PIN 9
#define BUZZER_FREQ 1000
#define BUZZER_DUR 125
#define PAUSE_DUR 125

// -------- For Threshold Calibration (computed in setup) --------
int threshL;
int threshC;
int threshR;

bool bL;  // black left (true or false)
bool bC;  // black center
bool bR;  // black right

// -------- Calibration settings --------
#define CAL_SAMPLES        200
#define CAL_PWM            90     // slow spin
#define CAL_SAMPLE_DELAY   25     // delay between samples [ms]
#define CAL_SWITCH_EVERY   50     // change spin direction every N samples

// -------------------- SPEEDS (TUNE IF NEEDED) --------------------
#define SPD_FWD     120
#define SPD_TURN    110
#define SPD_HARD    140
#define SPD_BACK    110

// -------------------- TIMINGS --------------------
#define LOOP_DELAY_MS     25
#define LOST_BACK_MS      250     // for case 000
#define LOST_STOP_MS      250

#define ALL_FWD_MS        200     // for case 111: move forward a bit
#define ALL_HOLD_LIMIT    800     // if stuck on 111 with no change, stop
#define ALL_CHANGE_EPS    25      // "significant change" placeholder (tune if needed)

// -------------------- EXPANDER STATE --------------------
uint8_t expState = 0xFF;

void expWrite(uint8_t v) {
  Wire.beginTransmission(EXP_ADDR);
  Wire.write(v);
  Wire.endTransmission();
  expState = v;
}


// ---------------- MOTOR CONTROL FUNCTIONS ----------------
void stopMotors() {
  analogWrite(PIN_AENBL, 0);
  analogWrite(PIN_BENBL, 0);
}

// Motor PWM apply (LEFT = B, RIGHT = A)
void setPWM(int leftPWM, int rightPWM) {
  leftPWM  = constrain(leftPWM,  0, 255);
  rightPWM = constrain(rightPWM, 0, 255);
  analogWrite(PIN_BENBL, leftPWM);
  analogWrite(PIN_AENBL, rightPWM);
}

// FORWARD  = P0=0, P1=0
// BACKWARD = P0=1, P1=1
// RIGHT    = P0=1, P1=0
// LEFT     = P0=0, P1=1
void setForwardDir() { expWrite(expState & 0b11111100); }
void setBackwardDir() { expWrite(expState | 0b00000011); }
void setRight() { expWrite((expState & 0b11111100) | 0b000001); }
void setLeft()  { expWrite((expState & 0b11111100) | 0b00000010); }

void buzzError() {
  tone(BUZZER_PIN, BUZZER_FREQ);
  delay(BUZZER_DUR);
  noTone(BUZZER_PIN);
  delay(PAUSE_DUR);
  tone(BUZZER_PIN, BUZZER_FREQ);
  delay(BUZZER_DUR);
  noTone(BUZZER_PIN);
  delay(PAUSE_DUR);
}

// -------------------- SETUP --------------------
void setup() {
  pinMode(BUZZER_PIN, OUTPUT);

  // Motor PWM pins
  pinMode(PIN_AENBL, OUTPUT);  // A = RIGHT motor (your test)
  pinMode(PIN_BENBL, OUTPUT);  // B = LEFT motor
  stopMotors();

  // I2C expander
  Wire.begin();
  expWrite(0xFF);

  // initial values for comparisons
  int minL = 1023;
  int minC = 1023;
  int minR = 1023;
  int maxL = 0;
  int maxC = 0;
  int maxR = 0;

  for (int i = 0; i < CAL_SAMPLES; i++) {
    // Alternate spin direction every CAL_SWITCH_EVERY samples
    // Spin LEFT:  right motor forward, left motor backward
    // Spin RIGHT: right motor backward, left motor forward
    if (i < (CAL_SAMPLES/4) | i > (3*(CAL_SAMPLES/4))){
      setLeft();
    } else {
      setRight();
    }
    
    // Keep speed slow and equal magnitude
    analogWrite(PIN_AENBL, CAL_PWM);  // RIGHT motor
    analogWrite(PIN_BENBL, CAL_PWM);  // LEFT motor

    // Read sensors
    int vL = analogRead(PIN_IR_L);
    int vC = analogRead(PIN_IR_C);
    int vR = analogRead(PIN_IR_R);

    // Update min/max
    if (vL < minL) {
      minL = vL;
    }
    if (vC < minC) {
      minC = vC;
    } 
    if (vR < minR) {
      minR = vR;
    }

    if (vL > maxL) {
      maxL = vL;
    }
    if (vC > maxC) {
      maxC = vC;
    }
    if (vR > maxR) {
      maxR = vR;
    }

    delay(CAL_SAMPLE_DELAY);
  }

  stopMotors();

  // Compute thresholds (midpoint)
  threshL = (minL + maxL) / 2;
  threshC = (minC + maxC) / 2;
  threshR = (minR + maxR) / 2;
}


// -------------------- MAIN LOOP --------------------
void loop() {
  // Read sensors
  int vL = analogRead(PIN_IR_L);
  int vC = analogRead(PIN_IR_C);
  int vR = analogRead(PIN_IR_R);

  // Convert to black/white bits
  if (vL < threshL) {
    bL = 1; // Black detected on left side
  } else {
    bL = 0;
  }

  if (vC < threshC) {
    bC = 1; // Black detected in center
  } else {
    bC = 0;
  }

  if (vR < threshR) {
    bR = 1; // Black detected on Right side
  } else {
    bR = 0;
  }

  // Actions
  if ( (bC & !(bR | bL)) | (bL & bC & bR) ) {  // if (BlackCenter AND NOT (BlackRight OR BlackLeft)) OR (all sensors detect black) (case: straight on line OR thick line/intersection)
    setForwardDir();
    setPWM(SPD_FWD, SPD_FWD);
  } else if (bR & !bL) {  // if BlackRight AND NOT BlackLeft (case: line is on the right side)
    if (bC) {           // if BlackCenter (case: line is slightly to the right)
      setForwardDir();
      setPWM(SPD_HARD, SPD_TURN); //soft right turn
    } else {            // if NOT BlackCenter (case: line is further to the right)
      setForwardDir();
      setPWM(SPD_HARD, 0);  //hard right turn
    }
  } else if (bL & !bR) {  // if BlackLeft AND NOT BlackRight (case: line is on the left side)
    if (bC) {           // if BlackCenter (case: line is slightly to the left)
      setForwardDir();
      setPWM(SPD_TURN, SPD_HARD); //soft left turn
    } else {            // if NOT BlackCenter (case: line is further to the left)
      setForwardDir();
      setPWM(0, SPD_HARD);  //hard left turn
    }
  } else if (!bL & !bC & !bR) { // case: all sensors lost the line
    stopMotors();
    buzzError();
    delay(200);
    setBackwardDir();
    setPWM(SPD_BACK, SPD_BACK);
    delay(500);
  } else {  // case: undefined/error
    stopMotors();
    buzzError();
    delay(200);
  }

  delay(LOOP_DELAY_MS);
}
