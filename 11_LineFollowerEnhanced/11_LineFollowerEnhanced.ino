/*
  ============================================================
  Project: Enhanced Line Follower (3-IR, considering previous state)
  Board: Kypruino UNO+ v0.8
  Platform: RoboRover Core (DRV8836 MODE=1: PHASE/ENABLE)
  ============================================================

  Description:
  A line follower using 3 IR sensors (Left, Center, Right) on a white floor with
  a black line and logic considering the previous state. Designed to incorporate
  obstacle avoidance and the ability to traverse over broken lines and perform
  right angled turns (not competition-ready).

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
  - Ultrasonic sensor:
      TRIG = D4
      ECHO = D5 
  ============================================================
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>

// -------------------- IR SENSOR PINS --------------------
#define PIN_IR_L A0
#define PIN_IR_C A1
#define PIN_IR_R A2

// -------------------- ULTRASONIC --------------------
#define PIN_TRIG 4
#define PIN_ECHO 5
#define ECHO_TIMEOUT_US 30000UL // 30000 Unsigned Long [μs]

// -------------------- MOTOR PINS --------------------
#define PIN_AENBL 11          // RIGHT motor PWM
#define PIN_BENBL 10          // LEFT motor PWM
#define EXP_ADDR  0x20

// -------------------- EXPANDER DEFAULT STATE --------------------
uint8_t expState = 0xFF;

// -------------------- OLED SETTINGS --------------------
// 0.91" OLED is 128x32
#define OLED_WIDTH   128
#define OLED_HEIGHT  32
#define OLED_ADDR    0x3C

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

// -------- Calibration settings --------
#define CAL_SAMPLES        160
#define CAL_PWM            120 

// -------------------- SPEEDS (TUNE IF NEEDED) --------------------
#define SPD_TURN    125
#define SPD_HARD    180

#define SPD_FWD       165
#define SPD_SLOW      90
#define SPD_SOFT_LOW  155 // Low/High refers to one side being slower than the other for the corresponding turn
#define SPD_SOFT_HIGH 175
#define SPD_TURN_LOW  85
#define SPD_TURN_HIGH 185



// -------- Declaration of variables --------
bool bL;  // black left (true or false)
bool bC;  // black center
bool bR;  // black right

int threshL;
int threshC;
int threshR;

uint8_t prevState = 0;
uint8_t state;

// -------------------- SETUP --------------------
void setup() {
  // Ultrasonic
  pinMode(PIN_TRIG, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);

  // Motor PWM pins
  pinMode(PIN_AENBL, OUTPUT);  // A = RIGHT motor (your test)
  pinMode(PIN_BENBL, OUTPUT);  // B = LEFT motor
  stopMotors();

  // I2C expander
  Wire.begin();
  expWrite(0xFF);

  // OLED init
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.display();

  // initial values for comparisons
  int minL = 1023;
  int minC = 1023;
  int minR = 1023;
  int maxL = 0;
  int maxC = 0;
  int maxR = 0;

  // CALIBRATION PART - Place the robot on the line and allow it to take readings from the line and the ground
  for (int i = 0; i < CAL_SAMPLES; i++) {
    oledShowCalibrating(i + 1, CAL_SAMPLES);

    // Spin LEFT:  right motor forward, left motor backward
    // Spin RIGHT: right motor backward, left motor forward
    if (i < (CAL_SAMPLES/4) | i > (3*(CAL_SAMPLES/4))){ // setting direction to spin during calibration phase
      setLeft();  // for the first 25% and last 25% of samples, spin left
    } else {
      setRight(); // for the 25-75% of the samples, spin right
    }
    
    // setting spin speed for calibrtation phase 
    analogWrite(PIN_AENBL, CAL_PWM);  // RIGHT motor
    analogWrite(PIN_BENBL, CAL_PWM);  // LEFT motor

    // Read sensors
    int vL = analogRead(PIN_IR_L);
    int vC = analogRead(PIN_IR_C);
    int vR = analogRead(PIN_IR_R);

    // Update min/max values found
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

    delay(10);
  }

  stopMotors();

  // Compute thresholds (midpoint of min-max)
  threshL = (minL + maxL) / 2;
  threshC = (minC + maxC) / 2;
  threshR = (minR + maxR) / 2;
}


// -------------------- MAIN LOOP --------------------
void loop() {
  bool delayed = 0; // a variable created to check if there needs to be a loop delay applied or not
  
  // Read sensors
  int vL = analogRead(PIN_IR_L);
  int vC = analogRead(PIN_IR_C);
  int vR = analogRead(PIN_IR_R);

  // Convert to black/white bits
  bL = (vL < threshL);  // TRUE (1) or FALSE (0)
  bC = (vC < threshC);
  bR = (vR < threshR);
  oledShowBits(bL, bC, bR);
  state = (bL << 2) | (bC << 1) | (bR); // Arranges in binary bL-bC-bR
  
  // Read ultrasonic distance (cm)
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long us = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);  // [μs]
  
  float distanceCm = us / 58.0;


  // Actions
  //NOTE: in case of 0b000, prevState does not update to 0b000 by the end of the loop - dead state - should do something until it returns to a line to continue loop

  if (distanceCm < 20 && distanceCm > 0) {  // Tune if needed
    // GO_AROUND_OBSTACLE - experimental, needs tuning - should search to re-enter line, handling states at the same time
    /*
    hardRight(500);
    forwards();
    delay(1300);
    hardLeft(1000);
    forwards();
    delay(1000);
    hardRight(400);
    bL = (analogRead(PIN_IR_L) < threshL);
    bC = (analogRead(PIN_IR_C) < threshC);
    bR = (analogRead(PIN_IR_R) < threshR);
    state = (bL << 2) | (bC << 1) | (bR);
    while (state == 0b000) {
      slowFwd();
      delay(10);
      bL = (analogRead(PIN_IR_L) < threshL);
      bC = (analogRead(PIN_IR_C) < threshC);
      bR = (analogRead(PIN_IR_R) < threshR);
      state = (bL << 2) | (bC << 1) | (bR);
    }
    */
    stopMotors(); // for now, stop when detecting an obstacle
  } else {
    if (bC) { // Center is on top of the line
      if (state == 0b010) { // Only center is on line 
        if (prevState == 0b110) { // Left TRUE, Center TRUE, Right FALSE
          //turn slightly right for orientation fix
          right();
        } else if (prevState == 0b011) {  // Left FALSE, Center TRUE, Right TRUE
          //turn slightly left for orientation fix
          left();
        } else {
          //go straight
          forwards();
        }
      } else if (state == 0b111) {  // LCR
        slowFwd();
      } else if (state == 0b110) {  // LCr
        softLeft();
      } else if (state == 0b011) {  // lCR
        softRight();
      }
    } else {  // Center lost line (bC != 1)
      if (state == 0b000 && prevState == 0b010) { // Line break detected (will only work if it detects the line break while being perferctly straight)
        //move forwards until next line - must handle states until new line 
        while (state == 0b000) {
          forwards();
          delay(10);
          bL = (analogRead(PIN_IR_L) < threshL);
          bC = (analogRead(PIN_IR_C) < threshC);
          bR = (analogRead(PIN_IR_R) < threshR);
          state = (bL << 2) | (bC << 1) | (bR);
        }
        delayed = 1;
      } else {  // Hard turning needed
        if (state == 0b000) {
          if (prevState == 0b100 || prevState == 0b110) {
            //HARD turn left until bC gets triggered - WHAT IF it was actually a line break?
            while (bC == 0) {
              hardLeft(10);
              bC = (analogRead(PIN_IR_C) < threshC);
            }
            delayed = 1;
            bL = (analogRead(PIN_IR_L) < threshL);
            bR = (analogRead(PIN_IR_R) < threshR);
            state = (bL << 2) | (bC << 1) | (bR); // to update previous state
          } else if (prevState == 0b001 || prevState == 0b011) {
            //HARD turn right until bC gets triggered -WHAT IF it was actually a line break?
            while (bC == 0) {
              hardRight(10);
              bC = (analogRead(PIN_IR_C) < threshC);
            }
            delayed = 1;
            bL = (analogRead(PIN_IR_L) < threshL);
            bR = (analogRead(PIN_IR_R) < threshR);
            state = (bL << 2) | (bC << 1) | (bR); // to update previous state
          }
        } else {
          if (state == 0b100) {
            left();
          } else if (state == 0b001) {
            right();
          } else { //101 - undefined/error case
            slowFwd();
          }
        }
      }
    }
  }

  prevState = state;
  if (!delayed) {
    delay(10);
  }
}


// ---------- Expander helper ----------
void expWrite(uint8_t v) {
  Wire.beginTransmission(EXP_ADDR);
  Wire.write(v);
  Wire.endTransmission();
  expState = v;
}

// ---------------- MOTOR CONTROL FUNCTIONS ----------------
// FORWARD  = P0=0, P1=0
// BACKWARD = P0=1, P1=1
// RIGHT    = P0=1, P1=0
// LEFT     = P0=0, P1=1
void setForwardDir() { expWrite(expState & 0b11111100); }
void setBackwardDir() { expWrite(expState | 0b00000011); }
void setRight() { expWrite((expState & 0b11111100) | 0b000001); }
void setLeft()  { expWrite((expState & 0b11111100) | 0b00000010); }

// Motor PWM apply (LEFT = B, RIGHT = A)
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

void forwards() {
  setForwardDir();
  setPWM(SPD_FWD, SPD_FWD);
}

void softRight() {
  setForwardDir();
  setPWM(SPD_SOFT_HIGH, SPD_SOFT_LOW);
}

void slowFwd() {
  setForwardDir();
  setPWM(SPD_SLOW, SPD_SLOW);
}

void softLeft() {
  setForwardDir();
  setPWM(SPD_SOFT_LOW, SPD_SOFT_HIGH);
}

void right() {
  setForwardDir();
  setPWM(SPD_TURN_HIGH, SPD_TURN_LOW);
}

void left() {
  setForwardDir();
  setPWM(SPD_TURN_LOW, SPD_TURN_HIGH);
}

void hardRight(int del) {
  setRight();
  setPWM(100, 100);
  delay(del);
}

void hardLeft(int del) {
  setLeft();
  setPWM(100, 100);
  delay(del);
}

// -------------------- OLED HELPERS --------------------
void oledShowCalibrating(int i, int total) {
  // Simple progress bar at the bottom
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Calibrating");

  // Progress bar
  int barY = OLED_HEIGHT - 8;
  int barW = OLED_WIDTH - 2;
  int filled = (int)((long)barW * i / total);

  display.drawRect(0, barY, barW, 7, SSD1306_WHITE);
  display.fillRect(1, barY + 1, filled, 5, SSD1306_WHITE);

  display.display();
}

void oledShowBits(bool L, bool C, bool R) {
  char bits[4];
  bits[0] = L ? '1' : '0';
  bits[1] = C ? '1' : '0';
  bits[2] = R ? '1' : '0';
  bits[3] = '\0';

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Big text
  display.setTextSize(4);

  // Centering with default 5x7 font: char width = 6 * textSize
  int textW = 3 * 6 * 4; // 3 chars
  int x = (OLED_WIDTH - textW) / 2;
  int y = (OLED_HEIGHT - (8 * 4)) / 2; // char height ~ 8*size

  if (x < 0) x = 0;
  if (y < 0) y = 0;

  display.setCursor(x, y);
  display.print(bits);
  display.display();
}