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
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// -------------------- SENSOR PINS --------------------
#define PIN_IR_L A0
#define PIN_IR_C A1
#define PIN_IR_R A2

// -------------------- MOTOR PINS --------------------
#define PIN_AENBL 11          // RIGHT motor PWM
#define PIN_BENBL 10          // LEFT motor PWM
#define EXP_ADDR  0x20

// -------------------- OLED SETTINGS --------------------
// 0.91" OLED is 128x32.
#define OLED_WIDTH   128
#define OLED_HEIGHT  32
#define OLED_ADDR    0x3C

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

// -------- Calibration settings --------
#define CAL_SAMPLES        160
#define CAL_PWM            120     // Calibration spin speed

// -------------------- SPEEDS (TUNE IF NEEDED) --------------------
#define SPD_FWD     165
#define SPD_TURN    125
#define SPD_HARD    180
#define SPD_SLOW    90

// -------------------- EXPANDER DEFAULT STATE --------------------
uint8_t expState = 0xFF;

// -------- Declaration of variables --------
bool bL;  // black left (true or false)
bool bC;  // black center
bool bR;  // black right

int threshL;
int threshC;
int threshR;


// -------------------- SETUP --------------------
void setup() {
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
  // Read sensors
  int vL = analogRead(PIN_IR_L);
  int vC = analogRead(PIN_IR_C);
  int vR = analogRead(PIN_IR_R);

  oledShowBits(bL, bC, bR);

  // Convert to black/white bits
  if (vL < threshL) {
    bL = 1; // Black detected on Left side - TRUE (1)
  } else {
    bL = 0; // FALSE (0)
  }

  if (vC < threshC) {
    bC = 1; // Black detected in Center
  } else {
    bC = 0;
  }

  if (vR < threshR) {
    bR = 1; // Black detected on Right side
  } else {
    bR = 0;
  }

  // Actions
  if ( (bC && !(bR || bL)) || (bL && bC && bR) ) {  // if (BlackCenter AND NOT (BlackRight OR BlackLeft)) OR (all sensors detect black) (case: straight on line OR thick line/intersection)
    setForwardDir();
    setPWM(SPD_FWD, SPD_FWD);
  } else if (bR && !bL) {  // if BlackRight TRUE AND BlackLeft FALSE (case: line is on the right side)
    if (bC) {           // if BlackCenter TRUE (case: line is only slightly to the right)
      setForwardDir();
      setPWM(SPD_HARD, SPD_TURN); //soft right turn
    } else {            // if BlackCenter FALSE (case: line is further to the right)
      setForwardDir();
      setPWM(SPD_HARD, SPD_SLOW);  //hard right turn
    }
  } else if (bL && !bR) {  // if BlackLeft TRUE AND BlackRight FALSE (case: line is on the left side)
    if (bC) {           // if BlackCenter TRUE (case: line is only slightly to the left)
      setForwardDir();
      setPWM(SPD_TURN, SPD_HARD); //soft left turn
    } else {            // if BlackCenter FALSE (case: line is further to the left)
      setForwardDir();
      setPWM(SPD_SLOW, SPD_HARD);  //hard left turn
    }
  } else if (!bL && !bC && !bR) { // if all sensors FALSE (case: all sensors lost the line)
    stopMotors();
    delay(20);
    setBackwardDir();
    setPWM(SPD_SLOW, SPD_SLOW);
    delay(500);
  } else {  // case: undefined/101
    stopMotors();
    delay(20);
    setForwardDir();
    setPWM(SPD_SLOW,SPD_SLOW);
    delay(200);
  }

  delay(10);
}


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

// Motor PWM (LEFT = B, RIGHT = A)
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
