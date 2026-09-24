// 10_ChromaTiltSphere_V3 - tilt the rover to roll a ball around the screen.
// Nothing moves. Needs the U8g2 library.
//
// The ball rolls downhill, the way a real one would. The lights follow it:
// rolling left and right changes their colour, and rolling up and down
// changes how bright they are.

#include <RoboRoverCore3.h>
#include <U8g2lib.h>
#include <Adafruit_NeoPixel.h>

RoboRoverCore3      rover;
RoboRoverCore3Accel accel;
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C screen(U8G2_R0);
Adafruit_NeoPixel leds(RR_NEOPIXEL_COUNT, RR_PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// The box the ball rolls around in.
const int BOX_X = 8;
const int BOX_Y = 2;
const int BOX_W = 112;
const int BOX_H = 28;
const float BALL_R = 3.0;

// How the ball moves.
const float PUSH    = 0.00045;  // how strongly tilt pushes the ball
const float FRICTION = 0.88;    // how quickly the ball slows down
const float MAX_SPEED = 1.8;    // fastest the ball can roll
const float BOUNCE  = 0.45;     // how much speed is kept after hitting a wall

float ballX, ballY;             // position on the screen
float speedX, speedY;

void resetBall() {
  ballX = BOX_X + BOX_W / 2.0;
  ballY = BOX_Y + BOX_H / 2.0;
  speedX = 0;
  speedY = 0;
}

void moveBall() {
  accel.poll();

  // Accelerometer readings are in milli-g. Tipping the rover's right side
  // down makes Y positive, and lifting its nose makes X positive. Either way
  // the ball rolls towards the lower side.
  speedX += accel.y() * PUSH;
  speedY += accel.x() * PUSH;

  speedX = constrain(speedX * FRICTION, -MAX_SPEED, MAX_SPEED);
  speedY = constrain(speedY * FRICTION, -MAX_SPEED, MAX_SPEED);

  ballX += speedX;
  ballY += speedY;

  // Bounce off the walls of the box.
  float minX = BOX_X + BALL_R + 1;
  float maxX = BOX_X + BOX_W - BALL_R - 2;
  float minY = BOX_Y + BALL_R + 1;
  float maxY = BOX_Y + BOX_H - BALL_R - 2;

  if (ballX < minX) { ballX = minX; speedX = -speedX * BOUNCE; }
  if (ballX > maxX) { ballX = maxX; speedX = -speedX * BOUNCE; }
  if (ballY < minY) { ballY = minY; speedY = -speedY * BOUNCE; }
  if (ballY > maxY) { ballY = maxY; speedY = -speedY * BOUNCE; }
}

void drawBall() {
  screen.clearBuffer();
  screen.drawFrame(BOX_X, BOX_Y, BOX_W, BOX_H);

  // A small cross marks the centre: hold the rover level to park the ball on it.
  int cx = BOX_X + BOX_W / 2;
  int cy = BOX_Y + BOX_H / 2;
  screen.drawHLine(cx - 5, cy, 11);
  screen.drawVLine(cx, cy - 5, 11);

  screen.drawDisc((int)ballX, (int)ballY, (int)BALL_R);
  screen.sendBuffer();
}

void lightsFollowBall() {
  // Where the ball is, from -1 to +1 in each direction.
  float across = (ballX - (BOX_X + BOX_W / 2.0)) / (BOX_W / 2.0 - BALL_R - 1);
  float along  = (ballY - (BOX_Y + BOX_H / 2.0)) / (BOX_H / 2.0 - BALL_R - 1);
  across = constrain(across, -1.0, 1.0);
  along  = constrain(along, -1.0, 1.0);

  // Left and right picks the colour: red on the far left, through yellow,
  // green and blue, to purple on the far right. Stopping short of the full
  // rainbow keeps the two ends from both being red.
  uint16_t hue = (across + 1.0) / 2.0 * 50000;

  // Up and down sets the brightness: brightest at the top of the screen,
  // dimmest at the bottom, but never fully off.
  uint8_t brightness = 6 + (1.0 - along) / 2.0 * 120;

  leds.fill(leds.ColorHSV(hue, 255, brightness));
  leds.show();
}

void setup() {
  // Starts the connection to the rover's chips, and stops the wheels in case
  // the last sketch left them turning.
  rover.begin();
  accel.begin();

  screen.begin();
  screen.setBusClock(100000);   // match the speed of the rover's other chips

  leds.begin();
  leds.clear();
  leds.show();

  resetBall();
}

void loop() {
  moveBall();
  lightsFollowBall();
  drawBall();
  delay(20);
}
