// TurnCalibration - make a 90 degree turn really 90 degrees on your rover.
// The wheels turn. Needs the IRremote library (for TinyIRReceiver).
//
// To turn on the spot the rover works out how far each wheel must roll from
// the track width, the distance between the wheels. Tyres and floors grip
// differently, so each rover is tuned once, on the floor it will drive on.
// The result is kept in the Arduino, and rover.begin() uses it from then on
// in every sketch.
//
// 1. Put the rover on the floor, lined up with something straight: the joint
//    between two tiles, or a strip of tape.
// 2. Press RIGHT on the remote. It makes four right turns of 90 degrees and
//    should end facing the way it started. LEFT does the same to the left.
// 3. Stopped short? Press UP once for each degree it was short.
//    Went too far? Press DOWN once for each degree too far.
//    Line it up again and repeat. If right and left disagree, settle between.
// 4. Press # to keep the result. * goes back to the default.
//
// Open the Serial Monitor at 115200 baud to see the track width.

#include <RoboRoverCore3.h>

// The IR receiver pin must be set before this include.
#define IR_RECEIVE_PIN RR_PIN_IR_RECV
#include <TinyIRReceiver.hpp>

RoboRoverCore3 rover;
RoboRoverCore3Buzzer buzzer;

// Remote buttons
const uint8_t UP    = 0x18;
const uint8_t DOWN  = 0x52;
const uint8_t LEFT  = 0x08;
const uint8_t RIGHT = 0x5A;
const uint8_t HASH  = 0x0D;
const uint8_t STAR  = 0x16;

void setup() {
  Serial.begin(115200);
  rover.begin();                  // also puts back a saved track width
  buzzer.begin();
  initPCIInterruptForTinyReceiver();
  showTrackWidth();
}

void loop() {
  // A held button repeats: act on the first press only.
  if (!TinyReceiverDecode() || (TinyIRReceiverData.Flags & IRDATA_FLAGS_IS_REPEAT)) {
    return;
  }
  uint8_t button = TinyIRReceiverData.Command;

  if (button == RIGHT || button == LEFT) {
    for (int turn = 0; turn < 4; turn++) {
      if (button == RIGHT) rover.turnRightAndWait(90);
      else                 rover.turnLeftAndWait(90);
      delay(300);
    }
    TinyReceiverDecode();         // forget anything pressed while it turned

  } else if (button == UP || button == DOWN) {
    // The turn grows with the track width, so one part in 360 is one degree
    // over the four turns.
    float track = rover.motion.trackWidthMm();
    if (button == UP) track += track / 360;
    else              track -= track / 360;
    rover.motion.setTrackWidthMm(track);
    buzzer.beep();
    showTrackWidth();

  } else if (button == HASH) {
    if (rover.saveTrackWidth()) {
      buzzer.beep(3000, 300);
      Serial.println("Saved. rover.begin() will use it in every sketch.");
    } else {
      Serial.println("Not saved.");
    }

  } else if (button == STAR) {
    rover.forgetTrackWidth();
    rover.motion.setTrackWidthMm(RR_M_DEF_TRACK_WIDTH / 100.0);
    buzzer.beep(1000, 300);
    showTrackWidth();
  }
}

void showTrackWidth() {
  Serial.print("Track width: ");
  Serial.print(rover.motion.trackWidthMm());
  Serial.println(" mm");
}
