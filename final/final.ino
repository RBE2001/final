// Currently contains code for testing bluetooth code on field-provided robot.
#include <Servo.h>
#include <QTRSensors.h>
#include <BluetoothClient.h>
#include <BluetoothMaster.h>
#include <ReactorProtocol.h>
#include "armpid.h"
#include "linefollower.h"
#include "linecounter.h"
#include "bluetooth.h"

const uint8_t lmotorport = 10;
const uint8_t rmotorport = 11;
const bool linverted = false;
const bool rinverted = true;
const uint8_t team = 8;

const uint8_t button = 3;
const uint8_t vtrigger = 22;
const unsigned char num_sensors = 8;
unsigned char linepins[num_sensors] = {46, 44, 42, 40, 38, 36, 34, 32};
const uint8_t lline = 0, rline = 1;
Bluetooth *bt;
LineFollower *lf;
Servo *lmotor, *rmotor; // Same as used in lf.
LineCounter *lcounter /*Unused*/, *rcounter;
ArmPID *armpid;
const uint8_t armmotor = 100, armpot = 100;

const unsigned long turn90 = 1000;

volatile bool go = false;
volatile uint8_t radlevel = 0;

void writeMotors(int left, int right); // Pass in -90 to 90.
void Turn(unsigned long time /*ms*/, bool left);
bool TurnUpdate(); // Returns whether done.

enum State {
  kLine,
  kTurn,
  kReactorPull,
  kReactorDrop,
  kSupplyPull,
  kStorageDrop,
} state;

enum RodAction {
  kGetReactor,
  kStore,
  kGetSupply,
  kSetReactor,
} rodstate;

enum Direction {
  // CCW order is important.
  kUp=0,
  kLeft=1,
  kDown=2,
  kRight=3,
} dirstate; // Direction robot is facing.

enum Location {
  kCenter, // Along center Up-Down line.
  kSupplyLines, // On right (supply) side of board.
  kStorageLines, // On left (storage) side of field.
} locstate;

// Number either of current storage/supply line or of nearest storage/supply
// line to the upwards direction (4 if past last line).
uint8_t nearline = 0;

// goal possibilities:
// Reactor: 0 for down, 1 for up.
// Supply/Storage: 0 - 3.
// -1 Means undecided.
uint8_t goal = 0;

// whether we have re-supplied reactor 0.
bool zerodone = false;

// Whether to line follow.
bool updatelf = true;

void setup() {
  Serial.begin(115200);
  Serial.println("Hello.");
  bt = new Bluetooth(team);
  lf = new LineFollower(linepins, num_sensors, lmotorport, rmotorport,
                        linverted, rinverted);
  lcounter = new LineCounter(lline); // Unused.
  rcounter = new LineCounter(rline);
  armpid = new ArmPID(armmotor, armpot, 0.0, 0.0, 0.0);
  lmotor = lf->left();
  rmotor = lf->right();
  state = kLine;
  // Pointing straight towards near (Down) reactor about to pull up tube.
  rodstate = kGetReactor;
  dirstate = kDown;
  locstate = kCenter;
  goal = 0;
  pinMode(vtrigger, INPUT_PULLUP);
  Serial.println("Done constructing.");

  pinMode(button, INPUT_PULLUP);
  while (digitalRead(button));
  Serial.println("Calibrating.");
  lf->Calibrate();
  Serial.println("Done Calibrating.");

  lf->set_pid(1e-2, 0.0, 0.0);
}

void loop() {
  switch (state) {
    case kLine:
      updatelf = true;
      // Make decisions based on whether we have a rod.
      switch (rodstate) {
        case kGetReactor:
          if (digitalRead(vtrigger)) {
            state = kReactorPull;
            writeMotors(0, 0);
            updatelf = false;
            goal = -1; // Store will need to decide where to put this.
            break;
          }
          if ((dirstate == kUp && goal == 0) ||
              (dirstate == kDown && goal == 1)) {  // Need to turn around.
            Turn(turn90 * 2, true);
            state = kTurn;
            updatelf = false;
            dirstate = (Direction) (2 - goal * 2);
            break;
          }
          else if (dirstate == kLeft || dirstate == kRight) {
            // Point up or down so that above if can deal with it.
            Turn(turn90, false); // So that dirstate-1 is guaranteed to work.
            state = kTurn;
            updatelf = false;
            dirstate = (Direction)((int)dirstate - 1);
            break;
          }
          break; // case kGetReactor
        case kStore:
          // If goal is undecided or no longer feasible.
          if (goal == -1 || !bt->storage(goal)) {
            if (bt->storage(0)) goal = 0;
            if (bt->storage(1)) goal = 1;
            if (bt->storage(2)) goal = 2;
            if (bt->storage(3)) goal = 3;
          }
          if (goal == -1) {
            // Something is wrong; No storage is available.
            updatelf = false;
            writeMotors(0, 0);
            break;
          }

          // Make sure that we are pointing the right direction.
          Direction goaldir;
          // Determine what direction we should be facing.
          if (locstate == kCenter) {
            if (nearline > goal) goaldir = kDown;
            else goaldir = kUp;
          }
          else goaldir = kLeft;

          int dirdiff = (int)goaldir - (int)dirstate; // Positive = left.
          if (dirdiff != 0) {
            Turn(abs(dirdiff) * turn90, (dirdiff > 0));
            state = kTurn;
            updatelf = false;
            dirstate = goaldir;
            break; // May as well quit this case statement.
          }

          // Check to see whether we have just noticed the line we want.
          if ((dirstate == kUp && nearline > goal) ||
              (dirstate == kDown && nearline == goal)) {
            // Turn left if we were going up; turn right if we were going down.
            Turn(turn90, dirstate == kUp);
            state = kTurn;
            updatelf = false;
            dirstate = kLeft;
            locstate = kStorageLines;
            break; // May as well quit to let turn execute.
          }

          // If we are on the lines and our trigger is hit, then we have arrived!
          // XXX: digitalRead(vtrigger)'s are backwards.
          if (locstate != kCenter && digitalRead(vtrigger)) {
            writeMotors(0, 0);
            updatelf = false;
            goal = -1;
            state = kStorageDrop;
            break;
          }

          // Otherwise, we just continue line following...
          break; // case kStore
        case kGetSupply:
          // Basically same logic as kStore, but with different variables.
          // Refactor to reuse code.
          // If goal is undecided or no longer feasible.
          if (goal == -1 || !bt->supply(goal)) {
            if (bt->supply(0)) goal = 0;
            if (bt->supply(1)) goal = 1;
            if (bt->supply(2)) goal = 2;
            if (bt->supply(3)) goal = 3;
          }
          if (goal == -1) {
            // Something is wrong; No storage is available.
            updatelf = false;
            writeMotors(0, 0);
            break;
          }

          // Make sure that we are pointing the right direction.
          Direction goaldir;
          // Determine what direction we should be facing.
          // We are either on the right line and need to follow it, on the
          // center and need to go up/down, or on the other side and need to get
          // to the center.
          if (locstate == kCenter) {
            if (nearline > goal) goaldir = kDown;
            else goaldir = kUp;
          } else if (locstate == kSupplyLines)
            goaldir = kRight;
          else if (locstate == kStorageLines)
            goaldir = kRight;

          int dirdiff = (int)goaldir - (int)dirstate; // Positive = left.
          if (dirdiff != 0) {
            Turn(abs(dirdiff) * turn90, (dirdiff > 0));
            state = kTurn;
            updatelf = false;
            dirstate = goaldir;
            break; // May as well quit this case statement.
          }

          // Check to see whether we have just noticed the line we want.
          if ((dirstate == kUp && nearline > goal) ||
              (dirstate == kDown && nearline == goal)) {
            // Turn right if we were going up; turn left if we were going down.
            Turn(turn90, dirstate == kDown);
            state = kTurn;
            updatelf = false;
            dirstate = kRight;
            locstate = kSupplyLines;
            break; // May as well quit to let turn execute.
          }

          // Stop and turn if we are on the wrong side and hit the center line.
          if (locstate == kStorageLines && rcounter->counts()) {
            if (goal == nearline) {
              // No need to turn; just go straight.
              locstate = kSupplyLines;
            }
            bool up = nearline < goal;

            // If we need to go up, then we turn left to go up.
            Turn(turn90, up);
            state = kTurn;
            updatelf = false;
            dirstate = up ? kUp : kDown;
            break;
          }

          break; // case kSetReactor
      }  // switch rodstate
      break; // case kLine
    case kTurn:
      if (TurnUpdate()) state = kLine;
      break; // case kTurn
    default:
      writeMotors(-20, -20);
      //XXX: Remove this delay. We shouldn't have time delays in here.
      delay(200);
      writeMotors(0, 0);
      state = kLine;
      if (state == kReactorPull) rodstate = kStore;
      if (state == kReactorDrop) rodstate = kGetReactor;
      if (state == kSupplyPull) rodstate = kSetReactor;
      if (state == kStorageDrop) rodstate = kGetSupply;
      break;
  } // switch state.

  // Update appropriate loops.
  bt->Update();
  if (bt->stopped()) {
    updatelf == false;
    if (state != kTurn) writeMotors(0, 0);
  }
  if (updatelf) lf->Update();
  // We only want to update the counter when we are pure line following.
  // When following the center line, we want to:
  // -Decrement if dirstate == kDown and nearline > 0.
  // -Increment if dirstate == kUp and nearline < 4.
  // When on the side lines, we want to set to zero and let it update.
  if (state == kLine) {
    if (locstate == kCenter) {
      if (dirstate == kDown && nearline > 0) {
        rcounter->increment(false);
        rcounter->Update();
      }
      if (dirstate == kUp && nearline < 4) {
        rcounter->increment(true);
        rcounter->Update();
      }
    }
    else {
      rcounter->reset();
      rcounter->increment(true);
      rcounter->Update();
    }
  }
  armpid->Update();
}

void writeMotors(int left, int right) {
  left = linverted ? left : -left;
  right = rinverted ? right : -right;
  lmotor->write(90 + left);
  rmotor->write(90 + right);
}

unsigned long turn_end = 0;
void Turn(unsigned long time, bool left) {
  if (left) writeMotors(-20, 20);
  else writeMotors(20, -20);
  turn_end = millis() + time;
  Serial.println(turn_end);
  Serial.println(millis());
}
bool TurnUpdate() {
  if (millis() > turn_end) {
    Serial.println("Ending Turn!");
    writeMotors(0, 0);
    return true;
  }
  else return false;
}
