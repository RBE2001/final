// Problems to figure out:
// --Where we might get false positives from line counters.
// --Where we end up after turning back onto center line.
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

const unsigned long turn90 = 1100;
const unsigned long turn180 = ((float)turn90 * 1.9);
const unsigned long turndelay = 300;

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
volatile int goal = 0;

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

  lf->set_pid(8e-3, 0.0, 5e-2);
}

// For serial debugging;
unsigned long next_update = 0;
void loop() {
  switch (state) {
    case kLine:
      updatelf = true;
      // Make decisions based on whether we have a rod.
      int dirdiff;
      Direction goaldir;
      switch (rodstate) {
        case kGetReactor:
          if (goal == -1) {
            goal = zerodone ? 1 : 0;
          }
          if (!digitalRead(vtrigger)) {
            state = kReactorPull;
            writeMotors(0, 0);
            updatelf = false;
            goal = -1; // Store will need to decide where to put this.
            break;
          }
          if ((dirstate == kUp && goal == 0) ||
              (dirstate == kDown && goal == 1)) {  // Need to turn around.
            Turn(turn180, true);
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
          if (goal == -1 || bt->storage(goal)) {
            if (!bt->storage(0)) goal = 0;
            if (!bt->storage(1)) goal = 1;
            if (!bt->storage(2)) goal = 2;
            if (!bt->storage(3)) goal = 3;
          }
          if (goal == -1) {
            // Something is wrong; No storage is available.
            updatelf = false;
            writeMotors(0, 0);
            break;
          }

          // Make sure that we are pointing the right direction.
          // Determine what direction we should be facing.
          if (locstate == kCenter) {
            if ((dirstate == kUp && nearline == (goal + 1)) ||
                (dirstate == kDown && nearline == goal)) {
              // XXX: Corner case: pointing towards reactor while at reactor,
              // before turning around.
              goaldir = kLeft;
              locstate = kStorageLines;
              nearline = goal;
            }
            else if (nearline > goal) goaldir = kDown;
            else goaldir = kUp;
          }
          else goaldir = kLeft;

          dirdiff = (int)goaldir - (int)dirstate; // Positive = left.
          if (dirdiff != 0) {
            int turntime = abs(dirdiff) == 2 ? turn180 : turn90;
            bool leftturn = (dirdiff > 0) ^ (abs(dirdiff) == 3);
            Turn(turntime, leftturn);
            state = kTurn;
            updatelf = false;
            dirstate = goaldir;
            break; // May as well quit this case statement.
          }

          // If we are on the lines and our trigger is hit, then we have arrived!
          if (locstate != kCenter && !digitalRead(vtrigger)) {
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
          // Determine what direction we should be facing.
          // We are either on the right line and need to follow it, on the
          // center and need to go up/down, or on the other side and need to get
          // to the center.
          if (locstate == kCenter) {
            if ((dirstate == kUp && nearline == (goal + 1)) ||
                (dirstate == kDown && nearline == goal)) {
              goaldir = kRight;
              locstate = kSupplyLines;
              nearline = goal;
            }
            else if (nearline > goal) goaldir = kDown;
            else goaldir = kUp;
          } else if (locstate == kSupplyLines)
            goaldir = kRight;
          else if (locstate == kStorageLines)
            goaldir = kRight;

          dirdiff = (int)goaldir - (int)dirstate; // Positive = left.
          if (dirdiff != 0) {
            int turntime = abs(dirdiff) == 2 ? turn180 : turn90;
            bool leftturn = (dirdiff > 0) ^ (abs(dirdiff) == 3);
            Turn(turntime, leftturn);
            state = kTurn;
            updatelf = false;
            dirstate = goaldir;
            break; // May as well quit this case statement.
          }

          // Stop and turn if we are on the wrong side and hit the center line.
          if (locstate == kStorageLines && rcounter->count()) {
            if (goal == nearline) {
              // No need to turn; just go straight.
              locstate = kSupplyLines;
            }
            else {
              locstate = kCenter;
              bool up = nearline < goal;

              // If we need to go up, then we turn left to go up.
              Turn(turn90, up);
              state = kTurn;
              updatelf = false;
              dirstate = up ? kUp : kDown;
            }
            break;
          }

          // If we are on the lines and our trigger is hit, then we have arrived!
          if (locstate != kCenter && !digitalRead(vtrigger)) {
            writeMotors(0, 0);
            updatelf = false;
            goal = -1;
            state = kSupplyPull;
            break;
          }
          break; // case kGetSupply
        case kSetReactor:
          if (!digitalRead(vtrigger)) {
            state = kReactorDrop;
            writeMotors(0, 0);
            updatelf = false;
            goal = -1; // Store will need to decide where to put this.
            zerodone = true;
            break;
          }

          if (goal == -1) {
            goal = zerodone ? 1 : 0;
          }

          // Make sure that we are pointing the right direction.
          // Determine what direction we should be facing.
          if (locstate == kCenter) {
            goaldir = goal ? kUp : kDown;
          }
          else if (locstate = kSupplyLines) goaldir = kLeft;
          else if (locstate = kStorageLines) goaldir = kRight;

          dirdiff = (int)goaldir - (int)dirstate; // Positive = left.
          if (dirdiff != 0) {
            int turntime = abs(dirdiff) == 2 ? turn180 : turn90;
            bool leftturn = (dirdiff > 0) ^ (abs(dirdiff) == 3);
            Turn(turntime, leftturn);
            state = kTurn;
            updatelf = false;
            dirstate = goaldir;
            break; // May as well quit this case statement.
          }

          // Stop and turn if we are on the sides. hit the center line.
          if (locstate != kCenter && rcounter->count()) {
            locstate = kCenter;
            bool up = goal;
            bool leftside = locstate == kStorageLines;
            bool leftturn;
            // 4 possibilities: coming from left->up, left->down, right->up,
            // right->down.
            if (up && leftside || !up && !leftside) {
              leftturn = true;
            }
            else leftturn = false;

            // If we need to go up, then we turn left to go up.
            Turn(turn90, leftturn);
            state = kTurn;
            updatelf = false;
            dirstate = up ? kUp : kDown;
          }



          break; // case kSetReactor
      }  // switch rodstate
      break; // case kLine
    case kTurn:
      if (TurnUpdate()) state = kLine;
      break; // case kTurn
    default:
      Serial.println("Backing Up.");
      writeMotors(-20, -20);
      //XXX: Remove this delay. We shouldn't have time delays in here.
      delay(700);
      writeMotors(0, 0);
      if (state == kReactorPull) {
        rodstate = kStore;
        bt->set_radlevel(Bluetooth::kSpent);
      }
      if (state == kReactorDrop) {
        rodstate = kGetReactor;
        bt->set_radlevel(Bluetooth::kNone);
      }
      if (state == kSupplyPull) {
        rodstate = kSetReactor;
        bt->set_radlevel(Bluetooth::kNew);
      }
      if (state == kStorageDrop) {
        rodstate = kGetSupply;
        bt->set_radlevel(Bluetooth::kNone);
      }
      state = kLine;
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
      rcounter->set_count(nearline);
      if (dirstate == kDown && nearline > 0) {
        rcounter->increment(false);
        rcounter->Update();
      }
      if (dirstate == kUp && nearline < 4) {
        rcounter->increment(true);
        rcounter->Update();
      }
      nearline = rcounter->count();
    }
    else {
      rcounter->reset();
      rcounter->increment(true);
      rcounter->Update();
    }
  }
  armpid->Update();

  if (millis() > next_update) {
    Serial.print("State:\t");
    Serial.print(state);
    Serial.print("\tRod:\t");
    Serial.print(rodstate);
    Serial.print("\tDir:\t");
    Serial.print(dirstate);
    Serial.print("\tLoc:\t");
    Serial.print(locstate);
    Serial.print("\tGoal:\t");
    Serial.print(goal);
    Serial.print("\tLine:\t");
    Serial.print(nearline);
    Serial.print("\tStorage:\t");
    Serial.print(bt->raw_storage());
    if (updatelf) Serial.print("\tLining");
    Serial.println();
    next_update = millis() + 250;
  }
}

void writeMotors(int left, int right) {
  left = linverted ? left : -left;
  right = rinverted ? right : -right;
  lmotor->write(90 - left);
  rmotor->write(90 - right);
}

unsigned long turn_end = 0;
unsigned long start_turn = 0;
bool turning_left = false;
void Turn(unsigned long time, bool left) {
  writeMotors(0, 0);
  turning_left = left;
  turn_end = millis() + time + 2 * turndelay /* Time it takes to stop */;
  start_turn = millis() + turndelay;
  Serial.println(turn_end - 400);
  Serial.println(millis());
}
bool TurnUpdate() {
  if (millis() > turn_end) {
    Serial.println("Ending Turn!");
    return true;
  }
  else if (millis() > turn_end - turndelay) {
    writeMotors(0, 0);
  }
  else if (millis() > start_turn) {
    if (turning_left) writeMotors(-20, 15);
    else writeMotors(20, -15);
  }
  return false;
}
