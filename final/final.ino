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
ArmPID *arm;
const uint8_t armmotor = 9, armpot = 2;

const unsigned long turn90 = 1100;
const unsigned long turn180 = ((float)turn90 * 1.9);
const unsigned long turndelay = 100;

const int closegrip = 190;
const int opengrip = 90;
const uint8_t gripservo = 7;
Servo gripper;

const int flatwrist = 150;
const int vertwrist = 65;
const uint8_t wristservo = 8;
Servo wrist;

const int armdown = 160;
const int armup = 275;

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

enum PlaceAction {
  kStartArm, // Move arm into place.
  kManipulate, // Grab/release rod.
  kRemoveArm, // Lift arm (or equivalent).
  kBackup,
} placestate;
unsigned long place_action_end = 0;

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
  arm = new ArmPID(armmotor, armpot, 0.25, 0.0015, 5.0);
  lmotor = lf->left();
  rmotor = lf->right();
  wrist.attach(wristservo);
  gripper.attach(gripservo, 500, 2600 /*Need full reach of servo*/);
  state = kLine;
  // Pointing straight towards near (Down) reactor about to pull up tube.
  rodstate = kGetReactor;
  dirstate = kDown;
  locstate = kCenter;
  goal = 0;
  pinMode(vtrigger, INPUT_PULLUP);
  Serial.println("Done constructing.");

  // Set up servos as open, ready to grip from reactor.
  wrist.write(vertwrist);
  gripper.write(opengrip);
  arm->set_setpoint(armup);

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
      bool just_starting;
      switch (rodstate) {
        case kGetReactor:
          wrist.write(vertwrist);
          gripper.write(opengrip);
          arm->set_setpoint(armup);

          if (goal == -1) {
            goal = zerodone ? 1 : 0;
          }
          if (!digitalRead(vtrigger)) {
            state = kReactorPull;
            placestate = kStartArm;
            writeMotors(0, 0);
            arm->set_setpoint(armdown);
            wrist.write(vertwrist);
            gripper.write(opengrip);
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
          arm->set_setpoint(armup);
          wrist.write(flatwrist);
          gripper.write(closegrip);
          // If goal is undecided or no longer feasible.
          if (goal == -1 || bt->storage(goal)) {
            just_starting = true;
            if (!bt->storage(0)) goal = 0;
            else if (!bt->storage(1)) goal = 1;
            else if (!bt->storage(2)) goal = 2;
            else if (!bt->storage(3)) goal = 3;
          }
          else just_starting = false;
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
                (dirstate == kDown && nearline == goal) && !just_starting) {
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
            placestate = kStartArm;
            break;
          }

          // Otherwise, we just continue line following...
          break; // case kStore
        case kGetSupply:
          arm->set_setpoint(armup);
          wrist.write(flatwrist);
          gripper.write(opengrip);
          // Basically same logic as kStore, but with different variables.
          // Refactor to reuse code.
          // If goal is undecided or no longer feasible.
          if (goal == -1 || !bt->supply(goal)) {
            if (bt->supply(0)) goal = 0;
            else if (bt->supply(1)) goal = 1;
            else if (bt->supply(2)) goal = 2;
            else if (bt->supply(3)) goal = 3;
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
            placestate = kStartArm;
            break;
          }
          break; // case kGetSupply
        case kSetReactor:
          arm->set_setpoint(armup);
          wrist.write(vertwrist);
          gripper.write(closegrip);
          if (!digitalRead(vtrigger)) {
            state = kReactorDrop;
            placestate = kStartArm;
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

    // These cases will use goal to indicate whether a particular stage of the
    // action has started/finished.
    case kReactorPull:
      // Position arm, grab, pull up, back up.
      switch (placestate) {
        case kStartArm:
          if (goal == -1) {
            place_action_end = millis() + 5000;
            goal = 0;
            arm->set_setpoint(armdown);
            wrist.write(vertwrist);
            gripper.write(opengrip);
          }
          else if (millis() > place_action_end) {
            placestate = kManipulate;
            goal = -1;
          }
          break; // kStartArm
        case kManipulate:
          if (goal == -1) {
            place_action_end = millis() + 1500;
            goal = 0;
            gripper.write(closegrip);
          }
          else if (millis() > place_action_end) {
            placestate = kRemoveArm;
            goal = -1;
          }
          break; // kManipulate
        case kRemoveArm:
          if (goal == -1) {
            place_action_end = millis() + 3000;
            goal = 0;
            arm->set_setpoint(armup);
            bt->set_radlevel(Bluetooth::kSpent);
          }
          else if (millis() > place_action_end) {
            placestate = kBackup;
            goal = -1;
          }
          break; // kRemoveArm
        case kBackup:
          if (goal == -1) {
            place_action_end = millis() + 700;
            goal = 0;
            writeMotors(-20, -20);
          }
          else if (millis() > place_action_end) {
            writeMotors(0, 0);
            placestate = kStartArm;
            rodstate = kStore;
            state = kLine;
            goal = -1;
          }
          break; // kBackup
      }
      break; // case kReactorPull
    case kReactorDrop:
      // Position arm, grab, pull up, back up.
      switch (placestate) {
        case kStartArm:
          if (goal == -1) {
            place_action_end = millis() + 3000;
            goal = 0;
            arm->set_setpoint(armdown);
            wrist.write(vertwrist);
            gripper.write(closegrip);
          }
          else if (millis() > place_action_end) {
            placestate = kManipulate;
            goal = -1;
          }
          break; // kStartArm
        case kManipulate:
          if (goal == -1) {
            place_action_end = millis() + 500;
            goal = 0;
            gripper.write(opengrip);
          }
          else if (millis() > place_action_end) {
            bt->set_radlevel(Bluetooth::kNone);
            placestate = kRemoveArm;
            goal = -1;
          }
          break; // kManipulate
        case kRemoveArm:
          if (goal == -1) {
            place_action_end = millis() + 3000;
            goal = 0;
            arm->set_setpoint(armup);
          }
          else if (millis() > place_action_end) {
            placestate = kBackup;
            goal = -1;
          }
          break; // kRemoveArm
        case kBackup:
          if (goal == -1) {
            place_action_end = millis() + 700;
            goal = 0;
            writeMotors(-20, -20);
          }
          else if (millis() > place_action_end) {
            writeMotors(0, 0);
            placestate = kStartArm;
            rodstate = kGetReactor;
            state = kLine;
            goal = -1;
          }
          break; // kBackup
      }
      break; // case kReactorDrop
    case kSupplyPull:
      // Position arm, grab, pull up, back up.
      switch (placestate) {
        case kStartArm:
          if (goal == -1) {
            place_action_end = millis() + 3000;
            goal = 0;
            arm->set_setpoint(armup);
            wrist.write(flatwrist);
            gripper.write(opengrip);
          }
          else if (millis() > place_action_end) {
            placestate = kManipulate;
            goal = -1;
          }
          break; // kStartArm
        case kManipulate:
          if (goal == -1) {
            place_action_end = millis() + 500;
            goal = 0;
            gripper.write(closegrip);
          }
          else if (millis() > place_action_end) {
            bt->set_radlevel(Bluetooth::kNew);
            placestate = kRemoveArm;
            goal = -1;
          }
          break; // kManipulate
        case kRemoveArm:
          // Fall through.
          placestate = kBackup;
          goal = -1;
          break; // kRemoveArm
        case kBackup:
          if (goal == -1) {
            place_action_end = millis() + 700;
            goal = 0;
            writeMotors(-20, -20);
          }
          else if (millis() > place_action_end) {
            writeMotors(0, 0);
            placestate = kStartArm;
            rodstate = kSetReactor;
            state = kLine;
            goal = -1;
          }
          break; // kBackup
      }
      break; // case kSupplyPull
    case kStorageDrop:
      // Position arm, grab, pull up, back up.
      switch (placestate) {
        case kStartArm:
          if (goal == -1) {
            place_action_end = millis() + 3000;
            goal = 0;
            arm->set_setpoint(armup);
            wrist.write(flatwrist);
            gripper.write(closegrip);
          }
          else if (millis() > place_action_end) {
            placestate = kManipulate;
            goal = -1;
          }
          break; // kStartArm
        case kManipulate:
          if (goal == -1) {
            place_action_end = millis() + 500;
            goal = 0;
            gripper.write(opengrip);
          }
          else if (millis() > place_action_end) {
            bt->set_radlevel(Bluetooth::kNone);
            placestate = kRemoveArm;
            goal = -1;
          }
          break; // kManipulate
        case kRemoveArm:
          // Fall through.
          placestate = kBackup;
          goal = -1;
          break; // kRemoveArm
        case kBackup:
          if (goal == -1) {
            place_action_end = millis() + 700;
            goal = 0;
            writeMotors(-20, -20);
          }
          else if (millis() > place_action_end) {
            writeMotors(0, 0);
            placestate = kStartArm;
            rodstate = kGetSupply;
            state = kLine;
            goal = -1;
          }
          break; // kBackup
      }
      break; // case kStorageDrop
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
  arm->Update();

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
  writeMotors(-20, -20);
  turning_left = left;
  turn_end = millis() + time - 400 + 2 * turndelay /* Time it takes to stop */;
  start_turn = millis() + turndelay;
  Serial.println(turn_end - 400);
  Serial.println(millis());
}
bool TurnUpdate() {
  int linepos = lf->Read();
  /*
  if (millis() > turn_end) {
    Serial.println("Ending Turn!");
    return true;
  }
  else */ if ((millis() > turn_end - turndelay) && (linepos > 2500 && linepos < 4500)) {
    writeMotors(0, 0);
    return true;
  }
  else if (millis() > start_turn) {
    if (turning_left) writeMotors(-20, 15);
    else writeMotors(20, -15);
  }
  return false;
}
