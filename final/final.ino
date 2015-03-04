/**
 * This is the final project code for the RBE2001 project by Team 8, consisting
 * of James Kuszmaul, Gabrielle O'Dell, and Ryan Wiesenberg.
 * A video of the final robot, running a slightly less commented version of this
 * code, can be found on YouTube at:
 * https://www.youtube.com/watch?v=_5CLEVsUC38
 * The final project was due on Wednesday March 4th, 2015 at the end of C-Term
 * 2015 at Worcester Polytechnic Institute (WPI).
 */
#include <Servo.h>
#include <QTRSensors.h>
#include <BluetoothClient.h>
#include <BluetoothMaster.h>
#include <ReactorProtocol.h>
#include <LiquidCrystal.h>
#include "armpid.h"
#include "linefollower.h"
#include "linecounter.h"
#include "bluetooth.h"

// This file is organized as follows:
// 1) Various variables, constants, etc.
// 2) arduino setup() function.
// 3) main loop() function.
// 4) Helper functions defined.

// This first section of constants is largely port numbers and declaring objects
// that we will be using.

// Information for motors.
// Digital Output ports of motors.
const uint8_t lmotorport = 10;
const uint8_t rmotorport = 11;
// Whether each motor is mounted such that we need to send inverted values to
// the motor controller.
const bool linverted = false;
const bool rinverted = true;

// Team number (for Bluetooth).
const uint8_t team = 8;

// Port of button for starting the robot.
const uint8_t button = 3;

// Port of trigger at front of robot for detecting hitting a post.
const uint8_t vtrigger = 22;

// Number of sensor in line sensor array and their respective DIO ports.
const unsigned char num_sensors = 8;
unsigned char linepins[num_sensors] = {52, 53, 51, 49, 47, 27, 25, 23};

// Ports of left and right Vex line sensors (used for counting lines).
// Note: Left sensor not actually used in final logic.
const uint8_t lline = 3, rline = 1;

// Bluetooth object for handling appropriate bluetooth interactions.
Bluetooth *bt;

// LineFollower object for handling PD loop for line following.
LineFollower *lf;
// left and right motor objects; note that these will be initialized and
// attached to the appropriate ports inside the lf constructor and the
// LineFollower should be disabled before writing values to the lmotor and
// rmotor objects.
Servo *lmotor, *rmotor;  // Same as used in lf.

// Objects for counting the lines as we pass them by. Only rcounter is used.
LineCounter *lcounter /*Unused*/, *rcounter;

// Controls arm using PID.
ArmPID *arm;
// Ports for arm motor, Potentiometer, and bottom limit button, respectively.
const uint8_t armmotor = 9, armpot = 2, armbutton = 24;

// Ports of the line sensor towards the back of the robot, used to detect when
// we have finished our turns.
const uint8_t backline = 0;

// This section of comments is largely comprised of tuned values; ie, turnign
// times, servo positions, etc.

// All times in milliseconds.

// turn90 and turn180 are the minimum amount of time that should be spent
// turning 90 and 180 degrees--avoids early exits from turns.
const unsigned long turn90 = 600;
const unsigned long turn180 = 1100;
const unsigned long turndelay = 30;  // Amount to backup before turning.

// Amount to backup after having gone to the reactor or storaage/supply tubes.
const unsigned long reactorbackup = 700;
const unsigned long tubebackup = 750;

// Max amount of time that it should generally take the arm to get from one
// position to another.
const unsigned long armreaction = 1500;

const unsigned long supplybackup =
    150;  // Time to backup before grabbing supply tube.

// Amount to wait after turning onto the supply lines before allowing the line
// counter to detect the line that is immediately in front of the supply tubes.
const unsigned long supplyturn = 2500;

// Servo values for gripper.
const int closegrip = 180;
const int slightgrip = closegrip - 30;  // Largely unused.
const int opengrip = 90;
// DIO port of gripper.
const uint8_t gripservo = 7;
// Actual gripper servo object.
Servo gripper;

// Servo values for wrist (thing that turns the gripper).
const int flatwrist = 103;
// Tilts wrist slightly upwards when going to supply tubes so that we don't ram
// the gripper into the supply tubes.
const int tiltedwrist = flatwrist + 10;
const int vertwrist = 13;
// Upsidedown name is misleading; merely a position used for when dropping the
// rod into the reactor.
const int upsidedownwrist = vertwrist + 3;
// DIO port of wrist.
const uint8_t wristservo = 8;
// Actual wrist Servo object.
Servo wrist;

// Potentiometer positions corresponding to down and up positions of the arm.
const int armdown = 140;
const int armup = 255;

// Function prototypes:

// Used to write raw values to the motors--positive is forwards.
// Be sure to disable line tracking before calling writeMotors.
void writeMotors(int left, int right);  // Pass in -90 to 90.

// Turn functions
// Turn is called to initiate a turn.
// Takes a time in milliseconds which is the minimum time that will be spent
// turning and takes a boolean indicating which direction to go.
void Turn(unsigned long time /*ms*/, bool left);

// Updates the motors for the turn function; if the conditions for ending the
// turn are met, then returns true and stops the motors.
bool TurnUpdate();  // Returns whether done.

// The remaining variables that are defined are all state variables used to keep
// track of the current position, goal, etc. of the robot.

// The current high level state of the robot.
enum State {
  kLine,         // We are following a line somewhere.
  kTurn,         // Currently in the process of turning.
  kReactorPull,  // In the act of pulling a tube from the reactor.
  kReactorDrop,  // In the act of dropping a tube back into the reactor.
  kSupplyPull,   // Pulling a tube from the supply tubes.
  kStorageDrop,  // Placing a spent rod into the storage tubes.
} state;

// What we are currently aiming to do with the rod.
enum RodAction {
  kGetReactor,  // Need to retrieve a rod from the reactor.
  kStore,       // Need to store a spent rod which we are holding.
  kGetSupply,   // Need to get a rod from the supply.
  kSetReactor,  // Need to place a rod back into the reactor.
} rodstate;

// If we are in the process of manipulating a tube, indicates our current state.
enum PlaceAction {
  kStartArm,    // Move arm into place.
  kManipulate,  // Grab/release rod.
  kRemoveArm,   // Lift arm (or equivalent).
  kBackup,      // Backup from tube.
} placestate;
// Time, in milliseconds, at which to end the current PlaceAction. Also borrowed
// for use as a general-purpose end action time in a few other spots.
unsigned long place_action_end = 0;

enum Direction {
  // CCW order is important.
  kUp = 0,
  kLeft = 1,
  kDown = 2,
  kRight = 3,
} dirstate;  // Direction robot is facing.

enum Location {
  kCenter,        // Along center Up-Down line.
  kSupplyLines,   // On right (supply) side of board.
  kStorageLines,  // On left (storage) side of field.
} locstate;

// Number either of current storage/supply line or of nearest storage/supply
// line to the upwards direction (4 if past last line).
uint8_t nearline = 0;

// goal possibilities:
// Reactor: 0 for down, 1 for up.
// Supply/Storage: 0 - 3.
// -1 Means undecided.
volatile int goal = -1;

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
  lcounter = new LineCounter(lline);  // Unused.
  rcounter = new LineCounter(rline);
  arm = new ArmPID(armmotor, armpot, armbutton, 0.25, 0.0018, 2.0);
  lmotor = lf->left();
  rmotor = lf->right();
  wrist.attach(wristservo);
  gripper.attach(gripservo, 500, 2600 /*Need full reach of servo*/);
  state = kLine;
  // Pointing straight towards near (Down) reactor about to pull up tube.
  rodstate = kGetReactor;
  dirstate = kDown;
  nearline = 0;
  locstate = kCenter;
  goal = -1;
  pinMode(vtrigger, INPUT_PULLUP);
  Serial.println("Done constructing.");

  // Set up servos as open, ready to grip from reactor.
  wrist.write(vertwrist);
  gripper.write(opengrip);
  arm->set_setpoint(armup);

  pinMode(button, INPUT_PULLUP);
  while (digitalRead(button))
    ;
  Serial.println("Calibrating.");
  lf->Calibrate();
  Serial.println("Done Calibrating.");
  // Delay until we want to start.
  while (digitalRead(button)) bt->Update();
  // Delay so that we have time to press button.
  for (int i = 0; i < 5; i++) {
    delay(200);
    bt->Update();
  }

  lf->set_pid(9e-3, 0.0, 1.1e-1);
  lf->set_back_pid(5e-3, 0.0, 0.1);
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
      bool just_starting, onsupplyline;
      switch (rodstate) {
        case kGetReactor:
          wrist.write(vertwrist);
          gripper.write(opengrip);
          arm->set_setpoint(armup);

          if (goal == -1) {
            Serial.print("Setting goal to: ");
            goal = zerodone ? 1 : 0;
            Serial.println(goal);
          }
          if (!digitalRead(vtrigger)) {
            state = kReactorPull;
            placestate = kStartArm;
            writeMotors(0, 0);
            arm->set_setpoint(armdown);
            wrist.write(vertwrist);
            gripper.write(opengrip);
            updatelf = false;
            goal = -1;  // Store will need to decide where to put this.
            break;
          }
          if ((dirstate == kUp && goal == 0) ||
              (dirstate == kDown && goal == 1)) {  // Need to turn around.
            Turn(turn180, true);
            state = kTurn;
            updatelf = false;
            dirstate = (Direction)(2 - goal * 2);
            break;
          } else if (dirstate == kLeft || dirstate == kRight) {
            // Point up or down so that above if can deal with it.
            Turn(turn90, false);  // So that dirstate-1 is guaranteed to work.
            state = kTurn;
            updatelf = false;
            dirstate = (Direction)((int)dirstate - 1);
            break;
          }
          break;  // case kGetReactor
        case kStore:
          arm->set_setpoint(armup);
          wrist.write(flatwrist);
          gripper.write(closegrip);
          // If goal is undecided or no longer feasible.
          if (goal == -1 && (bt->storage(goal) != 0x0F)) {
            just_starting = true;
            if (!bt->storage(0))
              goal = 0;
            else if (!bt->storage(1))
              goal = 1;
            else if (!bt->storage(2))
              goal = 2;
            else if (!bt->storage(3))
              goal = 3;
          } else
            just_starting = false;
          if (goal == -1) {
            // Something is wrong; No storage is available.
            updatelf = false;
            writeMotors(0, 0);
            break;
          }

          // Make sure that we are pointing the right direction.
          // Determine what direction we should be facing.
          if (locstate == kCenter) {
            if (((dirstate == kUp && nearline == (goal + 1)) ||
                 (dirstate == kDown && nearline == goal)) &&
                !just_starting) {
              goaldir = kLeft;
              locstate = kStorageLines;
              nearline = goal;
            } else if (nearline > goal)
              goaldir = kDown;
            else
              goaldir = kUp;
          } else
            goaldir = kLeft;

          dirdiff = (int)goaldir - (int)dirstate;  // Positive = left.
          if (dirdiff != 0) {
            int turntime = abs(dirdiff) == 2 ? turn180 : turn90;
            bool leftturn = (dirdiff > 0) != (abs(dirdiff) == 3);
            Turn(turntime, leftturn);
            state = kTurn;
            updatelf = false;
            dirstate = goaldir;
            break;  // May as well quit this case statement.
          }

          // If we are on the lines and our trigger is hit, then we have
          // arrived!
          if (locstate != kCenter && !digitalRead(vtrigger)) {
            writeMotors(0, 0);
            updatelf = false;
            goal = -1;
            state = kStorageDrop;
            placestate = kStartArm;
            break;
          }

          // Otherwise, we just continue line following...
          break;  // case kStore
        case kGetSupply:
          arm->set_setpoint(armup);
          wrist.write(tiltedwrist);
          gripper.write(opengrip);
          // Basically same logic as kStore, but with different variables.
          // Refactor to reuse code.
          // If goal is undecided or no longer feasible and it is not -2 (which
          // means we are just about finished).
          if (goal != -2 && (goal == -1 || !bt->supply(goal))) {
            if (bt->supply(0))
              goal = 0;
            else if (bt->supply(1))
              goal = 1;
            else if (bt->supply(2))
              goal = 2;
            else if (bt->supply(3))
              goal = 3;
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
            } else if (nearline > goal)
              goaldir = kDown;
            else
              goaldir = kUp;
          } else if (locstate == kSupplyLines)
            goaldir = kRight;
          else if (locstate == kStorageLines)
            goaldir = kRight;

          dirdiff = (int)goaldir - (int)dirstate;  // Positive = left.
          if (dirdiff != 0) {
            int turntime = abs(dirdiff) == 2 ? turn180 : turn90;
            bool leftturn = (dirdiff > 0) ^ (abs(dirdiff) == 3);
            Turn(turntime, leftturn);
            state = kTurn;
            updatelf = false;
            dirstate = goaldir;
            place_action_end = millis() + supplyturn;
            break;  // May as well quit this case statement.
          }

          // Stop and turn if we are on the wrong side and hit the center line.
          if (locstate == kStorageLines && rcounter->count()) {
            if (goal == nearline) {
              // No need to turn; just go straight.
              locstate = kSupplyLines;
            } else {
              locstate = kCenter;
              bool up = nearline < goal;

              // If we need to go up, then we turn left to go up.
              Turn(turn90, up);
              state = kTurn;
              updatelf = false;
              dirstate = up ? kUp : kDown;
              // Update nearline correctly.
              if (dirstate == kUp) nearline += 1;
            }
            break;
          }

          onsupplyline =
              locstate != kCenter && goal != -2 && place_action_end < millis();
          if ((rcounter->count() && onsupplyline) || goal == -3) {
            goal = -3;
            // Slow down and go forwards.
            updatelf = false;
            writeMotors(0, 0);
          }

          // If we are on the lines and our trigger is hit, then we have
          // arrived!
          if (!digitalRead(vtrigger) && onsupplyline) {
            updatelf = false;
            goal = -2;  // backup a short bit before grabbing the rod..
            gripper.write(slightgrip);
            place_action_end = millis() + supplybackup;
            writeMotors(-11, -11);
            break;
          } else if (goal == -2 && millis() > place_action_end) {
            writeMotors(0, 0);
            updatelf = false;
            goal = -1;
            state = kSupplyPull;
            placestate = kStartArm;
            break;
          } else if (goal == -2) {
            updatelf = false;
            writeMotors(-12, -12);
          }
          break;  // case kGetSupply
        case kSetReactor:
          arm->set_setpoint(armup);
          wrist.write(upsidedownwrist);
          gripper.write(closegrip);
          if (!digitalRead(vtrigger)) {
            state = kReactorDrop;
            placestate = kStartArm;
            writeMotors(0, 0);
            updatelf = false;
            goal = -1;  // Store will need to decide where to put this.
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
          } else if (locstate = kSupplyLines)
            goaldir = kLeft;
          else if (locstate = kStorageLines)
            goaldir = kRight;

          dirdiff = (int)goaldir - (int)dirstate;  // Positive = left.
          if (dirdiff != 0) {
            int turntime = abs(dirdiff) == 2 ? turn180 : turn90;
            bool leftturn = (dirdiff > 0) ^ (abs(dirdiff) == 3);
            Turn(turntime, leftturn);
            state = kTurn;
            updatelf = false;
            dirstate = goaldir;
            break;  // May as well quit this case statement.
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
            } else
              leftturn = false;

            // If we need to go up, then we turn left to go up.
            Turn(turn90, leftturn);
            state = kTurn;
            updatelf = false;
            dirstate = up ? kUp : kDown;
          }

          break;  // case kSetReactor
      }           // switch rodstate
      break;      // case kLine
    case kTurn:
      if (TurnUpdate()) state = kLine;
      break;  // case kTurn

    // These cases will use goal to indicate whether a particular stage of the
    // action has started/finished.
    case kReactorPull:
      // Position arm, grab, pull up, back up.
      switch (placestate) {
        case kStartArm:
          if (goal == -1) {
            updatelf = false;
            place_action_end = millis() + 2000;
            goal = 0;
            arm->set_setpoint(armdown);
            wrist.write(vertwrist);
            gripper.write(opengrip);
          } else if (millis() > place_action_end) {
            placestate = kManipulate;
            goal = -1;
          }
          break;  // kStartArm
        case kManipulate:
          if (goal == -1) {
            place_action_end = millis() + 1000;
            goal = 0;
            gripper.write(closegrip);
          } else if (millis() > place_action_end) {
            placestate = kRemoveArm;
            goal = -1;
          }
          break;  // kManipulate
        case kRemoveArm:
          if (goal == -1) {
            place_action_end = millis() + armreaction;
            goal = 0;
            arm->set_setpoint(armup);
            bt->set_radlevel(Bluetooth::kSpent);
          } else if (millis() > place_action_end) {
            placestate = kBackup;
            goal = -1;
          }
          break;  // kRemoveArm
        case kBackup:
          updatelf = false;
          if (goal == -1) {
            place_action_end = millis() + reactorbackup;
            goal = 0;
            writeMotors(-20, -20);
          } else if (millis() > place_action_end) {
            writeMotors(0, 0);
            placestate = kStartArm;
            rodstate = kStore;
            state = kLine;
            goal = -1;
          }
          break;  // kBackup
      }
      break;  // case kReactorPull
    case kReactorDrop:
      // Position arm, grab, pull up, back up.
      switch (placestate) {
        case kStartArm:
          if (goal == -1) {
            place_action_end = millis() + armreaction;
            goal = 0;
            arm->set_setpoint(armdown);
            wrist.write(upsidedownwrist);
            gripper.write(closegrip);
          } else if (millis() > place_action_end) {
            placestate = kManipulate;
            goal = -1;
          }
          break;  // kStartArm
        case kManipulate:
          if (goal == -1) {
            place_action_end = millis() + 500;
            goal = 0;
            gripper.write(opengrip);
          } else if (millis() > place_action_end) {
            bt->set_radlevel(Bluetooth::kNone);
            placestate = kRemoveArm;
            goal = -1;
          }
          break;  // kManipulate
        case kRemoveArm:
          if (goal == -1) {
            place_action_end = millis() + armreaction;
            goal = 0;
            arm->set_setpoint(armup);
          } else if (millis() > place_action_end) {
            placestate = kBackup;
            goal = -1;
          }
          break;  // kRemoveArm
        case kBackup:
          updatelf = false;
          if (goal == -1) {
            place_action_end = millis() + reactorbackup;
            goal = 0;
            writeMotors(-20, -20);
          } else if (millis() > place_action_end) {
            writeMotors(0, 0);
            placestate = kStartArm;
            rodstate = kGetReactor;
            state = kLine;
            goal = -1;
          }
          break;  // kBackup
      }
      break;  // case kReactorDrop
    case kSupplyPull:
      // Position arm, grab, pull up, back up.
      switch (placestate) {
        case kStartArm:
          if (goal == -1) {
            place_action_end = millis() + armreaction;
            goal = 0;
            arm->set_setpoint(armup);
            wrist.write(flatwrist);
            gripper.write(opengrip);
          } else if (millis() > place_action_end) {
            placestate = kManipulate;
            goal = -1;
          }
          break;  // kStartArm
        case kManipulate:
          if (goal == -1) {
            place_action_end = millis() + 500;
            goal = 0;
            gripper.write(closegrip);
          } else if (millis() > place_action_end) {
            bt->set_radlevel(Bluetooth::kNew);
            placestate = kRemoveArm;
            goal = -1;
          }
          break;  // kManipulate
        case kRemoveArm:
          // Fall through.
          placestate = kBackup;
          goal = -1;
          break;  // kRemoveArm
        case kBackup:
          updatelf = false;
          if (goal == -1) {
            place_action_end = millis() + tubebackup;
            goal = 0;
            writeMotors(-20, -20);
          } else if (millis() > place_action_end) {
            writeMotors(0, 0);
            placestate = kStartArm;
            rodstate = kSetReactor;
            state = kLine;
            goal = -1;
          }
          break;  // kBackup
      }
      break;  // case kSupplyPull
    case kStorageDrop:
      // Position arm, grab, pull up, back up.
      switch (placestate) {
        case kStartArm:
          if (goal == -1) {
            place_action_end = millis() + armreaction;
            goal = 0;
            arm->set_setpoint(armup);
            wrist.write(flatwrist);
            gripper.write(closegrip);
          } else if (millis() > place_action_end) {
            placestate = kManipulate;
            goal = -1;
          }
          break;  // kStartArm
        case kManipulate:
          if (goal == -1) {
            place_action_end = millis() + 500;
            goal = 0;
            gripper.write(opengrip);
          } else if (millis() > place_action_end) {
            bt->set_radlevel(Bluetooth::kNone);
            placestate = kRemoveArm;
            goal = -1;
          }
          break;  // kManipulate
        case kRemoveArm:
          // Fall through.
          placestate = kBackup;
          goal = -1;
          break;  // kRemoveArm
        case kBackup:
          updatelf = false;
          if (goal == -1) {
            place_action_end = millis() + tubebackup;
            goal = 0;
            writeMotors(-20, -20);
          } else if (millis() > place_action_end) {
            writeMotors(0, 0);
            placestate = kStartArm;
            rodstate = kGetSupply;
            state = kLine;
            goal = -1;
          }
          break;  // kBackup
      }
      break;  // case kStorageDrop
  }           // switch state.

  // Update appropriate loops.
  bt->Update();
  if (bt->stopped()) {
    updatelf = false;
    if (state != kTurn) writeMotors(0, 0);
  }

  lf->enable_outputs(updatelf);
  lf->Update();
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
    } else {
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

unsigned long end_turn = 0;
unsigned long stop_turn = 0;
unsigned long start_turn = 0;
bool turning_left = false;
bool turned = false;
bool sawline = false;  // Whether the front sensors have seen the line yet.

void Turn(unsigned long mintime, bool left) {
  sawline = false;
  turned = false;
  writeMotors(-20, -20);
  turning_left = left;
  stop_turn = millis() + mintime;
  start_turn = millis() + turndelay;
  Serial.println(stop_turn);
  Serial.println(millis());
}
bool TurnUpdate() {
  int sensval = analogRead(backline);
  const int kCutoff = 400;
  bool online = sensval > kCutoff;
  int frontsens = lf->Read();
  // Don't stop turning until we have data from line following sensors.
  if ((frontsens > 2800 && frontsens < 4200) && millis() > stop_turn) {
    sawline = true;
  }
  if (millis() > end_turn && turned) {
    rcounter->reset_timeout();
    writeMotors(0, 0);
    return true;
  } else if ((millis() > stop_turn - turndelay) && (online || turned)) {
    // Reverse motors abruptly.
    if (sawline) {
      if (!turning_left)
        writeMotors(-14, 14);
      else
        writeMotors(14, -14);
    } else
      end_turn = millis() + 120;
    // Until both have happened, keep on pushing this back.
    if (!sawline || !turned) end_turn = millis() + 250;
    turned = true;
  } else if (millis() > start_turn && !turned) {
    if (turning_left)
      writeMotors(-14, 14);
    else
      writeMotors(14, -14);
  }
  return false;
}
