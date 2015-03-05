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

// Represents the current direction the robot is facing. The code which has
// logic for determining which direction the robot should turn does make use of
// the fact that the enum can be cast to ints, so the numbering is important.
// Up is towards the second reactor, down is towards the first reactor, left is
// towards the storage tubes, and right is towards the supply tubes.
enum Direction {
  // CCW order is important.
  kUp = 0,
  kLeft = 1,
  kDown = 2,
  kRight = 3,
} dirstate;  // Direction robot is facing.

// Represents where on the field the robot is.
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
// -2 is used in a few places for non-line related goals.
volatile int goal = -1;

// whether we have re-supplied reactor 0.
bool zerodone = false;

// Whether to line follow.
bool updatelf = true;

// For serial debugging; time of next update to send so we don't spam serial
// console.
unsigned long next_update = 0;

// The setup function performs appropriate initialization, calibration, and
// setup for various components.
void setup() {
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("Hello.");
#endif  // DEBUG

  // Initialize various classes.
  bt = new Bluetooth(team);
  lf = new LineFollower(linepins, num_sensors, lmotorport, rmotorport,
                        linverted, rinverted);
  lcounter = new LineCounter(lline);  // Unused.
  rcounter = new LineCounter(rline);
  arm = new ArmPID(armmotor, armpot, armbutton, 0.25 /*p*/, 0.0018 /*i*/,
                   2.0 /*d*/);
  lmotor = lf->left();
  rmotor = lf->right();
  wrist.attach(wristservo);
  gripper.attach(gripservo, 500, 2600 /*Need full reach of servo*/);

  // Initialize various states to have robot facing down towards first reactor.
  state = kLine;
  // Pointing straight towards near (Down) reactor about to pull up tube.
  rodstate = kGetReactor;
  dirstate = kDown;
  nearline = 0;
  locstate = kCenter;
  goal = -1;

  // Initialize front trigger.
  pinMode(vtrigger, INPUT_PULLUP);

#ifdef DEBUG
  Serial.println("Done constructing.");
#endif  // DEBUG

  // Set up servos as open, ready to grip from reactor.
  wrist.write(vertwrist);
  gripper.write(opengrip);
  arm->set_setpoint(armup);

  // Wait for start button to be pressed to begin calibration.
  pinMode(button, INPUT_PULLUP);
  while (digitalRead(button)) continue;

#ifdef DEBUG
  Serial.println("Calibrating.");
#endif  // DEBUG

  // Take a few seconds to calibrate linefollowing sensors by moving them over
  // darkest and lightest areas of board.
  lf->Calibrate();

#ifdef DEBUG
  Serial.println("Done Calibrating.");
#endif  // DEBUG

  // Delay until we want to start, all the while updating bluetooth.
  while (digitalRead(button)) bt->Update();

  // Delay so that we have time to press button -- we don't want to start whie
  // our hand is still on the robot.
  for (int i = 0; i < 5; i++) {
    delay(200);
    bt->Update();
  }

  // Set forwards and packward PID values for the linefollowing. We only use P
  // and D gains.
  lf->set_pid(9e-3, 0.0, 1.1e-1);
  lf->set_back_pid(5e-3, 0.0, 0.1);
}  // setup()

// Main arduino loop function. In theory, this loop takes zero time (ie, no
// blocking functions are called). I have not actually measured the total loop
// delay, but I expect that it is on the order of several ms, depending on
// whether we are outputting to the Serial console or not.
// Note that the robot has been designed such that there is no need for
// interrupt-level timing in order for everything to work, so jitter of a few
// milliseconds is survivable.
void loop() {
  // Highest-level state machine, determines whether we are line following,
  // turning, or manipulating something with the arm.
  // kLine is by far the most substantial case in this whole switch; the rest of
  // the cases just do their thing and get straight back to line following, but
  // line following has to handle navigation, where we are on the field, what to
  // do next, etc.
  switch (state) {
    case kLine:
      // By default, we want to update the line follower.
      updatelf = true;

      // A few useful variables that are used later.
      int dirdiff;        // amount that we have to turn, in quarter-turns.
      Direction goaldir;  // Goal direction.
      bool just_starting, onsupplyline;  // Local state variables.

      // Switch for choosing what to do depending on what stage of the process
      // we
      // are in.
      switch (rodstate) {
        // Handles case for when we need to get a rod from the reactor.
        case kGetReactor:
          // Initialize arm to appropriate state.
          wrist.write(vertwrist);
          gripper.write(opengrip);
          arm->set_setpoint(armup);

          // If we just entered this state, then goal will need to be set to
          // either the top or bottom reactor.
          // Note that this has no way of knowing if we have already completed
          // the second reactor, so once completing the second reactor, the code
          // will keep on working on the second reactor.
          if (goal == -1) {
            goal = zerodone ? 1 : 0;
#ifdef DEBUG
            Serial.print("Setting goal to: ");
            Serial.println(goal);
#endif  // DEBUG
          }

          // If we have reached the reactor, do appropriate stuff.
          if (!digitalRead(vtrigger)) {
            // Put is in appropriate state.
            state = kReactorPull;
            placestate = kStartArm;

            // Stop the robot and set arm positions.
            writeMotors(0, 0);
            arm->set_setpoint(armdown);
            wrist.write(vertwrist);
            gripper.write(opengrip);

            // Stop line following.
            updatelf = false;
            goal = -1;  // Store will need to decide where to put this.
            break;      // kGetReactor
          }

          // All this code decides whether/where we need to turn to work on
          // getting to the reactor and executes said turn.
          if ((dirstate == kUp && goal == 0) ||
              (dirstate == kDown && goal == 1)) {  // Need to turn around.
            // Start turning, stop line following, and update direction state.
            Turn(turn180, true);
            state = kTurn;
            updatelf = false;
            dirstate = (Direction)(2 - goal * 2);
            break;  // case kGetReactor
          } else if (dirstate == kLeft || dirstate == kRight) {
            // Start turning, stop line following, and update direction state.
            // Point up or down so that above if can deal with it in the above
            // code.
            // Turn right so that dirstate can never go negative.
            Turn(turn90, false);
            state = kTurn;
            updatelf = false;
            dirstate = (Direction)((int)dirstate - 1);
            break;  // case kGetReactor
          }
          break;  // case kGetReactor

        // Handles case if we are going from reactor to the storage tubes to
        // place the rod.
        case kStore:
          // Initialize arm/gripper to position appropriate for storage.
          arm->set_setpoint(armup);
          wrist.write(flatwrist);
          gripper.write(closegrip);

          // Determines what our goal position will be; just goes with lowest
          // numbered open port.
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

          // If our goal is still -1 after the previous code, then something is
          // wrong.
          if (goal == -1) {
            // Something is wrong; No storage is available.
            updatelf = false;
            writeMotors(0, 0);
            break;  // kStore
          }

          // Make sure that we are pointing the right direction.
          // Determine what direction we should be facing.
          if (locstate == kCenter) {
            // Checks if we have hit the correct line yet.
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

          dirdiff = (int)goaldir - (int)dirstate;  // Positive = turn left.
          // Determine appropriate way to call the Turn() function.
          if (dirdiff != 0) {
            // Avoids turnign more than 180 degrees.
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

        // Case for when we are going to try and retrieve a new rod.
        // Perhaps the most complicated rodstate case, because it has to both
        // navigate off of the storage lines, onto the supply lines, and stop in
        // a way that avoids jerking the robot so much that it misses the grab
        // of the supply tube.
        case kGetSupply:
          // Set arm/gripper appropriately.
          arm->set_setpoint(armup);
          // Tilted up slightly to avoid bashing gripper into tube.
          wrist.write(tiltedwrist);
          gripper.write(opengrip);

          // Basically same logic for determining goal as kStore, but with
          // different variables.
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
            break;  // kGetSupply
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

          // Actually execute turn based on what our goal is.
          dirdiff = (int)goaldir - (int)dirstate;  // Positive = left.
          if (dirdiff != 0) {
            int turntime = abs(dirdiff) == 2 ? turn180 : turn90;
            bool leftturn = (dirdiff > 0) ^ (abs(dirdiff) == 3);
            Turn(turntime, leftturn);
            state = kTurn;
            updatelf = false;
            dirstate = goaldir;
            place_action_end = millis() + supplyturn;
            break;  // kGetSupply
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
            break;  // kGetSupply
          }

          // Logic to determine if we are on the supply line, haven't already
          // encountered a line/trigger (which cause us to finish), and at least
          // a certain minimum time has passed.
          onsupplyline =
              locstate != kCenter && goal != -2 && place_action_end < millis();

          // Slows robot down when we hit the line in front of the supply tube
          // and allows the robot to coast into the post, reducing jerking.
          if ((rcounter->count() && onsupplyline) || goal == -3) {
            goal = -3;
            // Slow down and go forwards.
            updatelf = false;  // stop line following.
            writeMotors(0, 0);
          }

          // If we are on the lines and our trigger is hit, then we have
          // arrived!
          if (!digitalRead(vtrigger) && onsupplyline) {
            updatelf = false;
            goal = -2;  // backup a short bit before grabbing the rod..
            gripper.write(slightgrip);
            place_action_end = millis() + supplybackup;
            writeMotors(-12, -12);
            break;  // kGetSupply
          } else if (goal == -2 && millis() > place_action_end) {
            // Stop robot and start grabbing
            writeMotors(0, 0);
            updatelf = false;
            goal = -1;
            state = kSupplyPull;
            placestate = kStartArm;
            break;  // kGetSupply
          } else if (goal == -2) {
            // Continue backing up for a brief time until place_action_end has
            // passed.
            updatelf = false;
            writeMotors(-12, -12);
          }
          break;  // case kGetSupply

        // Deals with putting supply rod back into the reactor.
        case kSetReactor:
          // Set arm to appropriate position.
          arm->set_setpoint(armup);
          wrist.write(upsidedownwrist);
          gripper.write(closegrip);

          // If we hit the post, then we are ready to begin dropping the rod.
          if (!digitalRead(vtrigger)) {
            state = kReactorDrop;
            placestate = kStartArm;
            writeMotors(0, 0);
            updatelf = false;
            goal = -1;  // Store will need to decide where to put this.
            zerodone = true;
            break;  // kSetReactor
          }

          // If we are just starting and don't know our goal, choose.
          if (goal == -1) {
            goal = zerodone ? 1 : 0;
          }

          // Make sure that we are pointing the right direction.
          // Determine what direction we should be facing.
          if (locstate == kCenter) {
            goaldir = goal ? kUp : kDown;
          } else if (locstate = kSupplyLines)
            goaldir = kLeft;
          else if (locstate = kStorageLines)  // Not sure how it would ever end
                                              // up in this case...
            goaldir = kRight;

          // ACtually execute appropriate turns.
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

          // Stop and turn if we are on the sides and hit the center line.
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

    // Handles turning until TurnUpdate() finishes.
    case kTurn:
      if (TurnUpdate()) state = kLine;
      break;  // case kTurn

    // These cases will use goal to indicate whether a particular stage of the
    // action has started/finished. goal will be -1 during the transition and 0
    // once a particular PlaceAction has started.
    // All of these actions (kReactorPull, kReactorDrop, kSupplyPull,
    // kStorageDrop) use the same basic routine:
    // 1) Position the arm appropriately.
    // 2) Grab or release the rod.
    // 3) Remove the arm appropriately.
    // 4) Back Up.

    // Handles when we need to grab a tube from the reactor. Called when the
    // rodstate kGetReactor completes and initiates rodstate = kStore.
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

    // Very similar in structure to kReactor Pull, but instead is called after
    // rodstate == kSetReactor and leads into rodstate = kGetReactor.
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

    // Again, the same basic structure. Occurs after rodstate = kGetSupply and
    // transitions to rodstate = kSetReactor.
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

    // Again, the same basic structure. Occurs after rodstate = kStore and
    // transitions to rodstate = kGetSupply.
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
  bt->Update();  // updates bluetooth.

  // If we have been stopped, stop linefollowing and motors.
  if (bt->stopped()) {
    updatelf = false;
    // We don't want to accidentally mess up a turn, and turning is not really a
    // movement that needs to be stopped by the stop button anyways. All turns
    // complete in less than a couple seconds anyways.
    if (state != kTurn) writeMotors(0, 0);
  }

  // Decide whether to run linefollower or not.
  lf->enable_outputs(updatelf);
  // Note that it is important that we always update the linefollower even if we
  // aren't setting outputs, because if the line tracking sensor happen to get
  // off of the line while we are turning, they will remember which direction
  // they need to turn in to get back onto the line.
  lf->Update();

  // We only want to update the counter when we are pure line following.
  // When following the center line, we want to:
  // -Decrement if dirstate == kDown and nearline > 0.
  // -Increment if dirstate == kUp and nearline < 4.
  // When on the side lines, we want to set to zero and let it update. Any
  // functions that need to know will have one loop iteration to catch it, which
  // is fine since this is only a single threaded process.
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

  // Update arm pid.
  arm->Update();

#ifdef DEBUG
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
#endif  // DEBUG
}

// Simple interface for writing left and right drive motors.
void writeMotors(int left, int right) {
  left = linverted ? left : -left;
  right = rinverted ? right : -right;
  lmotor->write(90 - left);
  rmotor->write(90 - right);
}

// State variables used in turning. Times in milliseconds.
unsigned long end_turn = 0;    // Definitive end time for turn.
unsigned long stop_turn = 0;   // Time to start stopping the turn.
unsigned long start_turn = 0;  // Time to actually start the turn.
bool turning_left = false;
bool turned = false;   // Whether we have seen a line with the back line sensor.
bool sawline = false;  // Whether the front sensors have seen the line yet.

// The structure of a turn is as follows:
// 1) Briefly backup to eliminate previous forwards motion.
// 2) At the time start_turn, start turning the motors for the actual turn.
// 3) Once we have seen lines with both the front and back line sensors and a
// certain minimum timeout specified by stop_turn has passed (because the line
// sensors will trigger too early if we are still on our original line), briefly
// reverse the motors.
// 4) After another brief interval, we stop the motors and return true.

// Initiates turn with a minimum time of mintime turning either left or right.
void Turn(unsigned long mintime, bool left) {
  sawline = false;
  turned = false;
  writeMotors(-20, -20);
  turning_left = left;
  stop_turn = millis() + mintime;
  start_turn = millis() + turndelay;
#ifdef DEBUG
  Serial.println(stop_turn);
  Serial.println(millis());
#endif  // DEBUG
}

// Called every iteration; contains a mini state machine which decides which
// part of the turn, as described above, we are in and returns true if the turn
// is over and false otherwise.
bool TurnUpdate() {
  int sensval = analogRead(backline);  // Retrieves the back line sensor.
  // Cutoff for whether the back sensor is seeing black or white.
  const int kCutoff = 400;
  bool online = sensval > kCutoff;  // True if back sensor is on a line.
  int frontsens = lf->Read();  // Position of line under front; ranges 0 - 7000.

  // Don't stop turning until we have data from front line following sensors.
  // Also, the line must be sufficiently centered under the front sensors.
  if ((frontsens > 2800 && frontsens < 4200) && millis() > stop_turn) {
    sawline = true;
  }

  // Check to see if we can go ahead and finish the turn.
  if (millis() > end_turn && turned) {
    // Avoid accidentally counting extra lines immediately after turn.
    rcounter->reset_timeout();
    writeMotors(0, 0);
    return true;
  }
  // If the back sensor is on or has seen a line, and enough time has passed.
  else if ((millis() > stop_turn - turndelay) && (online || turned)) {
    // Reverse motors abruptly if the front sensor has also seen a line already.
    if (sawline) {
      if (!turning_left)
        writeMotors(-14, 14);
      else
        writeMotors(14, -14);
    } else
      end_turn = millis() + 120;  // essentially, end_turn = inf.
    // Until both sensors have seen a line, keep on pushing this back.
    if (!sawline || !turned) end_turn = millis() + 250;

    // Flag so that once the back line sensor leaves the line, we still know
    // that it has passed.
    turned = true;
  }
  // If enough time has passed so that we can start our actual turning, do so.
  else if (millis() > start_turn && !turned) {
    if (turning_left)
      writeMotors(-14, 14);
    else
      writeMotors(14, -14);
  }

  return false;
}
