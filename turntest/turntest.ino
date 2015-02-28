#include <Servo.h>
#include <QTRSensors.h>
#include "loop.h"
#include "linefollower.h"

const unsigned char num_sensors = 8;
unsigned char linepins[num_sensors] = {52, 53, 51, 49, 47, 27, 25, 23};
LineFollower *lf;
Servo *lmotor, *rmotor; // Same as used in lf.
const uint8_t lmotorport = 10;
const uint8_t rmotorport = 11;
const bool linverted = false;
const bool rinverted = true;
const unsigned long turndelay = 100;

#ifndef NEW
const uint8_t backline = 0;
const int kCutoff = 500;
unsigned long starttime = 0;
#endif

void Turn(unsigned long time /*ms*/, bool left);
bool TurnUpdate(); // Returns whether done.

void setup() {
  Serial.begin(115200);
  lf = new LineFollower(linepins, num_sensors, lmotorport, rmotorport,
                        linverted, rinverted);
  lmotor = lf->left();
  rmotor = lf->right();

  //Turn(1350, false);
  Turn(2550, false);
}

void loop() {
  if (TurnUpdate()) {
    writeMotors(0, 0);
    exit(0);
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
bool turned = false;
bool turning_left = false;
void Turn(unsigned long mintime, bool left) {
  turned = false;
  writeMotors(-20, -20);
  turning_left = left;
  stop_turn = millis() + mintime;
  start_turn = millis() + turndelay;
  starttime = millis();
  Serial.println(stop_turn);
  Serial.println(millis());
}
bool TurnUpdate() {
  int sensval = analogRead(backline);
  bool online = sensval > kCutoff;
  if (millis() > end_turn && turned) {
    writeMotors(0, 0);
    Serial.println(end_turn - starttime);
    return true;
  }
  else if ((millis() > stop_turn - turndelay) && (online || turned)) {
    // Reverse motors abruptly.
    if (!turning_left) writeMotors(-10, 10);
    else writeMotors(10, -10);
    if (!turned) end_turn = millis() + 20;
    turned = true;
  }
  else if (millis() > start_turn && !turned) {
    if (turning_left) writeMotors(-20, 20);
    else writeMotors(20, -20);
  }
  return false;
}
