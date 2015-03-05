// This file is just meant for messing around with tuning the armpid on the
// robot; it is not a finished product or for use on the final robot. See the
// README for more information.
#include "Arduino.ha"
#include <Servo.h>
#include "armpid.h"

// Declare armpid object. we don't want to construct this before the setup()
// function because the armpid constructor includes clals to Servo.attach, which
// must be made in or after setup().
ArmPID *arm;

Servo wrist, gripper;

void setup() {
  Serial.begin(115200);
  // Initialize things with various port numbers and PID values.
  arm = new ArmPID(9, 2, 23, 0.25, 0.0018, 2.0);
  arm->set_setpoint(150);
  wrist.attach(8);
  gripper.attach(7, 500, 2600);
}

unsigned long switchtime = 0;
bool up = true;

void loop() {
  if (millis() > switchtime) {
    switchtime += 5000;
    up = !up;
    arm->set_setpoint(up ? 250 : 140);
    if (up) {
      Serial.println("Going Up");
      gripper.write(180);
    }
    else {
      Serial.println("Going Down");
      wrist.write(10);
      gripper.write(90);
    }
  }
  if (millis() > switchtime - 4500 && up) wrist.write(100);
  if (millis() > switchtime - 1000 && !up) gripper.write(180);
  arm->Update();
}
