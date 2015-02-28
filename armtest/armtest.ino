#include "Arduino.h"
#include <Servo.h>
#include "armpid.h"

ArmPID *arm;

Servo wrist, gripper;

void setup() {
  Serial.begin(115200);
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
