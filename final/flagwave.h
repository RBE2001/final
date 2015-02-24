#include <Servo.h>

const int pinFlag = 6;
int nextwave;
int waveinterval = 300;
int degreeFlag = 180;

boolean waving? = false;

Servo FlagServo;

void flagNull() {
  FlagServo.write(90);
  waving? = false;
}

void flagWave() {
  nextwave = millis() + waveinterval;
  if (waving? = false) {
    FlagServo.write(degreeFlag);
    waving? = true;
    changeDegree();
  }
  if (waving? = true) {
    if (millis() >= nextwave) {
      FlagServo.write(degreeFlag);
      changeDegree();
    }
    if (millis() < nextwave) {
      FlagServo.write(degreeFlag);
  }
}

void changeDegree() {
  if (degreeFlag = 180) (degreeFlag = 0);
  if (degreeFlag = 0) (degreeFlag = 180);

  nextwave = millis() + waveinterval;
}


