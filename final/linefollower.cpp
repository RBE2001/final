// LineFollower implementation. See linefollower.h for more info.
#include "linefollower.h"

#include <Servo.h>
#include <QTRSensors.h>
#include "Arduino.h"
#include "loop.h"

// Takes ~10 seconds to calibrate the line sensor array.
void LineFollower::Calibrate() {
  for (int i = 0; i < 200; i++) {
    // According to docs, takes ~25 ms per loop.
    qtrrc.calibrate();
  }
}

// Reads all the line sensors and returns the values, normalized for whatever
// the current calibration is (ranges from 0 - 1000). The actual read itself
// takes ~2.5ms.
unsigned *LineFollower::sensors() {
  unsigned values[num_sensors_];
  qtrrc.readCalibrated(values);
  return values;
}

// Performs PID calculation.
int8_t LineFollower::Calc() {
  // Calculates error.
  int error = setpoint_ - Read();
  //Serial.print(error);
  //Serial.print("\t");
  sum_ += error;
  float retval = p_ * error + i_ * sum_ + d_ * (error - prev_error_);
  if (retval > 90) retval = 90;
  if (retval < -90) retval = -90;
  prev_error_ = error;
  //Serial.print(retval);
  //Serial.print("\t");
  return (int8_t)retval;
}

// Writes values out to the motors using the pid output.
void LineFollower::Write(int8_t pidout) {
  // Prevents outputs that would create motor speeds > maxspeed_.
  int8_t maxpid = maxspeed_ - motorspeed_;
  if (pidout > maxpid) pidout = maxpid;
  else if (pidout < -maxpid) pidout = -maxpid;
  // Calculate speeds for left and right and actually perform the write.
  uint8_t left = motorspeed_ + pidout;
  uint8_t right = motorspeed_ - pidout;
  // Exclusive Or.
  if (linv_ != backwards_) left = 180 - left;
  if (rinv_ != backwards_) right = 180 - right;
  if (backwards_)  {
    int tmp = left;
    left = 180 - right;
    right = 180 - tmp;
  }
  //Serial.print(left);
  //Serial.print("\t");
  //Serial.println(right);
  if (enable_outputs_) {
    left_->write(left);
    right_->write(right);
  }
}
