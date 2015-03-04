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
// See Pololu documentation for more information.
unsigned *LineFollower::sensors() {
  unsigned values[num_sensors_];
  qtrrc.readCalibrated(values);
  return values;
}

// Performs PID calculation.
// Returns a value -90 to 90, in same units as are passed to the Servo.write
// function.
int8_t LineFollower::Calc() {
  // Calculates error.
  int error = setpoint_ - Read();
#ifdef DEBUG
  Serial.print(error);
  Serial.print("\t");
#endif
  sum_ += error;

  // Chose pid constants depending on if we are line following forwards or
  // backwards.
  float p = backwards_ ? bp_ : p_;
  float i = backwards_ ? bi_ : i_;
  float d = backwards_ ? bd_ : d_;
  float retval = p * error + i * sum_ + d * (error - prev_error_);
  // Cap values to within appropriate range.
  if (retval > 90) retval = 90;
  if (retval < -90) retval = -90;
  prev_error_ = error;
#ifdef DEBUG
  Serial.print(retval);
  Serial.print("\t");
#endif
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
  // Exclusive Or -- Basically, we want to reverse the motor values if only one
  // if linv_ or backwards_ is set for each motor.
  if (linv_ != backwards_) left = 180 - left;
  if (rinv_ != backwards_) right = 180 - right;
#ifdef DEBUG
  Serial.print(left);
  Serial.print("\t");
  Serial.println(right);
#endif
  if (enable_outputs_) {
    left_->write(left);
    right_->write(right);
  }
}
