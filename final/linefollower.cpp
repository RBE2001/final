// LineFollower implementation. See linefollower.h for more info.
#include "linefollower.h"

#include <Servo.h>
#include <QTRSensors.h>
#include "Arduino.h"
#include "loop.h"

// Takes ~10 seconds to calibrate the line sensor array.
void LineFollower::Calibrate() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  for (int i = 0; i < 400; i++) {
    // According to docs, takes ~25 ms per loop.
    qtrrc.calibrate();
  }
  digitalWrite(13, LOW);
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
uint8_t LineFollower::Calc() {
  // Calculates error.
  int error = setpoint_ - Read();
  sum_ += error;
  uint8_t retval = p_ * error + i_ * sum_ + d_ * (error - prev_error_);
  prev_error_ = error;
  return retval;
}

// Writes values out to the motors using the pid output.
void LineFollower::Write(uint8_t pidout) {
  // Prevents outputs that would create motor speeds > maxspeed_.
  uint8_t maxpid = maxspeed_ - motorspeed_;
  if (pidout > maxpid) pidout = maxpid;
  else if (pidout < -maxpid) pidout = -maxpid;
  // Calculate speeds for left and right and actually perform the write.
  uint8_t left = motorspeed_ + pidout;
  uint8_t right = motorspeed_ - pidout;
  left_.write(left);
  right_.write(right);
}
