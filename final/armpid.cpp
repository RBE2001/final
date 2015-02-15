#include "armpid.h"
#include "Arduino.h"
#include <Servo.h>

// Performs simple PID calculation.
uint8_t ArmPID::Calc() {
  // Calculate Error. Not the analogRead blocks until ADC finishes reading.
  int error = setpoint_ - analogRead(pot_);
  sum_ += error;
  // Perform actual PID calculation.
  uint8_t retval = p_ * error + i_ * sum_ + d_ * (error - prev_error_);
  prev_error_ = error;
  return retval;
}
