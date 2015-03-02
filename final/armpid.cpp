#include "armpid.h"
#include "Arduino.h"
#include <Servo.h>

// Performs simple PID calculation.
int ArmPID::Calc() {
  // Calculate Error. Not the analogRead blocks until ADC finishes reading.
  int error = setpoint_ - analogRead(pot_);
  sum_ += (abs(error) > 20) ? 0 : error;
  // Perform actual PID calculation.
  int retval = p_ * error + i_ * sum_ + d_ * (error - prev_error_);
  int max = 30;
  if (error < 5) max = 5; // Prevent running into stops.
  if (retval > max) retval = max;
  if (retval < -max) retval = -max;
  // Account for small deadband + gravity.
  if (retval > 0) retval += 12;
  else retval += 0;
  prev_error_ = error;
  /*
  Serial.print("Error:\t");
  Serial.print(error);
  Serial.print("\tOut:\t");
  Serial.println(retval);
  */
  return retval;
}
