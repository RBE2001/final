#include "armpid.h"
#include "Arduino.h"
#include <Servo.h>
// See armpid.h for a more thorough description.

// Performs simple PID calculation.
// Returns a value which can relatively easily be fed into the arduino
// Servo.write function. The return value will be from -90 to 90 and the write
// function takes a value from 0 to 180.
int ArmPID::Calc() {
  // Calculate Error. Note the analogRead blocks until ADC finishes reading.
  int error = setpoint_ - analogRead(pot_);
  sum_ += (abs(error) > 20) ? 0 : error;
  // Perform actual PID calculation.
  int retval = p_ * error + i_ * sum_ + d_ * (error - prev_error_);
  int max = 30; // maximum allowable throttle (max can range from 0 - 90).
  // Prevent running into stops too hard, especially since there are things that
  // can break near the bottom and top goal positions.
  if (error < 5) max = 5;
  if (retval > max) retval = max;
  if (retval < -max) retval = -max;
  // Account for small deadband + gravity.
  if (retval > 0) retval += 12;
  else retval += 0;
  prev_error_ = error;
#ifdef DEBUG
  // Debugging prints.
  Serial.print("Error:\t");
  Serial.print(error);
  Serial.print("\tOut:\t");
  Serial.println(retval);
#endif  // DEBUG
  return retval;
}
