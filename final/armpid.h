/**
 * The ArmPID class is intended to provide a basic positional PID control for
 * something with one motor and a potentiometer for feedback.
 */
#ifndef __ARMPID_H__
#define __ARMPID_H__
#include <Servo.h>
#include "Arduino.h"
#include "loop.h"

/**
 * This class inherits from Loop, which just causes the Run function to be
 * called at approximately the desired rate (it doesn't use interrupts or
 * anything, so it is just a soft guarantee).
 */
class ArmPID : public Loop {
 public:
  // Constructor which takes a motor port, analog input for the pot, and initial
  // PID values. Initializes loop to 100 Hz.
  // Also, this calls Servo::attach, which should not be called before the main
  // setup() function, so instances of this class should be initialized in or
  // after the setup() function.
  ArmPID(uint8_t motor, uint8_t pot, float p = 0.0, float i = 0.0,
         float d = 0.0)
      : pot_(pot),
        p_(p),
        i_(i),
        d_(d),
        prev_error_(0),
        sum_(0),
        setpoint_(0),
        Loop(1e4 /*100 Hz*/) {
    motor_.attach(motor);
  }

  // Set all of the PID values.
  void set_pid(float p, float i, float d) {
    p_ = p;
    i_ = i;
    d_ = d;
  }

  // Set the desired setpoint.
  void set_setpoint(int setpoint) {
    sum_ = 0;
    setpoint_ = setpoint;
  }

  // Used by the Loop class; called once per cycle.
  void Run() { motor_.write(OutToRaw(Calc())); }

 private:
  // Performs the PID calculations, using the current setpoint and reading from
  // the ADC to calculate a value in units of the Servo::write function,
  // although it will be centered on zero and may be inverted.
  int Calc();

  // Convert between the output of the PID loop (which centers on zero and may
  // be inverted) to useful motor values (typically 0 - 180).
  uint8_t OutToRaw(int out) { return out + 90 + 10; }

  // Current setpoint to use when calculating error.
  int setpoint_;
  // Previous error for calculating D term.
  int prev_error_;
  // Sum of all errors for calculating I term.
  long sum_;
  // PID gains.
  float p_, i_, d_;
  // Analog In port of pot.
  uint8_t pot_;
  // Servo object for motor.
  Servo motor_;
};
#endif  // __ARMPID_H__
