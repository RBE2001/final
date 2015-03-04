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
  // Constructor which takes a motor port, analog input for the pot, digital
  // input port for the bottom limit button, and initial PID values. Initializes
  // loop to 100 Hz by default.
  // Also, this calls Servo::attach, which should not be called before the main
  // setup() function, so instances of this class should be initialized in or
  // after the setup() function.
  ArmPID(uint8_t motor, uint8_t pot, uint8_t button, float p = 0.0, float i = 0.0,
         float d = 0.0)
      : pot_(pot),
        p_(p),
        i_(i),
        d_(d),
        prev_error_(0),
        sum_(0),
        setpoint_(0),
        button_(button),
        Loop(1e4 /*10000us => 100 Hz*/) {
    motor_.attach(motor);
    pinMode(button_, INPUT_PULLUP);
  }

  // Set all of the PID values.
  // Units of throttle (-90 to 90) per unit error (error is retrieved from ADC
  // and so can range from -1023 to 1023).
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
  // Actually performs PID calculation and writes it to the motor.
  void Run() { motor_.write(OutToRaw(Calc())); }

 private:
  // Performs the PID calculations, using the current setpoint and reading from
  // the ADC to calculate a value in units of the Servo::write function,
  // although it will be centered on zero and may be inverted.
  int Calc();

  // Convert between the output of the PID loop (which centers on zero and may
  // be inverted) to useful motor values (typically 0 - 180).
  uint8_t OutToRaw(int out) {
    if (!digitalRead(button_) && prev_error_ <= 0) {
#ifdef DEBUG
      Serial.println("Not bothering...");
#endif  // DEBUG
      return 90;
    }
    else return out + 90;
  }

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
  // Digital In for bottom button.
  uint8_t button_;
  // Servo object for motor.
  Servo motor_;
};
#endif  // __ARMPID_H__
