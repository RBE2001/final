#ifndef __ARMPID_H__
#define __ARMPID_H__
#include <Servo.h>
#include "Arduino.h"
#include "loop.h"

class ArmPID : public Loop {
 public:
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

  void set_pid(float p, float i, float d) {
    p_ = p;
    i_ = i;
    d_ = d;
  }

  void set_setpoint(int setpoint) { setpoint_ = setpoint; }

  void run() {
    motor_.write(out_to_raw(calc()));
  }

 private:
  uint8_t calc() {
    int error = setpoint_ - analogRead(pot_);
    sum_ += error;
    uint8_t retval = p_ * error + i_ * sum_ + d_ * (error - prev_error_);
    prev_error_ = error;
    return retval;
  }

  uint8_t out_to_raw(uint8_t out) { return out + 90; }

  int setpoint_;
  int prev_error_;
  long sum_;
  float p_, i_, d_;
  uint8_t pot_;
  Servo motor_;
};
#endif  // __ARMPID_H__
