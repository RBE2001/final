#ifndef __LOOP_H__
#define __LOOP_H__
#include "Arduino.h"
/**
 * This class is meant to provide a non-real time (ie, not driven by timer
 * interrupts) way of having functions run at some regular interval. Meant for
 * things like basic pid, status loops, blue tooth, etc.
 */

class Loop {
 public:
  Loop(unsigned long period /*microseconds*/) : period_(period), endtime_(0) {}

  void set_period(unsigned long period) { period_ = period; }

  // Checks if it sit ime to call the run() function and does so if necessary.
  void Update() {
    if (micros() >= endtime_) {
      endtime_ += period_;
      Run();
    }
  }

  // The run method should contain whatever you want done at the appropriate
  // interval.
  virtual void Run()=0;

 private:
  // Time at which to next call the Run() function.
  unsigned long endtime_;

  // Period, in milliseconds, at which to call the Run() function.
  unsigned long period_;
};

#endif  // __LOOP_H__
