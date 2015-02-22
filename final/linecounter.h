/**
 * The LineCounter simply uses a line sensor to estimate the number of reasonably spaced out lines that have passed.
 */

#ifndef __LINECOUNTER_H__
#define __LINECOUNTER_H__
#include "Arduino.h"
#include "loop.h"

class LineCounter : public Loop {
 public:
  LineCounter(uint8_t port, int cutoff=500, int delay=400)
      : port_(port),
        kCutoff(cutoff),
        kDelay(delay),
        timeout_(0),
        count_(0),
        inc_(true),
        Loop(0UL /* Poll as fast as possible */) {}

  uint8_t count() { return count_; }
  void set_count(uint8_t count) { count_ = count; }
  void reset() { set_count(0); }
  // Whether to increment or decrement when we encounter a line.
  void increment(bool inc) { inc_ = inc; }

  void Run() {
    if (millis() > timeout_) {
      int sensor = analogRead(port_);
      if (sensor > kCutoff /*over line*/) {
        if (inc_) count_++;
        else count_--;
        Serial.print("Count changed to:\t");
        Serial.println(count_);
        // Found line, now reset timeout_.
        timeout_ = millis() + kDelay;
      }
      else return;
    }
    else return;
  }

 private:
  const int kCutoff;
  const int
      kDelay;  // minimum time between two line counts, millis. For filtering.
  uint8_t port_;

  uint8_t count_;

  bool inc_;

  unsigned long timeout_; // millis() time to start looking for lines again.
};
#endif  // __LINECOUNTER_H__
