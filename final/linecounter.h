/**
 * The LineCounter simply uses a line sensor to estimate the number of
 * reasonably spaced out lines that have passed.
 */

#ifndef __LINECOUNTER_H__
#define __LINECOUNTER_H__
#include "Arduino.h"
#include "loop.h"

class LineCounter : public Loop {
 public:
  // Takes a port for the sensor being used, a cutoff (in units as read from the
  // ADC) for what is black/white for the sensor, and a delay (in milliseconds)
  // for what the minimum time to wait between counting lines--this delay
  // prevents a line being doublecounted but could also cause issues with an
  // unusually fast robot.
  LineCounter(uint8_t port, int cutoff = 500, int delay = 400)
      : port_(port),
        kCutoff(cutoff),
        kDelay(delay),
        timeout_(0),
        count_(0),
        inc_(true),
        Loop(0UL /* Poll sensor as fast as possible */) {}

  // Returns the number of lines passed so far.
  uint8_t count() { return count_; }

  // Set the count for the number of lines passed so far.
  void set_count(uint8_t count) { count_ = count; }

  // Reset the count to zero.
  void reset() { set_count(0); }

  // Whether to increment or decrement when we encounter a line.
  void increment(bool inc) { inc_ = inc; }

  // Run function; called very iteration of main arduino loop.
  void Run() {
    // Only run anything if it has been at least timeout_ milliseconds since we
    // last saw a line.
    if (millis() > timeout_) {
      // Read sensor and determine whether or not we are over a line.
      int sensor = analogRead(port_);
      if (sensor > kCutoff /*over line*/) {
        if (inc_)
          count_++;
        else
          count_--;
#ifdef DEBUG
        Serial.print("Count changed to:\t");
        Serial.println(count_);
#endif
        // Found line, now reset timeout_.
        timeout_ = millis() + kDelay;
      }
    }
    return;
  }

  // Resets timeout to be kDelay milliseconds in the future.
  void reset_timeout() { timeout_ = millis() + kDelay; }

 private:
  // Cutoff sensor value for determining whether or not a line is black/white.
  const int kCutoff;

  // minimum time between two line counts, millis. For filtering.
  const int kDelay;

  // Analog In port of sensor.
  uint8_t port_;

  // Current count of the number of lines passed.
  uint8_t count_;

  // Whether to increment up or down when we encounter a line.
  bool inc_;

  unsigned long timeout_;  // millis() time to start looking for lines again.
};
#endif  // __LINECOUNTER_H__
