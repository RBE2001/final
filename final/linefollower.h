#ifndef __LINEFOLLOWER_H__
#define __LINEFOLLOWER_H__
#include <Servo.h>
#include <QTRSensors.h>
#include "Arduino.h"
#include "loop.h"

class LineFollower : public Loop {
 public:
  LineFollower(unsigned char *pins, unsigned char num_sensors, uint8_t left,
               uint8_t right)
      : qtrrc(pins, num_sensors),
        p_(0),
        i_(0),
        d_(0),
        prev_error_(0),
        sum_(0),
        setpoint_((num_sensors - 1) * 1000 / 2),
        num_sensors_(num_sensors),
        Loop(1e4 /* 100Hz */),
        motorspeed_(120),
        maxspeed_(180) {
    left_.attach(left);
    right_.attach(right);
  }

  void calibrate() {
    pinMode(13, OUTPUT);
    digitalWrite(
        13,
        HIGH);  // turn on Arduino's LED to indicate we are in calibration mode
    for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
    {
      qtrrc.calibrate();  // reads all sensors 10 times at 2500 us per read
                          // (i.e. ~25 ms per call)
    }
    digitalWrite(13, LOW);  // turn off Arduino's LED to indicate we are through
                            // with calibration
  }

  void set_pid(float p, float i, float d) {
    p_ = p;
    i_ = i;
    d_ = d;
  }

  unsigned *sensors() {
    unsigned values[num_sensors_];
    qtrrc.readCalibrated(values);
    return values;
  }

 private:
  uint8_t calc() {
    int error = setpoint_ - read();
    sum_ += error;
    uint8_t retval = p_ * error + i_ * sum_ + d_ * (error - prev_error_);
    prev_error_ = error;
    return retval;
  }

  void write(uint8_t pidout) {
    uint8_t maxpid = maxspeed_ - motorspeed_;
    if (pidout > maxpid) pidout = maxpid;
    else if (pidout < -maxpid) pidout = -maxpid;
    uint8_t left = motorspeed_ + pidout;
    uint8_t right = motorspeed_ - pidout;
    left_.write(left);
    right_.write(right);
  }

  int read() {
    return qtrrc.readLine(new unsigned[num_sensors_]);
  }

  uint8_t motorspeed_;
  uint8_t maxspeed_;
  int setpoint_;
  int prev_error_;
  long sum_;
  float p_, i_, d_;

  uint8_t num_sensors_;

  QTRSensorsRC qtrrc;
  Servo left_, right_;
};
#endif  // __LINEFOLLOWER_H__
