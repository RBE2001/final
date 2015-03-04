/**
 * The LineFollower class is intended to use a QTRRC light sensor array from
 * Pololu to follow a line. The LineFollower class takes control fot the sensor
 * and the drive motors when running. Inherits from Loop so that it can run a
 * PID controller on the drive motors at a consistent frequency.
 */
#ifndef __LINEFOLLOWER_H__
#define __LINEFOLLOWER_H__
#include <Servo.h>
#include <QTRSensors.h>
#include "Arduino.h"
#include "loop.h"

// LineFollower class. See description at start of file.
// In order to actually follow the line, a PID controller calculates a
// correction value using the current position of the sensor over the line and
// then takes that value and adds it to the normal speed of one motor and
// subtracts it from the normal speed of the other motor, causing the robot to
// go straight if it si right over the line, and turn otherwise.
class LineFollower : public Loop {
 public:
  // Constructor takes array of digital input pins, the number of pins, and the
  // left and right motor ports.
  LineFollower(unsigned char *pins, unsigned char num_sensors, uint8_t left,
               uint8_t right, bool linverted=false, bool rinverted=false)
      : qtrrc(pins, num_sensors),
        p_(0),
        i_(0),
        d_(0),
        prev_error_(0),
        sum_(0),
        setpoint_(3500),
        num_sensors_(num_sensors),
        Loop(50000UL /* 20Hz */),
        motorspeed_(103),
        maxspeed_(116),
        linv_(linverted),
        rinv_(rinverted),
        backwards_(false),
        enable_outputs_(true) {
    left_ = new Servo;
    right_ = new Servo;
    left_->attach(left);
    right_->attach(right);
  }

  // Takes ~10 seconds to calibrate line sensor array. Be sure to move all of
  // the sensors over both the lightest and darkest colors that they will
  // observe.
  // Turns arduino board LED (DIO pin 13) on to indicate calibration is
  // occurring.
  void Calibrate();

  // Set forwards line tracking PID values.
  void set_pid(float p, float i, float d) {
    p_ = p;
    i_ = i;
    d_ = d;
  }

  // Set backwards line tracking PID values.
  void set_back_pid(float p, float i, float d) {
    bp_ = p;
    bi_ = i;
    bd_ = d;
  }

  // Called at 20Hz; updates motors and line following.
  void Run() { Write(Calc()); }

  // Returns a num_sensors length array of sensor values, normalized to be from
  // 0 - 1000. 1000 = darkest, 0 = lightest.
  unsigned *sensors();

  // Returns pointers to the left and write motors so that other functions can
  // access the motors. Be sure to call enable_outputs(false) before using the
  // Servos in external code.
  Servo *left() { return left_; }
  Servo *right() { return right_; }

  // Reads the current position of the sensor over the line.
  int Read() { return qtrrc.readLine(sensor_buffer); }

  // Reverses the direction of the robot.
  void reverse(bool backwards) { backwards_ = backwards; }

  // Enables/disables motor outputs while continuing to update PID/line sensor
  // values.
  void enable_outputs(bool enable) { enable_outputs_ = enable; }

 private:
  // Performs actual PID calculation.
  int8_t Calc();

  // Uses the output from the Calc function to determine motor speeds and write
  // them.
  void Write(int8_t pidout);

  // Buffer for sensor values; 32 is an arbitrary number, chosen to ensure
  // sufficient space for an arbitrarily sized sensor array.
  unsigned sensor_buffer[32];

  // Typical motorspeed when going straight.
  uint8_t motorspeed_;
  // Maximum allowed speed.
  uint8_t maxspeed_;
  // Goal--generally centers robot on line.
  int setpoint_;
  // PID variables.
  int prev_error_;
  long sum_;
  // PID gains for forwards and backwards.
  float p_, i_, d_;
  float bp_, bi_, bd_; // Backwards numbers.

  // Number of individual sensors on array.
  uint8_t num_sensors_;

  // Object for sensor array.
  QTRSensorsRC qtrrc;
  // Left and right drivetrain motors.
  Servo *left_, *right_;

  // Flags to indicate directionality of motors.
  bool linv_, rinv_;

  // Whether the robot is currently running forwards/backwards.
  bool backwards_;

  // Whether to actually write values out to the motors.
  bool enable_outputs_;
};
#endif  // __LINEFOLLOWER_H__
