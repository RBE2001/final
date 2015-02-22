/**
 * The Bluetooth class performs all the functions needed to run bluetooth on an
 * RBE2001 robot. This inherits from the Loop class so that it can run at a
 * moderately consistent interval.
 */
#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__
#include "Arduino.h"
#include <BluetoothClient.h>
#include <BluetoothMaster.h>
#include <ReactorProtocol.h>
#include "loop.h"

class Bluetooth : public Loop {
 public:
  // Various enums for creating message packets.

  // Type of message being sent.
  enum MsgType {
    kStorage = 0x01,
    kSupply = 0x02,
    kRadiation = 0x03,
    kStop = 0x04,
    kResume = 0x05,
    kStatus = 0x06,
    kHeart = 0x07,
  };

  // Destination possibilities (will always be kBroadcast).
  enum Dest { kBroadcast = 0x00, };

  // Radiation level of the current robot. kNone results in no radiation message
  // being broadcast.
  enum RadLevel { kNone = 0x00, kSpent = 0x2C, kNew = 0xFF, };

  // Movement Status enum.
  enum Movement { kStopped = 0x01, kTeleop = 0x02, kAuto = 0x03, };

  // Gripper Status enum.
  enum Gripper { kNoRod = 0x01, kHaveRod = 0x02, };

  // Operation Status enum.
  enum Operation {
    kGripAttempt = 0x01,
    kGripRelease = 0x02,
    kToReactor = 0x03,
    kToStorage = 0x04,
    kToSupply = 0x05,
    kIdle = 0x06,
  };

  // struct containing full status message.
  // Note: Consider defining all enums as inheriting from uint8_t and then
  // creating a union of a Status struct with an array of three uint8_t to be
  // sent directly in the status message.
  struct Status {
    Movement move;
    Gripper grip;
    Operation op;
  };

  // Declare a Bluetooth object with a specific robot number. Also, defines this
  // as running in a 25Hz loop.
  Bluetooth(uint8_t rnum = 8)
      : rnum_(rnum),
        pcol_(rnum),
        Loop(4e4 /* 40ms */),
        nexthb_(0),
        nextrad_(0),
        nextst_(0),
        stopped_(false),
        rad_level_(kNone) {
    state_.move = kStopped;
    state_.grip = kNoRod;
    state_.op = kIdle;
    // Initialize bluetooth.
    pinMode(14, INPUT_PULLUP);
    pinMode(15, INPUT_PULLUP);
    Serial3.begin(115200);
  }

  // Run function called by Loop::Update. Will send out heartbeat, radiation
  // alerts, status message, and read anything new.
  void Run();

  void set_radlevel(RadLevel radlevel) { rad_level_ = radlevel; }

  void set_status(Status state) { state_ = state; }

  bool stopped() { return stopped_; }

  // These return the current states of the supply and storage tubes as one-byte
  // bitmasks where the 4 least significant bits each refer to the occupation
  // state of a single tube.
  uint8_t raw_supply() { return supply_; }
  uint8_t raw_storage() { return storage_; }

  // These return true if and only if the specified tube (0 - 3) is currently
  // occupied.

  bool supply(uint8_t tube /* 0, 1, 2, or 3 */) {
    return (supply_ >> tube) & 0x01;
  }

  bool storage(uint8_t tube /* 0, 1, 2, or 3 */) {
    return (storage_ >> tube) & 0x01;
  }

  Status status() { return state_; }

 private:
  // Note: All of the Send* functions increment counters (next*_) to indicate
  // when they should next be called. However, they will not prevent you from
  // calling them too early, so be sure to include some sort of if statement
  // around them to prevent them from being called too quickly.

  // The send heartbeat message does the actual work of sending a heartbeat
  // message and increments the nexthb_ appropriately.
  void SendHB();

  // Sends out a radiation alert with the provided signal (low, high, or no
  // alert). Properly handles all cases.
  void SendRad(RadLevel rad_level);

  // Sends out a robot status message with the provided Status.
  void SendStatus(Status state);

  // Reads from the bluetooth attachment and updates any data that needs updating.
  // Returns true if a relevant packet was received, false otherwise.
  bool Read();

  // times at which the heratbeat, radiation, and status functions should next
  // be called, in milliseconds since the start of the program.
  unsigned long nexthb_;
  unsigned long nextrad_;
  unsigned long nextst_;

  // Number of the current robot.
  const uint8_t rnum_;

  // Provided library classes for interacting with bluetooth module.
  ReactorProtocol pcol_;
  BluetoothMaster btmaster_;

  // Current status information:
  // Current radiation alert level.
  RadLevel rad_level_;
  // Bitmasks of storage and supply tube availability.
  uint8_t storage_;
  uint8_t supply_;
  // true if the master has commanded us to stop; false otherwise.
  bool stopped_;
  // Current information to be sent in next status message.
  Status state_;
};
#endif  //  __BLUETOOTH_H__
