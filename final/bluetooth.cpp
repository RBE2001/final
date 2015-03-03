#include "bluetooth.h"

#include "Arduino.h"
#include <BluetoothClient.h>
#include <BluetoothMaster.h>
#include <ReactorProtocol.h>
#include "loop.h"

// Run function called by Loop::Update. Will send out heartbeat, radiation
// alerts, status message, and read anything new.
void Bluetooth::Run() {

  // Read any new messages--Returns true if there are any new message, false
  // otherwise.
  Read();

  if (supply_ == 0xFF) return; // Don't send new messages until we get a status.

  // Send out heartbeat if we are due to.
  if (millis() > nexthb_)
    SendHB();

  // Send out radiation alert if we are due to and if the current radiation
  // level necessitates it.
  if ((millis() > nextrad_) && rad_level_)
    SendRad(rad_level_);

  // Send out status message if appropriate time has passed.
  if ((millis() > nextst_))
    SendStatus(state_);

  if (rad_level_ != kNone) FlagWave();
}

// Sends out heartbeat message and increments nexthb_.
// Note that this function does not prevent you from sending a hearbeat too
// often; it just provides a convenient indicater (nexthb_) for when it should
// be called next.
void Bluetooth::SendHB() {
  // Containers for packet information.
  uint8_t pkt[10];
  uint8_t sz;
  uint8_t data[3];
  // Call again in 1 - 2 seconds.
  nexthb_ = millis() + 1500;
  // Create and send packet using provided utilities.
  pcol_.setDst((uint8_t)kBroadcast);
  sz = pcol_.createPkt((uint8_t)kHeart, data, pkt);
  btmaster_.sendPkt(pkt, sz);
}

// Sends out a radiation alert. Will not send out the alert of rad_level =
// kNone.
void Bluetooth::SendRad(RadLevel rad_level) {
  // Don't send a message of there is no alery.
  if (rad_level == kNone) return;
  // Containers for packet information.
  uint8_t pkt[10];
  uint8_t sz;
  uint8_t data[3];
  // Call again in 1 - 5 seconds.
  nextrad_ = millis() + 1900;
  // Put together packet.
  pcol_.setDst((uint8_t)kBroadcast);
  data[0] = (uint8_t)rad_level;
  sz = pcol_.createPkt((uint8_t)kRadiation, data, pkt);
  btmaster_.sendPkt(pkt, sz);
}

// Sends out a Status message.
void Bluetooth::SendStatus(Status state) {
  // Containers for packet information.
  uint8_t pkt[10];
  uint8_t sz;
  uint8_t data[3];
  // Call agian in >5 sec.
  nextst_ = millis() + 5130;
  // Set up packet and pack in the status data.
  pcol_.setDst((uint8_t)kBroadcast);
  data[0] = (uint8_t)state.move;
  data[1] = (uint8_t)state.grip;
  data[2] = (uint8_t)state.op;
  sz = pcol_.createPkt((uint8_t)kStatus, data, pkt);
  btmaster_.sendPkt(pkt, sz);
}

// Read data from stream and update member variables.
bool Bluetooth::Read() {
  // Containers for packet information.
  uint8_t pkt[10];
  uint8_t data[3];
  uint8_t type;

  // Read a packet from the stream; if no packet available, falls through.
  if (btmaster_.readPacket(pkt)) {
    // Process packet; if packet is invalid (eg, bad checksum), falls through.
    if (pcol_.getData(pkt, data, type)) {
      // Checks that the packet is addressed either to us or to everyone.
      if (pkt[4] != kBroadcast && pkt[4] != rnum_)
        return false;
      // Update status variables depending on type of message.
      switch ((MsgType)type) {
        case kStorage:
          storage_ = data[0];
          break;
        case kSupply:
          supply_ = data[0];
          break;
        case kStop:
          stopped_ = true;
          break;
        case kResume:
          stopped_ = false;
          break;
        default:
          return false;
      }
      return true;
    }
  }
  return false;
}

void Bluetooth::ChangeDegree() {
  if (degreeflag_ = 180) (degreeflag_ = 0);
  if (degreeflag_ = 0) (degreeflag_ = 180);

  nextwave_ = millis() + kWaveInterval;
}

void Bluetooth::FlagNull() {
  flagservo_.write(90);
  waving_ = false;
}

void Bluetooth::FlagWave() {
  if (waving_ = false) {
    flagservo_.write(degreeflag_);
    waving_ = true;
    ChangeDegree();
    nextwave_ = millis() + kWaveInterval;
  }
  if (waving_ = true) {
    if (millis() >= nextwave_) {
      flagservo_.write(degreeflag_);
      ChangeDegree();
    }
    if (millis() < nextwave_) {
      flagservo_.write(degreeflag_);
    }
  }
}
