/**
 * This file is intended to be downloaded onto the provided test robot on the
 * RBE2001 field and tests that the bluetooth code that we created works as
 * expected.
 * Note: In order to include bluetooth.h properly, you may need to do some
 * wrangling with your compiler and may just need to copy bluetooth.h into the
 * same directory as this file.
 */
#include <Servo.h>
#include <QTRSensors.h>
#include <BluetoothClient.h>
#include <BluetoothMaster.h>
#include <ReactorProtocol.h>
#include "bluetooth.h"

// Input port for button.
#define goSW        3
// Ports for various indicator LEDs.
#define goLED       2
#define spareLED   23
#define storLED1  46
#define storLED2  48
#define storLED4  50
#define storLED8  52
#define splyLED1  47
#define splyLED2  49
#define splyLED4  51
#define splyLED8  53

Bluetooth *bt;

// go indicates that bluetooth has connected and packets can now be sent.
volatile bool go = false;
// rad level refers to the radiation level to be sent out (none=0, low=1, or
// high=2). After the go button has been pushed initially, this will cycle
// between the three options when you press the goSW button.
volatile uint8_t radlevel = 0;

void setup() {
  // For serial port debugging.
  Serial.begin(115200);
  // Create bluetooth object for robot #21 (field test robot).
  bt = new Bluetooth(21);

  // set up the 'spare' LED and init it to the off state
  pinMode(spareLED, OUTPUT);
  digitalWrite(spareLED, LOW);

  // set up the received packet data LEDs and init them to the off state
  pinMode(splyLED1, OUTPUT);
  digitalWrite(splyLED1, LOW);
  pinMode(splyLED2, OUTPUT);
  digitalWrite(splyLED2, LOW);
  pinMode(splyLED4, OUTPUT);
  digitalWrite(splyLED4, LOW);
  pinMode(splyLED8, OUTPUT);
  digitalWrite(splyLED8, LOW);
  pinMode(storLED1, OUTPUT);
  digitalWrite(storLED1, LOW);
  pinMode(storLED2, OUTPUT);
  digitalWrite(storLED2, LOW);
  pinMode(storLED4, OUTPUT);
  digitalWrite(storLED4, LOW);
  pinMode(storLED8, OUTPUT);
  digitalWrite(storLED8, LOW);

  // set up the GO button, the GO LED, and the external interrupt for the button
  pinMode(goLED, OUTPUT);
  digitalWrite(goLED, LOW);
  pinMode(goSW, INPUT_PULLUP);
  attachInterrupt(1, extint_1ISR, FALLING);

}

// Triggers whenever the goSW button is pressed (but not when it is released).
// Sets go to true and increments radiaiton level.
void extint_1ISR(void) {
  go = true;
  radlevel = (radlevel + 1) % 3;
  switch (radlevel) {
    case 0:
      // The set_radlevel function just performs a copy operation.
      bt->set_radlevel(Bluetooth::kNone);
      break;
    case 1:
      bt->set_radlevel(Bluetooth::kSpent);
      break;
    case 2:
      bt->set_radlevel(Bluetooth::kNew);
      break;
  }
}

void loop() {
  // Don't send mess with bluetooth until go button has been pressed once.
  if (go) bt->Update();
  digitalWrite(splyLED1, bt->supply(0));
  digitalWrite(splyLED2, bt->supply(1));
  digitalWrite(splyLED4, bt->supply(2));
  digitalWrite(splyLED8, bt->supply(3));
  digitalWrite(storLED1, bt->storage(0));
  digitalWrite(storLED2, bt->storage(1));
  digitalWrite(storLED4, bt->storage(2));
  digitalWrite(storLED8, bt->storage(3));
  digitalWrite(goLED, bt->stopped());
}
