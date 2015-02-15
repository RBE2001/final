// Currently contains code for testing bluetooth code on field-provided robot.
#include <Servo.h>
#include <QTRSensors.h>
#include <BluetoothClient.h>
#include <BluetoothMaster.h>
#include <ReactorProtocol.h>
#include "armpid.h"
#include "linefollower.h"
#include "bluetooth.h"

#define goLED       2
#define goSW        3
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

volatile bool go = false;
volatile uint8_t radlevel = 0;

void setup() {
  Serial.begin(115200);
  bt = new Bluetooth(21);

  // set up the onboard and 'spare' LEDs and init them to the off state
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

void extint_1ISR(void) {
  go = true;
  radlevel = (radlevel + 1) % 3;
  switch (radlevel) {
    case 0:
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
