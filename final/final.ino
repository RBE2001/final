// Currently contains code for testing bluetooth code on field-provided robot.
#include <Servo.h>
#include <QTRSensors.h>
#include <BluetoothClient.h>
#include <BluetoothMaster.h>
#include <ReactorProtocol.h>
#include "armpid.h"
#include "linefollower.h"
#include "bluetooth.h"

const uint8_t lmotor = 10;
const uint8_t rmotor = 11;
const bool linverted = false;
const bool rinverted = true;
const uint8_t team = 8;

const uint8_t button = 3;
const unsigned char num_sensors = 8;
unsigned char linepins[num_sensors] = {46, 44, 42, 40, 38, 36, 34, 32};
Bluetooth *bt;
LineFollower *lf;

volatile bool go = false;
volatile uint8_t radlevel = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Hello.");
  bt = new Bluetooth(team);
  lf = new LineFollower(linepins, num_sensors, lmotor, rmotor, linverted, rinverted);
  Serial.println("Done constructing.");


  pinMode(button, INPUT_PULLUP);
  while (digitalRead(button));
  Serial.println("Calibrating.");
  lf->Calibrate();
  Serial.println("Done Calibrating.");

  lf->set_pid(1e-2, 0.0, 0.0);

  attachInterrupt(1 /*DIO 3*/, goButton, RISING);
}

void goButton() {
  //go = true;
}

void loop() {
  if (go) bt->Update();
  lf->Update();
}
