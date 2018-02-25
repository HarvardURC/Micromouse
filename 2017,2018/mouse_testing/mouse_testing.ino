#include "config.h"
#include "io.hh"
#include "motors.hh"
#include "sensors.hh"
#include "bluetooth.hh"

using namespace pins;

Driver* driver;
SensorArray* sensorArr;
Buzzer* buzz;
int flag = 0;

void setup() {
  Serial.begin(9600);
  delay(500);

  sensorArr = new SensorArray(
    tofLeftDiagS,
    tofRightDiagS,
    tofFrontS,
    tofFrontL,
    imuRST);

  driver = new Driver(
    motorPowerL,
    motorDirectionL,
    motorPowerR,
    motorDirectionR,
    motorMode,
    encoderL1,
    encoderL2,
    encoderR1,
    encoderR2,
    *sensorArr);

  buzz = new Buzzer(buzzer);

  pinMode(motorMode, OUTPUT);
  digitalWrite(motorMode, HIGH);
}

void loop() {
  if (flag == 0) {
    /* PID movement testing */
    driver->movePID(1000000);
    flag = 1;
  }
}
