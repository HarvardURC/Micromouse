#include "config.h"
#include "io.hh"
#include "motors.hh"
#include "sensors.hh"

using namespace pins;

Driver* driver;
// Motor* motor;
SensorArray* sensorArr;
Buzzer* buzz;
int flag = 0;

void setup() {
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
  // motor = new Motor(
  //     motorPowerL,
  //     motorDirectionL,
  //     encoderL1,
  //     encoderL2);

  buzz = new Buzzer(buzzer);
}

void loop() {
  if (flag == 0) {
    /* PID movement testing */
    driver->moveTicksPID(1000000);
    Serial.println("FIN");
    flag = 1;
  }
  /* Tick reading testing */
  // while (1) {
  //   Serial.println(motor->readTicks());
  //   delay(1000);

  // }

  /* Move ticks testing */
  // motor->moveTicksPID(-2000000);

  /* Driver moving testing */
  // driver->drive(40);
  // delay(2000);
  // driver->drive(0);
  // delay(2000);
  // driver->drive(-40);
  // delay(2000);
  // driver->drive(0);
  // delay(2000);
}
