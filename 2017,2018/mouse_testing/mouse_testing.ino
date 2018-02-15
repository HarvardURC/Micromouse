#include "config.h"
#include "io.hh"
#include "motors.hh"
#include "sensors.hh"

using namespace pins;

Driver* driver;
Motor* motor;
SensorArray* sensorArr;
Buzzer* buzz;
int flag = 0;

void setup() {
    Serial.begin(9600);
    delay(1000);

    sensorArr = new SensorArray(
      tofLeftDiagS,
      tofRightDiagS,
      tofFrontS,
      tofFrontL,
      imuRST);

    // driver = new Driver(
    //   motorPowerL,
    //   motorDirectionL,
    //   motorPowerR,
    //   motorDirectionR,
    //   motorMode,
    //   encoderL1,
    //   encoderL2,
    //   encoderR1,
    //   encoderR2,
    //   *sensorArr);
    motor = new Motor(
        motorPowerR,
        motorDirectionR,
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
      // driver->moveTicksPID(1000000);
      // motor->setpoint = 100000;
      // motor->drive(20);
      motor->testPID();
      Serial.println("FIN");
      flag = 1;
    }
    // Serial.println(motor->getPIDSpeed());
    // motor->moveTicksPID(1000000);

    // Serial.println(motor->moveTicksPID(100000));
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
