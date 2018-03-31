/**
  A time of flight (ToF) sensor test.
  Outputs to Serial console the distance measured
  by the ToF sensors.
*/

#include "config.h"
#include "sensors.hh"

using namespace pins;

SensorArray* sensorArr;


void setup() {
    sensorArr = new SensorArray(
        tofLeftDiagS,
        tofRightDiagS,
        tofFrontS,
        tofFrontL,
        imuRST);
}

void loop() {
    sensorArr->readToSerialTof();
    delay(10);
}
