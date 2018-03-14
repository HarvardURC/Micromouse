/**
  A time of flight (ToF) sensor test.
  Outputs to Serial console the distance measured
  by the left and right sensors.
*/

#include <vector>
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
    unsigned long start = millis();
    sensorArr->readToSerialTof();
    delay(10);
}
