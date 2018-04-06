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
        tofDiagL,
        tofFrontL,
        tofFrontR,
        tofDiagR,
        imuRST);
}

void loop() {
    sensorArr->readToSerialTof();
    /* Competition Calibration:
     *  tof_low_bound, tof_high_bound
     *  front_wall_threshold
     *  wall_follow_dist
     *  Also:
     *  IRThresholds in Maze::addWalls
     */
    delay(10);
}
