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
    // float left_front_dist = sensorArr->readShortTof(LEFTFRONT);
    // float right_front_dist = sensorArr->readShortTof(RIGHTFRONT);
    // float front_diff = left_front_dist - right_front_dist - 2;
    // float rangefinder_angle = atan2f(front_diff * 2 , 52.); // rangefinders 52 mm apart
    // Serial.println(rangefinder_angle);

    delay(10);
}
