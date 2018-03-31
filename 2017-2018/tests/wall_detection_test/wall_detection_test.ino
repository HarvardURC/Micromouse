#include <vector>
#include "config.h"
#include "sensors.hh"
#include "io.hh"

using namespace pins;

SensorArray* sensorArr;
RGB_LED* backRgb;

int thresholds[3] = {60, 120, 60};
std::vector<String> names = {"leftDiag", "front", "rightDiag"};

void setup() {
    Serial.begin(9600);
    delay(500);

    sensorArr = new SensorArray(
        tofLeftDiagS,
        tofRightDiagS,
        tofFrontS,
        tofFrontL,
        imuRST);
}

void loop() {
    Serial.print("Walls:");
    for (int i = 0; i < 3; i++) {
        if (sensorArr->readShortTof(i) < thresholds[i]) {
            Serial.print(" ");
            Serial.print(names[i]);
            Serial.print(" ");
        }
    }
    Serial.println("");
    delay(10);
}
