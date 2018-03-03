#include "config.h"
#include "io.hh"
#include "motors.hh"
#include "sensors.hh"
#include "bluetooth.hh"

using namespace pins;

Driver* driver;
SensorArray* sensorArr;
Buzzer* buzz;
Button* backButt;
Button* frontButt;
RGB_LED* backRgb;
RGB_LED* frontRgb;
int test_num = 0;
int test_level = 0;
int x = 60;
int y = 64;

void waitButton();
void adjustTestLevel();

void setup() {
    Serial.begin(9600);
    delay(500);

    // sensorArr = new SensorArray(
    //   tofLeftDiagS,
    //   tofRightDiagS,
    //   tofFrontS,
    //   tofFrontL,
    //   imuRST);

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
    backButt = new Button(backButton);
    frontButt = new Button(frontButton);
    backRgb = new RGB_LED(backLedR, backLedG, backLedB);
    frontRgb = new RGB_LED(frontLedR, frontLedG, frontLedB);
    frontRgb->switchLED(2);

    pinMode(motorMode, OUTPUT);
    digitalWrite(motorMode, HIGH);

    adjustTestLevel();
}

void loop() {
    frontRgb->switchLED(2);
    switch(test_level) {
        case 0:
            switch(test_num) {
                // LEVEL 1 TESTS
                case 0:
                    // 10cm forward
                    driver->go(10, 0, 0);
                    break;
                case 1:
                    // 30cm forward
                    driver->go(30, 0, 0);
                    break;
                case 2:
                    // 90 degrees left
                    driver->go(0, 0, 3.14 / 2);
                    break;
                case 3:
                    // 90 degree right
                    driver->go(0, 0, -1 * 3.14 / 2);
                    break;
                case 4:
                    // 180 turn
                    driver->go(0, 0, 3.14);
                    break;
                case 5:
                    // 360 turn
                    driver->go(0, 0, 2*3.14);
                    break;
                default:
                    // nothing
                    break;
            }
            break;
        case 1:
            switch(test_num) {
                // LEVEL 2 TESTS
                case 0:
                    driver->go(0, 0, 3.14 / 2);
                    driver->go(0, 10, 3.14 / 2);
                case 1:
                    driver->go(0, 0, 3.14 / 4);
                    driver->go(10, 10, 3.14 / 4);
                default:
                    break;
            }
            break;
        case 2:
            switch(test_num) {
                // LEVEL 3 TESTS
                case 0:
                    // Back and forth
                    driver->go(10, 0, 0);
                    driver->go(10, 0, 3.14);
                    driver->go(0, 0, 3.14);
                    driver->go(0, 0, 0);
                default:
                    break;
            }
            break;
        default:
            break;

    }
    frontRgb->switchLED(2);
    driver->resetState();
    backRgb->flashLED(0);
    waitButton();
}

// If the back button is pressed, go to next test case
// If front button pressed, repeat test case
void waitButton() {
    while (1) {
        if (backButt->read() == LOW) {
            test_num += 1;
            break;
        }
        if (frontButt->read() == LOW) {
            break;
        }
    }
    delay(1000);
}

// Increments the testing level if the front button is pressed
void adjustTestLevel() {
    while (backButt->read() != LOW) {
        if (frontButt->read() == LOW) {
            test_level += 1;
            backRgb->flashLED(1);
            delay(1000);
        }
    }
    delay(1000);
}
