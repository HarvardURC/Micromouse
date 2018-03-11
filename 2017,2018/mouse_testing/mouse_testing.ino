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
int test_num = 2;
int test_level = 0;
bool debug = true;


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
                    if (debug) Serial.println("10cm forward");
                    driver->forward(10);
                    break;
                case 1:
                    if (debug) Serial.println("30cm forward");
                    driver->forward(30);
                    break;
                case 2:
                    if (debug) Serial.println("90 degrees left");
                    driver->turnLeft(90);
                    break;
                case 3:
                    if (debug) Serial.println("90 degree right");
                    driver->turnRight(90);
                    break;
                case 4:
                    if (debug) Serial.println("180 degree turn");
                    driver->turnLeft(180);
                    break;
                case 5:
                    if (debug) Serial.println("360 degree turn");
                    driver->turnRight(360);
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
                    if (debug) Serial.println("Turn left then go forward.");
                    driver->turnLeft(90);
                    driver->forward(10);
                    break;
                case 1:
                    if (debug) Serial.println("Turn right 45 deg then forward.");
                    driver->turnRight(45);
                    driver->forward(10);
                    break;
                default:
                    break;
            }
            break;
        case 2:
            switch(test_num) {
                // LEVEL 3 TESTS
                case 0:
                    // Back and forth
                    if (debug) Serial.println("Back and forth");
                    driver->forward(20);
                    driver->turnLeft(180);
                    driver->forward(20);
                    driver->turnRight(180);
                    break;
                default:
                    break;
            }
            break;
        case 3:
            switch(test_num) {
                case 0:
                    Serial.println("Going...");
                    driver->tankGo(0, 10, 0);
                    break;
                default:
                    break;
            }
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
            delay(500);
        }
    }
    delay(1000);
}
