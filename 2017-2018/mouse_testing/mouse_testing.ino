#include "config.h"
#include "bluetooth.hh"
#include "helpers.hh"
#include "io.hh"
#include "motors.hh"
#include "sensors.hh"

using namespace pins;

/* Robot hardware abstractions */
Driver* driver;
SensorArray* sensorArr;
Buzzer* buzz;
Button* backButt;
Button* frontButt;
RGB_LED* backRgb;
RGB_LED* frontRgb;

/* Global vars */
int test_num = 0;
int test_level = 0;
bool debug = true;
bool bluetooth = false; // allows operator to set test_level with bluetooth
const char init_command = 'a';
char command = init_command; // holds commands from bluetooth


void setup() {
    Serial.begin(9600);
    delay(500);
    if (bluetooth) {
        bluetoothInitialize();
    }


    backRgb = new RGB_LED(backLedR, backLedG, backLedB);
    backRgb->flashLED(0);


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
    backButt = new Button(backButton);
    frontButt = new Button(frontButton);
    backRgb = new RGB_LED(backLedR, backLedG, backLedB);
    frontRgb = new RGB_LED(frontLedR, frontLedG, frontLedB);

    pinMode(motorMode, OUTPUT);
    digitalWrite(motorMode, HIGH);

    backRgb->flashLED(1);
    frontRgb->switchLED(2);

    adjustTestLevel();
}

void loop() {
    frontRgb->switchLED(2);
    switch(test_level) {
        case 0:
            switch(test_num) {
                // LEVEL 1 TESTS
                case 0:
                    if (debug) debug_println("10cm forward");
                    driver->forward(18);
                    break;
                case 1:
                    if (debug) debug_println("30cm forward");
                    driver->forward(30);
                    break;
                case 2:
                    if (debug) debug_println("90 degrees left");
                    driver->turnLeft(90);
                    break;
                case 3:
                    if (debug) debug_println("90 degree right");
                    driver->turnRight(90);
                    break;
                case 4:
                    if (debug) debug_println("180 degree turn");
                    driver->turnLeft(180);
                    break;
                case 5:
                    if (debug) debug_println("360 degree turn");
                    driver->turnRight(360);
                    break;
                default:
                    buzz->siren();
                    goNextTests();
                    break;
            }
            break;
        case 1:
            switch(test_num) {
                // LEVEL 2 TESTS
                case 0:
                    if (debug) debug_println("Turn left then go forward.");
                    driver->turnLeft(90);
                    driver->forward(10);
                    break;
                case 1:
                    if (debug) debug_println("Turn right 45 deg then forward.");
                    driver->turnRight(45);
                    driver->forward(10);
                    break;
                default:
                    buzz->siren();
                    goNextTests();
                    break;
            }
            break;
        case 2:
            switch(test_num) {
                // LEVEL 3 TESTS
                case 0:
                    // Back and forth
                    if (debug) debug_println("Back and forth");
                    driver->forward(20);
                    driver->turnLeft(180);
                    driver->forward(20);
                    driver->turnRight(180);
                    break;
                default:
                    buzz->siren();
                    goNextTests();
                    break;
            }
            break;
        case 3:
            switch(test_num) {
                // TANK MOVEMENT TEST
                case 0:
                    if (debug) debug_println("Going...");
                    driver->forward(5.5);
                    driver->resetState();

                    driver->tankGo(0, 18);
                    driver->tankGo(18, 18);
                    driver->tankGo(18, 36);
                    driver->tankGo(0, 36);

                    driver->tankGo(0, 18);
                    driver->tankGo(-18, 18);
                    driver->tankGo(-18, 36);
                    driver->tankGo(0, 36);
                    driver->tankGo(0, 0);
                default:
                    buzz->siren();
                    goNextTests();
                    break;
            }
            break;
        case 4:
            switch(test_num) {
                // REALIGN TEST
                case 0: {
                    if (debug) debug_println("Realign test");
                    driver->realign(20);
                    break;
                }
                default:
                    buzz->siren();
                    break;
            }
            break;
        case 5:
            switch(test_num) {
                // Turning Testing
                case 0: {
                    if (debug) debug_println("5 Left tests");
                    for (int i = 0; i < 5; i++) {
                        driver->turnLeft(90);
                        backRgb->flashLED(2);
                        delay(500);
                    }
                    break;
                }
                case 1: {
                    if (debug) debug_println("5 Right tests");
                    for (int i = 0; i < 5; i++) {
                        driver->turnRight(90);
                        backRgb->flashLED(2);
                        delay(500);
                    }
                    break;
                }
                case 2: {
                    for (int i = 0; i < 3; i ++) {
                        driver->turnRight(90);
                        backRgb->flashLED(2);
                        delay(500);
                    }
                    driver->forward(20);
                    break;
                }
                default:
                    buzz->siren();
                    break;
            }
        default:
            break;

    }
    frontRgb->switchLED(2);
    driver->resetState();
    backRgb->flashLED(0);
    // if (bluetooth && command == init_command) { waitCommand(); }
    // else { waitButton(); }
    waitButton();
}



void goNextTests() {
    test_level++;
    test_num = 0;
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


/* Waits on a command from bluetooth controller */
void waitCommand() {
    while (1) {
        /*while (ble.available()) {
            command = ble.read();
            break;
        }*/
        if (command != init_command) {
            test_level = (int)command - '0';

            frontRgb->flashLED(2);
            delay(1000);
            frontRgb->flashLED(1);
            break;
        }
    }
}


/* Flashes the front LED green if the condition was true else red */
void resultLed(bool success) {
    if (success) {
        backRgb->flashLED(1);
    }
    else {
        backRgb->flashLED(0);
    }
}
