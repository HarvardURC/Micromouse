/*
Test for checking if deploying to Teensy board is working.
Should cause a blinking orange LED on the corner of the Teensy board.
*/

#include "config.h"
#include "io.hh"

using namespace pins;

RGB_LED* rgbFront;
RGB_LED* rgbBack;

void setup() {
    rgbFront = new RGB_LED(frontLedR, frontLedG, frontLedB);
    rgbBack = new RGB_LED(backLedR, backLedG, backLedR);
}

void loop() {
    rgbBack->switchLED(0);
    delay(1000);
    rgbBack->switchLED(0);
    rgbFront->flashLED(1);
    delay(1000);
}
