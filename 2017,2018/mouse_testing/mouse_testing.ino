#include "config.h"
#include "rgb.h"

using namespace pins;

RGB_LED* rgbFront;
RGB_LED* rgbBack;

void setup() {
    rgbFront = new RGB_LED(frontLedR, frontLedG, frontLedB);
    rgbBack = new RGB_LED(backLedR, backLedG, backLedR);
}

void loop() {
    rgbBack->switchLED(0);
}
