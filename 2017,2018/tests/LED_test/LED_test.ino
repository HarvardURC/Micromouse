/*
Test for checking if deploying to Teensy board is working.
Should cause a blinking orange LED on the corner of the Teensy board.
*/

#include "config.h"

void setup() {
    pinMode(pins::cpuLed, OUTPUT);
}

void loop() {
    digitalWrite(pins::cpuLed, HIGH);   // sets the LED on
    delay(1000);                  // waits for a second
    digitalWrite(pins::cpuLed, LOW);    // sets the LED off
    delay(1000);                  // waits for a second
}