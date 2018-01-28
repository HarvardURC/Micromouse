/*
Test for checking if deploying to Teensy board is working.
Should cause a blinking orange LED on the corner of the Teensy board.
*/

#include "config.h"

void setup() {
    pinMode(pins::backLedR, OUTPUT);
    pinMode(pins::backLedG, OUTPUT);
    pinMode(pins::backLedB, OUTPUT);
    digitalWrite(pins::backLedR, HIGH);
    digitalWrite(pins::backLedG, HIGH);
    digitalWrite(pins::backLedB, HIGH);
}

void loop() {
    digitalWrite(pins::backLedR, LOW);   // sets the LED on
    delay(200);                  // waits for a second
    digitalWrite(pins::backLedR, HIGH);
    delay(500);
    digitalWrite(pins::backLedG, LOW);    // sets the LED off
    delay(200);                  // waits for a second
    digitalWrite(pins::backLedG, HIGH); 
    delay(500);
    digitalWrite(pins::backLedB, LOW); 
    delay(200);
    digitalWrite(pins::backLedB, HIGH); 
    delay(500);
}
