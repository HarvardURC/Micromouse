/*
Tests for functional buttons by lighting up back led different colors.
*/

// TODO: Use <Bounce.h> library for debouncing switches

#include "config.h"

int frontState = 1;
int backState = 1;

void debug();

void setup() {
    Serial.begin(9600);
    delay(1000);
    pinMode(pins::led, OUTPUT);
    pinMode(0, INPUT_PULLUP);
    pinMode(9, INPUT_PULLUP);
}

void loop() {
    checkButton(0, pins::led, backState);
    checkButton(9, pins::led, frontState);
    debug();
    delay(200);
}

void debug() {
    Serial.print("Front button ");
    Serial.print(frontState == 0 ? "ON" : "OFF");
    Serial.print(" Back button ");
    Serial.println(backState == 0 ? "ON" : "OFF");
}

// left button will make led flicker because of the loop; if s8 is off it will turn of the led, causing it to flicker

void checkButton(int buttonPin, int ledPin, int buttonState) {
    buttonState = digitalRead(buttonPin);
    Serial.print("LOL BUTTON STATE"); Serial.println(buttonState);
    if (buttonState == LOW) {
        digitalWrite(ledPin, HIGH);
    }
    else {
        digitalWrite(ledPin, LOW);
    }
}
