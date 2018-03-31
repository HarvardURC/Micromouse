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

    pinMode(pins::backButton, INPUT_PULLUP);
    pinMode(pins::frontButton, INPUT_PULLUP);
    pinMode(pins::backLedG, OUTPUT);
    pinMode(pins::frontLedG, OUTPUT);
}

void loop() {
    checkButton(pins::backButton, pins::backLedG, backState);
    checkButton(pins::frontButton, pins::frontLedG, frontState);
}

void debug() {
    Serial.print("Front button ");
    Serial.print(frontState == 0 ? "ON" : "OFF");
    Serial.print(" Back button ");
    Serial.println(backState == 0 ? "ON" : "OFF");
}

void checkButton(int buttonPin, int ledPin, int buttonState) {
    buttonState = digitalRead(buttonPin);
    if (buttonState == LOW) {
        digitalWrite(ledPin, LOW);
    }
    else {
        digitalWrite(ledPin, HIGH);
    }
}
