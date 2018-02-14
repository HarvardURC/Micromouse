#include <Arduino.h>
#include "config.h"
#include "io.hh"

int rgbPins[3];
int rgbState[3] = {0, 0, 0};
int buttonP;
int buzzerP;


RGB_LED::RGB_LED(int redPin, int greenPin, int bluePin) {
    rgbPins[0] = redPin;
    rgbPins[1] = greenPin;
    rgbPins[2] = bluePin;

    pinMode(pins::backLedR, OUTPUT);
    pinMode(pins::backLedG, OUTPUT);
    pinMode(pins::backLedB, OUTPUT);

    // LEDs must be set to HIGH to be off
    digitalWrite(pins::backLedR, HIGH);
    digitalWrite(pins::backLedG, HIGH);
    digitalWrite(pins::backLedB, HIGH);
}

void RGB_LED::flashLED(int color) {
    _turnOn(color);
    delay(100);
    _turnOff(color);
}

void RGB_LED::switchLED(int color) {
    if (rgbState[color]) {
        _turnOff(color);
    }
    else {
        _turnOn(color);
    }
}

void RGB_LED::_turnOn(int color) {
    pinMode(rgbPins[color], OUTPUT);
    digitalWrite(rgbPins[color], LOW);
    rgbState[color] = 1;
}

void RGB_LED::_turnOff(int color) {
    pinMode(rgbPins[color], INPUT);
    rgbState[color] = 0;
}


Button::Button(int buttonPin) {
    buttonP = buttonPin;
    pinMode(buttonP, INPUT_PULLUP);
}


int Button::readButton() {
    return digitalRead(buttonP);
}


Buzzer::Buzzer(int buzzerPin) {
    buzzerP = buzzerPin;
    pinMode(buzzerP, OUTPUT);
    digitalWrite(buzzerP, LOW);
}

void Buzzer::on() {
    digitalWrite(buzzerP, HIGH);
}

void Buzzer::off() {
    digitalWrite(buzzerP, LOW);
}

void Buzzer::siren() {
    on();
    delay(10);
    off();
}
