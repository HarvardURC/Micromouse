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

// Flashes the LED like a police car
void RGB_LED::flashLED(int color) {
    _turnOn(color);
    delay(100);
    _turnOff(color);
}

// Flips the state (on/off) of the `color` LED
void RGB_LED::switchLED(int color) {
    if (rgbState[color]) {
        _turnOff(color);
    }
    else {
        _turnOn(color);
    }
}

// Turns off all LEDS and turns on `color`
void RGB_LED::turnOn(int color) {
    for (int i = 0; i < 3; i++) {
        _turnOff(i);
    }
    _turnOn(color);
}

// Turns off all LEDs
void RGB_LED::turnOff() {
    for (int i = 0; i < 3; i++) {
        _turnOff(i);
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


Button::Button(int buttonPin) : _buttonPin(buttonPin) {
    pinMode(_buttonPin, INPUT_PULLUP);
}

/* Reads the input of the button. Note that the value of the read when the
 * button is pressed is LOW.
 */
int Button::read() {
    return digitalRead(_buttonPin);
}


Buzzer::Buzzer(int buzzerPin) : _buzzerPin(buzzerPin) {
    pinMode(_buzzerPin, OUTPUT);
    digitalWrite(_buzzerPin, LOW);
}

void Buzzer::on() {
    digitalWrite(_buzzerPin, HIGH);
}

void Buzzer::off() {
    digitalWrite(_buzzerPin, LOW);
}

void Buzzer::siren() {
    on();
    delay(10);
    off();
}
