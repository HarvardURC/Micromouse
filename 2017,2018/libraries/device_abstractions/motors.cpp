#include <Arduino.h>
#include "motors.hh"

Motor::Motor(int powerPin, int directionPin) {
    _powerPin = powerPin;
    _directionPin = directionPin;

    pinMode(_powerPin, OUTPUT);
    pinMode(_directionPin, OUTPUT);
}


void Motor::drive(int speed) {
    digitalWrite(_directionPin, speed > 0);
    analogWrite(_powerPin, speed >= 0 ? speed : speed * -1);
}


Driver::Driver(int powerPinL, int directionPinL, int powerPinR, int directionPinR, int motorModePin) :
    _leftMotor(powerPinL, directionPinL),
    _rightMotor(powerPinR, directionPinR)
{
    pinMode(motorModePin, OUTPUT);
    digitalWrite(motorModePin, HIGH);
}


void Driver::drive(int speed) {
    _leftMotor.drive(speed);
    _rightMotor.drive(speed);
}
