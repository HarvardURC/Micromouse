#include <Arduino.h>
#include <PID_v1.h>
#include "motors.hh"
#include "sensors.hh"

// The error threshold for the encoder values
int encoderTolerance = 10000;


Motor::Motor(
    int powerPin,
    int directionPin,
    int encoderPin1,
    int encoderPin2,
    SensorArray sensors) :
    _encoder(encoderPin1, encoderPin2),
    _pid(
        &_pidInput,
        &_pidOutput,
        &pidSetpoint,
        _pidProportion,
        _pidIntegral,
        _pidDerivative,
        DIRECT),
    _sensors(sensors)
{
    _powerPin = powerPin;
    _directionPin = directionPin;

    pinMode(_powerPin, OUTPUT);
    pinMode(_directionPin, OUTPUT);

    _pid.SetOutputLimits(-100, 100);
    //turn the PID on
    _pid.SetMode(AUTOMATIC);
}


void Motor::drive(int speed) {
    digitalWrite(_directionPin, speed > 0);
    analogWrite(_powerPin, speed >= 0 ? speed : speed * -1);
}


// Reads the current value of the encoder
long Motor::readTicks() {
    return _encoder.read();
}


// Moves a number of specified ticks as measured by the encoder
void Motor::moveTicks(long ticks) {
    // magic number
    int speed = 30;
    _encoder.write(0);
    while (_encoder.read() < abs(ticks)) {
        Serial.print("Encoder Value for motorpin ");
        Serial.print(_powerPin);
        Serial.print(" ");
        Serial.println(_encoder.read());
        drive(ticks > 0 ? speed : -1 * speed);
    }
    drive(0);
}


// Returns the speed the motor should go according to the PID to acheive the
// setpoint.
double Motor::getPIDSpeed() {
    Serial.print("Setpoint and input: ");
    Serial.print(pidSetpoint);
    Serial.print(" ");
    Serial.println(_encoder.read());
    _pidInput = _encoder.read();
    if (abs(pidSetpoint - _pidInput) > encoderTolerance) {
        _pid.Compute();
        return _pidOutput;
    }
    else {
        return 0;
    }
}

// Moves the ticks specified (mesaured by the encoder, and assisted by the PID)
void Motor::moveTicksPID(long ticks) {
    pidSetpoint = ticks;
    _pidInput = _encoder.read();
    if (abs(pidSetpoint - _pidInput) > encoderTolerance) {
        _pid.Compute();
        Serial.println(_pidInput);
        drive(_pidOutput);
    }
    drive(0);
}


Driver::Driver(
    int powerPinL,
    int directionPinL,
    int powerPinR,
    int directionPinR,
    int motorModePin,
    int encoderPinL1,
    int encoderPinL2,
    int encoderPinR1,
    int encoderPinR2,
    SensorArray sensors) :
    _leftMotor(powerPinL, directionPinL, encoderPinL1, encoderPinL2, sensors),
    _rightMotor(powerPinR, directionPinR, encoderPinR1, encoderPinR2, sensors),
    _sensors(sensors)
{
    pinMode(motorModePin, OUTPUT);
    digitalWrite(motorModePin, HIGH);
}


void Driver::drive(int speed) {
    _leftMotor.drive(speed);
    _rightMotor.drive(speed);
}


void Driver::moveTicks(long ticks) {
    _leftMotor.moveTicks(ticks);
    _rightMotor.moveTicks(ticks);
}

// Buggy -- does not work
void Driver::moveTicksPID(long ticks) {
    Serial.begin(9600);
    delay(1000);
    double leftSpeed = 1;
    double rightSpeed = 1;
    _leftMotor.pidSetpoint = ticks;
    _rightMotor.pidSetpoint = ticks;

    while (abs(leftSpeed) > 0.5 || abs(rightSpeed) > 0.5) {
        double leftSpeed = _leftMotor.getPIDSpeed();
        double rightSpeed = _rightMotor.getPIDSpeed();
        _leftMotor.drive(leftSpeed);
        _rightMotor.drive(rightSpeed);
    }
}
