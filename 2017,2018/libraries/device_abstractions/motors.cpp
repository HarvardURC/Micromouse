#include <Arduino.h>
#include <PID_v1.h>
#include "motors.hh"
#include "sensors.hh"

// The error threshold for the encoder values
int encoderTolerance = 5000;

// Function for taking the modulus of a double e.g. `200.56 % 10` = 0.56
// From https://stackoverflow.com/questions/9138790/cant-use-modulus-on-doubles
template<typename T, typename U>
constexpr T dmod (T x, U mod)
{
    return !mod ? x : static_cast<long long>(x) % mod + x - static_cast<long long>(x);
}

Motor::Motor(
    int powerPin,
    int directionPin,
    int encoderPin1,
    int encoderPin2,
    SensorArray sensors) :
    _powerPin(powerPin),
    _directionPin(directionPin),
    _encoder(encoderPin1, encoderPin2),
    _pid(
        &_pidInput,
        &_pidOutput,
        &_pidSetpoint,
        _pidProportion,
        _pidIntegral,
        _pidDerivative,
        DIRECT),
    _sensors(sensors)
{
    pinMode(_powerPin, OUTPUT);
    pinMode(_directionPin, OUTPUT);

    _pidSetpoint=100000;
    _pid.SetOutputLimits(-50.0, 50.0);
    //turn the PID on
    _pid.SetMode(AUTOMATIC);
}


void Motor::drive(int speed) {
    Serial.println(speed);
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
float Motor::getPIDSpeed(float setpoint) {
    _pidSetpoint = setpoint;
    Serial.print("Setpoint and input: ");
    Serial.print(_pidSetpoint);
    Serial.print(" ");
    Serial.println(_encoder.read());
    _pidInput = _encoder.read();
    _pid.Compute();
    return _pidOutput;
}


// Moves the ticks specified (mesaured by the encoder, and assisted by the PID)
void Motor::moveTicksPID(long ticks) {
    _pidSetpoint = ticks;
    _encoder.write(0);
    while (abs(_pidSetpoint - _pidInput) > encoderTolerance) {
        _pidInput = _encoder.read();
        _pid.Compute();
        Serial.println(_pidInput);
        Serial.println(_pidOutput);
        drive(_pidOutput);
    }
    drive(0);
}


// Testing function for if the PID works -- should oscillate at setpoint
// or stop if our tuning values are awesome.
void Motor::testPID() {
    _pidSetpoint = 100000;
    _encoder.write(0);
    _pidInput = _encoder.read();
    Serial.println(_pidInput);
    while (1) {
        _pidInput = _encoder.read();
        _pid.Compute();
        Serial.print("Setpoint: ");
        Serial.print(_pidSetpoint);
        Serial.print(", input: ");
        Serial.println(_pidInput);
        Serial.print(", output: ");
        Serial.println(_pidOutput);
        drive(_pidOutput);
        delay(50);
    }
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


void Driver::drive(int speedLeft, int speedRight) {
    _leftMotor.drive(speedLeft);
    _rightMotor.drive(speedRight);
}


void Driver::moveTicks(long ticks) {
    _leftMotor.moveTicks(ticks);
    _rightMotor.moveTicks(ticks);
}


// Buggy -- does not work
void Driver::moveTicksPID(long ticks) {
    movePID(ticks);
}


// Untested
void Driver::turnDegrees(float degrees) {
    double initialAngle = _sensors.readIMUAngle();
    movePID(dmod((initialAngle + degrees), 360) - 180);
}


// Moves the motors to adjust to the input setpoint using PID
// Untested
void Driver::movePID(float setpoint) {
    float leftSpeed = 1;
    float rightSpeed = 1;
    while (abs(leftSpeed) > 0.5 || abs(rightSpeed) > 0.5) {
        // float leftSpeed = _leftMotor.getPIDSpeed(setpoint);
        float rightSpeed = _rightMotor.getPIDSpeed(setpoint);
        Serial.print("Left motor speed: ");
        Serial.print(leftSpeed);
        Serial.print(" Right motor speed: ");
        Serial.println(rightSpeed);
        drive(leftSpeed, rightSpeed);
    }
}
