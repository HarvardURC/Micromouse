#include <Arduino.h>
#include <elapsedMillis.h>
#include <PID_v1.h>
#include "motors.hh"
#include "sensors.hh"

// The error threshold for the encoder values
const int encoderTolerance = 5000;
const unsigned long timeout = 10000;
const float proportion = 0.0001;
const float derivative = 0;
const float integral = 0;

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
        proportion,
        integral,
        derivative,
        DIRECT),
    _sensors(sensors)
{
    pinMode(_powerPin, OUTPUT);
    pinMode(_directionPin, OUTPUT);

    _pidSetpoint = 100000;
    _pidOutput = 0;
    _pid.SetOutputLimits(-30.0, 30.0);
    _pid.SetTunings(proportion, integral, derivative);
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
float Motor::getPIDSpeed(float setpoint) {
    _pidSetpoint = setpoint;
    _pidInput = _encoder.read();
    _pid.Compute();
    return _pidOutput;
}


// Moves the ticks specified (mesaured by the encoder, and assisted by the PID)
void Motor::moveTicksPID(long ticks) {
    _pidSetpoint = ticks;
    _pidInput = 0;
    _encoder.write(0);
    long time = millis();
    while (abs(_pidSetpoint - _pidInput) > encoderTolerance && millis() - time < timeout) {
        _pidInput = _encoder.read();
        _pid.Compute();
        drive(_pidOutput);
    }
    drive(0);
}


// Testing function for if the PID works -- should oscillate at setpoint
// or stop if our tuning values are awesome.
void Motor::testPID() {
    _pidSetpoint = 1000000;
    _encoder.write(0);
    _pidInput = _encoder.read();
    Serial.println(_pidInput);
    while (1) {
        _pidInput = _encoder.read();
        _pid.Compute();
        Serial.println(_pidSetpoint);
        Serial.println(_pidInput);
        Serial.println(_pidOutput);
        drive(_pidOutput);
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
    curr_xpos = 0.0;
    curr_ypos = 0.0;
    curr_angle = 0.0;
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


// Sequentially moves each motor a specified amount of ticks
void Driver::moveTicks(long ticks) {
    _leftMotor.moveTicks(ticks);
    _rightMotor.moveTicks(ticks);
}


// Untested
void Driver::turnDegrees(float degrees) {
    double initialAngle = _sensors.readIMUAngle();
    movePID(dmod((initialAngle + degrees), 360) - 180);
}


// Moves the motors to adjust to the input setpoint using PID
void Driver::movePID(float setpoint) {
    float leftSpeed = 1;
    float rightSpeed = 1;
    while (abs(leftSpeed) > 0.5 || abs(rightSpeed) > 0.5) {
        float leftSpeed = _leftMotor.getPIDSpeed(setpoint);
        float rightSpeed = _rightMotor.getPIDSpeed(setpoint);
        // Debugging code
        // Serial.print("Left motor speed: ");
        // Serial.print(leftSpeed);
        // Serial.print(" Right motor speed: ");
        // Serial.print(rightSpeed);
        // Serial.print(" Setpoint: ");
        // Serial.println(setpoint);
        delay(100);
        drive(leftSpeed, rightSpeed);
    }
}

void initializePID(PIDT<float>* pid, float proportion, float integral, float derivative) {
    pid->SetOutputLimits(-30.0, 30.0);
    pid->SetTunings(proportion, integral, derivative);
    //turn the PID on
    pid->SetMode(AUTOMATIC);
}


void Driver::go(float goal_x, float goal_y, float goal_a, int refreshMs) {
    unsigned int interval = refreshMs;
    elapsedMillis timeElapsed;
    // x = x position, y = y position, a = angle
    // i = input, o = output, s = setpoint
    float x_i=0, x_o=0, x_s=0, y_i=0, y_o=0, y_s=0, a_i=0, a_o=0, a_s=0;
    float prop = 0.001;
    float integ = 0;
    float der = 0;

    PIDT<float> pid_x(&x_i, &x_o, &x_s, prop, integ, der, DIRECT);
    PIDT<float> pid_y(&y_i, &y_o, &y_s, prop, integ, der, DIRECT);
    PIDT<float> pid_a(&a_i, &a_o, &a_s, prop, integ, der, DIRECT);
    initializePID(&pid_x, prop, integ, der);
    initializePID(&pid_y, prop, integ, der);
    initializePID(&pid_a, prop, integ, der);

    x_s = goal_x;
    y_s = goal_y;
    a_s = goal_a;

    const float L = 9.25; // centimeters

    while (1) {
        if (timeElapsed > interval) {
            x_i = curr_xpos;
            y_i = curr_ypos;
            a_i = curr_angle;

            pid_x.Compute();
            pid_y.Compute();
            pid_a.Compute();

            float lin_velocity = x_o + y_o;
            float ang_velocity = a_o;

            Serial.print("x_o: ");
            Serial.print(x_o);
            Serial.print(" lin_velocity: ");
            Serial.print(lin_velocity);
            Serial.print(" x_s: ");
            Serial.print(x_s);
            Serial.print(" xpos: ");
            Serial.println(curr_xpos);

            // L is width of robot?
            float v_left = lin_velocity - L * ang_velocity / 2;
            float v_right = lin_velocity + L * ang_velocity / 2;

            // Serial.print("v_left: ");
            // Serial.print(v_left);
            // Serial.print(" v_right: ");
            // Serial.println(v_right);
            drive(v_left, v_right);
            // robot state updates
            float sample_t = 1 / interval;
            curr_xpos += (v_left + v_right) / 2  * sample_t * cos(curr_angle);
            curr_ypos += (v_left + v_right) / 2 * sample_t * sin(curr_angle);
            curr_angle = dmod((curr_angle + ang_velocity * sample_t), 360);

            // reset sample time
            timeElapsed = 0;
        }
    }
}
