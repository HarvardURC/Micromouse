#include <Arduino.h>
#include <elapsedMillis.h>
#include <PID_v1.h>
#include "motors.hh"
#include "sensors.hh"
#include "bluetooth.hh"

// The error threshold for the encoder values
const int encoderTolerance = 5000;
const unsigned long timeout = 10000;
const float proportion = 0.002;
const float derivative = 0;
const float integral = 0;
const float pidLimit = 70.0;
// Conversion ratio from encoder ticks to centimeters
const float ticksToCm = 1. / 8630;
// Wheel distance
const float L = 9.25; // centimeters
const float degToRad = 3.14159 / 180;
int motorFloor = 30;

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
    analogWriteFrequency(_powerPin, 22000);

    _pidSetpoint = 100000;
    _pidOutput = 0;
    _pid.SetOutputLimits(pidLimit * -1, pidLimit);
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
    Serial.print("Encdoer: ");
    Serial.println(_pidInput);
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
    curr_xpos = 0.0;
    curr_ypos = 0.0;
    curr_angle = 0.0;
    pinMode(motorModePin, OUTPUT);
    digitalWrite(motorModePin, HIGH);
}

int floorPWM(int speed, int floor) {
    int direction = speed > 0 ? 1 : -1;
    speed = max(abs(speed), floor) * direction;
    Serial.print("Driving at: ");
    Serial.println(speed);
    return speed;
}


void Driver::drive(int speed) {
    speed = floorPWM(speed, motorFloor);

    _leftMotor.drive(speed);
    _rightMotor.drive(speed);
}


void Driver::drive(int speedLeft, int speedRight) {
    _leftMotor.drive(floorPWM(speedLeft, motorFloor));
    _rightMotor.drive(floorPWM(speedRight, motorFloor));
}

void Driver::brake() {
    _leftMotor.drive(0);
    _rightMotor.drive(0);
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
    int start_flag = 0;
    while (abs(leftSpeed) > 0.5 || abs(rightSpeed) > 0.5) {
        float leftSpeed = _leftMotor.getPIDSpeed(setpoint);
        float rightSpeed = _rightMotor.getPIDSpeed(setpoint);
        // Debugging code
        Serial.print("Left motor speed: ");
        Serial.print(leftSpeed);
        Serial.print(" Right motor speed: ");
        Serial.print(rightSpeed);
        Serial.print(" Setpoint: ");
        Serial.println(setpoint);
        // delay(500);
        if (abs(leftSpeed) > 1 && abs(rightSpeed) > 1) {
            start_flag = 1;
        }
        if (start_flag) {
            drive(leftSpeed, rightSpeed);
        }
    }
}

void initializePID(PIDT<float>* pid, float proportion, float integral, float derivative) {
    pid->SetOutputLimits(pidLimit * -1, pidLimit);
    pid->SetTunings(proportion, integral, derivative);
    //turn the PID on
    pid->SetMode(AUTOMATIC);
}


float fixMotorSpeed(float speed, int limit) {
    float outspeed = speed;
    if (speed > limit) {
        outspeed = limit;
    }
    else if (speed < -1 * limit) {
        outspeed = -1 * limit;
    }
    return outspeed;
}

// Checks if the two numbers are within a margin of error from each other
bool withinError(float a, float b, float error) {
    return abs(b - a) < error;
}


void Driver::go(float goal_x, float goal_y, float goal_a, int refreshMs) {
    unsigned int interval = refreshMs;
    elapsedMillis timeElapsed = 1000;
    // x = x position, y = y position, a = angle
    // i = input, o = output, s = setpoint
    float x_i=0, x_o=0, x_s=0, y_i=0, y_o=0, y_s=0, a_i=0, a_o=0, a_s=0;
    float prop = 10;
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

    _leftMotor._encoder.write(0);
    _rightMotor._encoder.write(0);
    long encLeft = 0;
    long encRight = 0;
    float v_left = 0.;
    float v_right = 0.;
    const int motorLimit = 50;

    do {
        if (timeElapsed > interval) {
            x_i = curr_xpos;
            y_i = curr_ypos;
            a_i = curr_angle;

            pid_x.Compute();
            pid_y.Compute();
            pid_a.Compute();

            float lin_velocity = x_o + y_o;
            float ang_velocity = a_o;

            // cut off loop for tank turns once angle is achieved
            // MAGIC NUMBER for cutoff
            float tank_angle_threshold = .2;
            if (x_s == 0 && y_s == 0 && abs(curr_angle - a_s) < tank_angle_threshold * 3.14/180) {
              break;
            }

            // L is width of robot?
            v_left = fixMotorSpeed(lin_velocity - L * ang_velocity / 2, motorLimit);
            v_right = fixMotorSpeed(lin_velocity + L * ang_velocity / 2, motorLimit);

            /* Begin debug code */
            Serial.print("x_o: ");
            Serial.print(a_s);
            Serial.print(" lin_velocity: ");
            Serial.print(lin_velocity);
            Serial.print(" x_s: ");
            Serial.print(x_s);
            Serial.print(" xpos: ");
            Serial.println(curr_xpos);

            Serial.print("y_o: ");
            Serial.print(y_o);
            Serial.print(" lin_velocity: ");
            Serial.print(lin_velocity);
            Serial.print(" y_s: ");
            Serial.print(y_s);
            Serial.print(" ypos: ");
            Serial.println(curr_ypos);

            Serial.print(" curr_angle: ");
            Serial.println(curr_angle * 180 / 3.14);

            Serial.print("v_left: ");
            Serial.print(v_left);
            Serial.print(" v_right: ");
            Serial.println(v_right);
            /* End debug code */

            drive(v_left, v_right);

            // robot state updates
            float sample_t = 1. / interval;
            float true_v_left = (_leftMotor.readTicks() - encLeft) * ticksToCm / sample_t;
            encLeft = _leftMotor.readTicks();
            float true_v_right = (_rightMotor.readTicks() - encRight) * ticksToCm / sample_t;
            encRight = _rightMotor.readTicks();

            float true_ang_v = (true_v_right - true_v_left) / L;
            curr_xpos += (true_v_left + true_v_right) / 2  * sample_t * cos(curr_angle);
            curr_ypos += (true_v_left + true_v_right) / 2 * sample_t * sin(curr_angle);
            curr_angle = curr_angle + true_ang_v * sample_t;

            if ((
            withinError(goal_x, curr_xpos, 0.5) &&
            withinError(goal_y, curr_ypos, 0.5) &&
            withinError(goal_a, curr_angle, 0.1)) ||
            (v_left < 2 && v_right < 2)) {
                break;
            }

            // reset sample time
            timeElapsed = 0;
        }
    } while (1);
    brake();
    Serial.println("Done with movement.");
}


void Driver::forward(float distance) {
    float goal_x = curr_xpos + cos(curr_angle) * distance;
    float goal_y = curr_ypos + sin(curr_angle) * distance;
    go(goal_x, goal_y, curr_angle);
}


void Driver::turnLeft(float degrees) {
    float goal_a = curr_angle + degToRad * degrees;
    go(curr_xpos, curr_ypos, goal_a);
}


void Driver::turnRight(float degrees) {
    turnLeft(-1 * degrees);
}


void Driver::resetState() {
    this->curr_xpos = 0;
    this->curr_ypos = 0;
    this->curr_angle = 0;
}