#include <Arduino.h>
#include <elapsedMillis.h>
#include <PID_v1.h>
#include "motors.hh"
#include "bluetooth.hh"

/* Globals */
const int encoderTolerance = 5000; // error threshold for the encoder values
const unsigned long timeout = 10000;
const float pidLimit = 40.0; // upperlimit on PID output
const float ticksToCm = 1. / 8630; // conversion ratio
const float L = 9.25; // wheel distance in `cm`
const float degToRad = 3.14159 / 180; // converstion ratio
int motorFloor = 27; // lowest motor PWM value


/* PID values */
float p = 10, i = 0, d = 1; // x and y PIDs
float p_a = 16, i_a = 0, d_a = 1; // angle PID
float p_m = 0.002, i_m = 0, d_m = 0; // motor/encoder PIDs


/* Helper functions */

// Function for taking the modulus of a double e.g. `200.56 % 10` = 0.56
// From https://stackoverflow.com/questions/9138790/cant-use-modulus-on-doubles
template<typename T, typename U>
constexpr T dmod (T x, U mod)
{
    return !mod ? x : static_cast<long long>(x) % mod + x - static_cast<long long>(x);
}


// void initializePID(PIDT<float>* pid, float proportion, float integral, float derivative, float pidLimit) {
//     pid->SetOutputLimits(pidLimit * -1, pidLimit);
//     pid->SetTunings(proportion, integral, derivative);
//     //turn the PID on
//     pid->SetMode(AUTOMATIC);
// }


int floorPWM(int speed, int floor) {
    int direction = speed > 0 ? 1 : -1;
    speed = max(abs(speed), floor) * direction;
    Serial.print("Driving at: ");
    Serial.println(speed);
    return speed;
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


/* PidController functions */
PidController::PidController(
    float proportion,
    float integral,
    float derivative) :
    _pid(&input, &output, &setpoint, proportion, integral, derivative, DIRECT)
{
    _pid.SetOutputLimits(pidLimit * -1, pidLimit);
    _pid.SetTunings(proportion, integral, derivative);
    //turn the PID on
    _pid.SetMode(AUTOMATIC);
}


void PidController::compute() {
    _pid.Compute();
}


/* Motor functions */
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
        p_m,
        i_m,
        d_m,
        DIRECT),
    _sensors(sensors)
{
    pinMode(_powerPin, OUTPUT);
    pinMode(_directionPin, OUTPUT);
    analogWriteFrequency(_powerPin, 22000);

    _pidSetpoint = 100000;
    _pidOutput = 0;
    _pid.SetOutputLimits(pidLimit * -1, pidLimit);
    _pid.SetTunings(p_m, i_m, d_m);
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

/* Driver functions */
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
    _sensors(sensors),
    _pid_x(p, i, d),
    _pid_y(p, i, d),
    _pid_a(p_a, i_a, d_a)
{
    curr_xpos = 0.0;
    curr_ypos = 0.0;
    curr_angle = 0.0;
    pinMode(motorModePin, OUTPUT);
    digitalWrite(motorModePin, HIGH);
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


void Driver::computePids() {
    _pid_x.compute();
    _pid_y.compute();
    _pid_a.compute();
}


void Driver::debugPidMovement() {
    Serial.print("x output: ");
    Serial.print(_pid_x.output);
    Serial.print(" x setpoint: ");
    Serial.print(_pid_x.setpoint);
    Serial.print(" xpos: ");
    Serial.println(curr_xpos);

    Serial.print("y output: ");
    Serial.print(_pid_y.output);
    Serial.print(" y setpoint: ");
    Serial.print(_pid_y.setpoint);
    Serial.print(" ypos: ");
    Serial.println(curr_ypos);

    Serial.print(" curr_angle: ");
    Serial.println(curr_angle / degToRad);
}


void Driver::go(float goal_x, float goal_y, float goal_a, int refreshMs) {
    unsigned int interval = refreshMs;
    elapsedMillis timeElapsed = 1000;

    _pid_x.setpoint = goal_x;
    _pid_y.setpoint = goal_y;
    _pid_a.setpoint = goal_a;

    _leftMotor._encoder.write(0);
    _rightMotor._encoder.write(0);
    long enc_left = 0;
    long enc_right = 0;

    float v_left = 0.;
    float v_right = 0.;
    const int motorLimit = 50;
    int end_iter = 0;

    int read_flag = 0;

    do {
        // stores wall readings at halfway point of movement
        if (abs(curr_xpos) >= abs(goal_x) / 2 &&
            abs(curr_ypos) >= abs(goal_y) / 2 &&
            !read_flag)
        {
            for (int i = 0; i < 3; i++) {
                shortTofWallReadings[i] = _sensors.readShortTof(i);
            }
            read_flag = 1;
        }
        if (timeElapsed > interval) {
            _pid_x.input = curr_xpos;
            _pid_y.input = curr_ypos;
            _pid_a.input = curr_angle;
            computePids();

            float lin_velocity = _pid_x.output + _pid_y.output;
            float ang_velocity = _pid_a.output;

            // cut off loop for tank turns once angle is achieved
            // MAGIC NUMBER for cutoff
            float tank_angle_threshold = .2;
            if (_pid_x.setpoint == 0 && _pid_y.setpoint == 0 &&
                abs(curr_angle - _pid_a.setpoint) <
                tank_angle_threshold * 3.14/180)
            {
              break;
            }

            // L is width of robot
            v_left = fixMotorSpeed(lin_velocity - L * ang_velocity / 2,
                motorLimit);
            v_right = fixMotorSpeed(lin_velocity + L * ang_velocity / 2,
                motorLimit);

            /* Begin debug code */
            debugPidMovement();

            Serial.print("v_left: ");
            Serial.print(v_left);
            Serial.print(" v_right: ");
            Serial.println(v_right);
            /* End debug code */

            drive(v_left, v_right);

            // robot state updates
            float sample_t = 1. / interval;
            float true_v_left = (_leftMotor.readTicks() - enc_left) * ticksToCm / sample_t;
            enc_left = _leftMotor.readTicks();
            float true_v_right = (_rightMotor.readTicks() - enc_right) * ticksToCm / sample_t;
            enc_right = _rightMotor.readTicks();

            float true_ang_v = (true_v_right - true_v_left) / L;
            curr_xpos += (true_v_left + true_v_right) / 2  * sample_t * cos(curr_angle);
            curr_ypos += (true_v_left + true_v_right) / 2 * sample_t * sin(curr_angle);
            curr_angle = curr_angle + true_ang_v * sample_t;

            /* If the movement looks like it's reached the goal position
            or it's converged, stop the movement */
            if ((
            withinError(goal_x, curr_xpos, 1.2) &&
            withinError(goal_y, curr_ypos, 1.2) &&
            withinError(goal_a, curr_angle, 0.5)) ||
            (v_left < 5 && v_right < 5))
            {
                end_iter++;
            }
            else {
                end_iter = 0;
            }
            if (end_iter > 250) {
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
