#include <Arduino.h>
#include <elapsedMillis.h>
#include <PID_v1.h>
#include <cmath>
#include "motors.hh"
#include "bluetooth.hh"

/* Globals */
const int encoderTolerance = 5000; // error threshold for the encoder values
const int frontTolerance = 50;
const unsigned long timeout = 10000;
const float pidLimit = 60.0; // upperlimit on PID output
const float ticksToCm = 1. / 8630; // conversion ratio
const float L = 9.25; // wheel distance in `cm`
const int motorLimit = 50; // highest motor PWM value
const int motorFloor = 25; // lowest motor PWM value
const float degToRad = PI / 180; // converstion ratio
const int sensorRefreshTime = 120; // milliseconds
const int convergenceTime = 250; // milliseconds
const float errorX = 0.7; // centimeters
const float errorY = errorX;
const float errorA = 0.35; // radians
const float angWeight = 1; // ratio of IMU vs. encoder measurements for angle


/* PID values */
float p = 18, i = 0, d = 1; // x and y PIDs
float p_a = 20, i_a = 0, d_a = 1; // angle PID
float p_m = 0.002, i_m = 0, d_m = 0; // motor/encoder PIDs


/* Helper functions */

// Function for taking the modulus of a double e.g. `200.56 % 10` = 0.56
// From https://stackoverflow.com/questions/9138790/cant-use-modulus-on-doubles
template<typename T, typename U>
constexpr T dmod (T x, U mod)
{
    return !mod ? x : static_cast<long long>(x) % mod + x - static_cast<long long>(x);
}


/* Enforces a minimum PWM input to the motors */
int floorPWM(int speed, int floor) {
    int direction = speed > 0 ? 1 : -1;
    speed = abs(speed) >= floor ? abs(speed) * direction : 0;
    return speed;
}


/* Enforces a maximum PWN input to the motors */
float ceilingPWM(float speed, int limit) {
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
    SensorArray sensors,
    bool bluetoothOn) :
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
    bluetoothOn_ = bluetoothOn;
    pinMode(motorModePin, OUTPUT);
    digitalWrite(motorModePin, HIGH);

    clearWallData();
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
    Serial.print("x input: ");
    Serial.print(_pid_x.input);
    Serial.print(" x output: ");
    Serial.print(_pid_x.output);
    Serial.print(" x setpoint: ");
    Serial.print(_pid_x.setpoint);
    Serial.print(" xpos: ");
    Serial.println(curr_xpos);

    Serial.print("y input: ");
    Serial.print(_pid_y.input);
    Serial.print(" y output: ");
    Serial.print(_pid_y.output);
    Serial.print(" y setpoint: ");
    Serial.print(_pid_y.setpoint);
    Serial.print(" ypos: ");
    Serial.println(curr_ypos);

    Serial.print(" a setpoint: ");
    Serial.print(_pid_a.setpoint / degToRad);
    Serial.print(" curr_angle: ");
    Serial.println(curr_angle / degToRad);
}


/* reads each of the short tof sensors values into an array for wall checking */
void Driver::readWalls() {
    for (int i = 0; i < 3; i++) {
        if ((i == 1 && _sensors.readShortTof(i) < shortTofWallReadings[1]) ||
            (_sensors.readShortTof(i) > shortTofWallReadings[i]))
        {
            shortTofWallReadings[i] = _sensors.readShortTof(i);
        }
    }
}


/* resets the wall readings array */
void Driver::clearWallData() {
    for (int i = 0; i < 3; i++) {
        shortTofWallReadings[i] = 0;
    }
    shortTofWallReadings[1] = 1000;
}


/* Calculates the right goal_angle to feed into `go` to get a minimal turn */
float minTurn(float goal_angle, float curr_angle) {
    // Get it into the same 2Pi bracket as curr_angle
    // e.g. [0, 2 Pi), [-2Pi, 0)
    while (goal_angle > curr_angle) {
        goal_angle -= 2 * PI;
    }
    while (goal_angle < curr_angle) {
        goal_angle += 2 * PI;
    }
    if (goal_angle - curr_angle > PI) {
        return goal_angle - (2 * PI);
    }
    else {
        return goal_angle;
    }
}


/* Moves based on absolute position on a coordinate grid */
void Driver::go(float goal_x, float goal_y, float goal_a, int refreshMs) {
    unsigned int interval = refreshMs;
    elapsedMillis timeElapsed = 1000;
    elapsedMillis bluetoothTimer = 0;
    elapsedMillis sensorTimer = 0;
    int sensorCounter = 0;

    goal_a = minTurn(goal_a, curr_angle);
    Serial.print("New goal_a: ");
    Serial.println(curr_angle);

    _leftMotor._encoder.write(0);
    _rightMotor._encoder.write(0);
    long enc_left = 0;
    long enc_right = 0;
    const float init_xpos = curr_xpos;
    const float init_ypos = curr_ypos;
    _pid_x.setpoint = abs(goal_x - init_xpos);
    _pid_y.setpoint = abs(goal_y - init_ypos);
    _pid_a.setpoint = goal_a;

    float v_left = 0;
    float v_right = 0;
    int end_iter = 0;
    int overflow_count = floor(curr_angle / (2 * PI));
    bool angle_flag = goal_x == curr_xpos && goal_y == curr_ypos;

    debugPidMovement();

    do {
        /* stores sensor readings to detect walls
         * NOTE: will break if sensors not initialized */
        if ((goal_y != init_ypos || goal_x != init_xpos) &&
            sensorTimer > sensorRefreshTime &&
            sensorCounter < 4)
        {
            if (bluetoothOn_) {
                ble.print("Walls read at ");
                ble.print("x=");
                ble.println(curr_xpos);
                ble.print("y=");
                ble.println(curr_ypos);
            }
            readWalls();
            sensorTimer = 0;
            sensorCounter++;
        }

        if (timeElapsed > interval) {
            _pid_x.input = abs(curr_xpos - init_xpos);
            _pid_y.input = abs(curr_ypos - init_ypos);
            _pid_a.input = curr_angle;
            computePids();

            // use distance formula to get positive linear velocity
            float lin_velocity = angle_flag ? 0 : sqrt(pow(_pid_x.output, 2) + pow(_pid_y.output, 2));
            float ang_velocity = _pid_a.output;

            // L is width of robot
            v_left = ceilingPWM(lin_velocity - L * ang_velocity / 2,
                motorLimit);
            v_right = ceilingPWM(lin_velocity + L * ang_velocity / 2,
                motorLimit);

            if (angle_flag) v_right = -1 * v_left;

            /* Begin debug code */
            debugPidMovement();

            if (bluetoothOn_ && bluetoothTimer >= 1000) {
                ble.print("v_left: ");
                ble.print(v_left);
                ble.print(" v_right: ");
                ble.println(v_right);
                bluetoothTimer = 0;
            }
            else if (!bluetoothOn_) {
                Serial.print("v_left: ");
                Serial.print(v_left);
                Serial.print(" v_right: ");
                Serial.println(v_right);
            }
            /* End debug code */

            drive(v_left, v_right);

            // robot state updates
            float sample_t = 1. / interval;
            float true_v_left = (_leftMotor.readTicks() - enc_left) * ticksToCm / sample_t;
            enc_left = _leftMotor.readTicks();
            float true_v_right = (_rightMotor.readTicks() - enc_right) * ticksToCm / sample_t;
            enc_right = _rightMotor.readTicks();

            float true_ang_v = (true_v_right - true_v_left) / L;
            curr_xpos += (true_v_left + true_v_right) / 2  * sample_t * -1 * sin(curr_angle);
            curr_ypos += (true_v_left + true_v_right) / 2 * sample_t * cos(curr_angle);
            float imu_rads = (360 - _sensors.readIMUAngle()) * degToRad;

            /* used for if the current angle `a` > 2PI or `a` < 0 to correct
            the IMU angle. */
            float overflow = overflow_count * 2 * PI;
            if (abs(curr_angle - (imu_rads + overflow)) > PI) {
                if (curr_angle > imu_rads + overflow) {
                    overflow_count++;
                }
                else {
                    overflow_count--;
                }
            }
            curr_angle = angWeight * (imu_rads + overflow_count * 2 * PI) +
                (1-angWeight) * (curr_angle + true_ang_v * sample_t);

            /* If the movement looks like it's reached the goal position
            or it's converged, stop the movement */
            if ((
            withinError(goal_x, curr_xpos, errorX) &&
            withinError(goal_y, curr_ypos, errorY) &&
            withinError(goal_a, curr_angle, errorA)) ||
            (v_left < motorFloor && v_right < motorFloor))
            {
                end_iter++;
            }
            else {
                end_iter = 0;
            }
            if (end_iter > convergenceTime) {
                break;
            }

            // reset sample time
            timeElapsed = 0;
        }
    } while (1);

    brake();
    if (bluetoothOn_) ble.println("Done with movement.");
}


void Driver::forward(float distance) {
    float goal_x = curr_xpos - sin(curr_angle) * distance;
    float goal_y = curr_ypos + cos(curr_angle) * distance;
    go(goal_x, goal_y, curr_angle);
}


void Driver::turnLeft(float degrees) {
    float goal_a = curr_angle + degToRad * degrees;
    go(curr_xpos, curr_ypos, goal_a);
}


void Driver::turnRight(float degrees) {
    turnLeft(-1 * degrees);
}



/* Moves the robot to the input goal state in discrete tank style movements
 * of move forward and turn */
void Driver::tankGo(float goal_x, float goal_y, float goal_a) {
    // I THINK IT SHOULD BE THIS
    //float temp_a = atan2(goal_y - curr_ypos, goal_x - curr_xpos);
    float temp_a = atan2(-1*(goal_x - curr_xpos), goal_y - curr_ypos);
    if (bluetoothOn_) {
        ble.print("goal_a: ");
        ble.print(goal_a);
        ble.print(" temp_a: ");
        ble.println(temp_a);
    }
    if (temp_a != goal_a) {
        // Turn
        go(curr_xpos, curr_ypos, temp_a);
        // Go forward
        go(goal_x, goal_y, temp_a);
    }
    else {
        go(goal_x, goal_y, goal_a);
    }
}


void Driver::resetState() {
    this->curr_xpos = 0;
    this->curr_ypos = 0;
}
