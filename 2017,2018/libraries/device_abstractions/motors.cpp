#include <Arduino.h>
#include <elapsedMillis.h>
#include <PID_v1.h>
#include <cmath>
#include "motors.hh"
#include "bluetooth.hh"

#define SIGN(x) 2 * (x > 0) - 1

/* Globals */
const bool debug = true; // set to true for serial debugging statements
const int encoderTolerance = 5000; // error threshold for the encoder values
const int frontTolerance = 50;
const unsigned long timeout = 10000;
const int pidSampleTime = 5; // parameter to PID library
const float pidLimit = 50.0; // upperlimit on PID output
const float ticksToCm = 1. / 8300; // conversion ratio 8360
const float L = 9.25; // wheel distance in `cm`
const int motorLimit = 50; // highest motor PWM value
const int motorFloor = 20; // lowest motor PWM value (IN TUNING)
const int motorCloseEnough = 20; // motor value considered close to target (IN TUNING) 15
const float perpendicular_threshold = 0.5; // range to 90 degrees to terminate move
const float degToRad = PI / 180; // converstion ratio
const int sensorRefreshTime = 120; // milliseconds
const int convergenceTime = 20; // milliseconds
const float errorX = 0.9; // centimeters .9
const float errorY = errorX;
const float errorA = 0.2; // radians .2
const float angWeight = 0; // ratio of IMU vs. encoder measurements for angle
const float cellSize = 18; // size in cm of cell
const float frontSensorToWheelAxis = 4.75; // cm
// the distance from the center of the cell to the wall where sensor reads
const float distCenterToInnerWall = 8.5;
// how far from wall to align to be in the center of the cell in cm
const float alignDist = distCenterToInnerWall - frontSensorToWheelAxis;

/* PID values */
float p_l = 12, i_l = 0, d_l = 0; // linear PIDs, x and y position
float p_a = 20, i_a = 0, d_a = 0; // angle PID
float p_tof = 12, i_tof = 0, d_tof = 0; // front ToF sensor PID

// motor/encoder PID (ONLY FOR MOTOR CLASS FUNCTIONS)
float p_m = 0.002, i_m = 0, d_m = 0;


/* Helper functions */

// Function for taking the modulus of a double e.g. `200.56 % 10` = 0.56
// From https://stackoverflow.com/questions/9138790/cant-use-modulus-on-doubles
template<typename T, typename U>
constexpr T dmod (T x, U mod)
{
    return !mod ? x :
        static_cast<long long>(x) % mod + x - static_cast<long long>(x);
}


/* Enforces a minimum PWM input to the motors */
int floorPWM(int speed, int floor) {
    return fabs(speed) >= floor ? fabs(speed) * SIGN(speed) : 0;
}


/* Enforces a maximum PWN input to the motors
   (scales 2 speeds down so the max is under the limit) */
float ceilingPWM(float speed, float otherspeed, int limit) {
    if (fabs(speed) > fabs(otherspeed)) {
        if (speed > limit) {
          return limit;
        } else if (speed < -1 * limit) {
          return -1 * limit;
        } else {
          return speed;
        }
    } else {
        if (otherspeed > limit) {
          return speed * limit / otherspeed;
        } else if (otherspeed < -1 * limit) {
          return speed * -1 * limit / otherspeed;
        } else {
          return speed;
        }
    }
}


// Checks if the two numbers are within a margin of error from each other
bool withinError(float a, float b, float error) {
    return fabs(b - a) < error;
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
    _pid.SetSampleTime(pidSampleTime);
    this->proportion = proportion;
    this->integral = integral;
    this->derivative = derivative;
    this->input = 0;
    this->output = 0;
    this->setpoint = 0;
    //turn the PID on
    _pid.SetMode(AUTOMATIC);
}


void PidController::compute() {
    _pid.Compute();
}

void PidController::setTunings(float p, float i, float d) {
    this->_pid.SetTunings(p, i, d);
    this->proportion = p;
    this->integral = i;
    this->derivative = d;
}

void PidController::printTunings() {
    if (ble.isConnected()) {
        ble.print("Proportion: ");
        ble.print(this->proportion);
        ble.print(" Integral: ");
        ble.print(this->integral);
        ble.print(" Derivative: ");
        ble.println(this->derivative);
    }
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
    while (abs(_pidSetpoint - _pidInput) > encoderTolerance &&
        millis() - time < timeout)
    {
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
    _pid_x(p_l, i_l, d_l),
    _pid_y(p_l, i_l, d_l),
    _pid_a(p_a, i_a, d_a),
    _pid_front_tof(p_tof, i_tof, d_tof)
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
    _leftMotor.drive(speedLeft);
    _rightMotor.drive(speedRight);
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
    while (fabs(leftSpeed) > 0.5 || fabs(rightSpeed) > 0.5) {
        float leftSpeed = _leftMotor.getPIDSpeed(setpoint);
        float rightSpeed = _rightMotor.getPIDSpeed(setpoint);
        // Debugging code
        if (debug) {
            Serial.print("Left motor speed: ");
            Serial.print(leftSpeed);
            Serial.print(" Right motor speed: ");
            Serial.print(rightSpeed);
            Serial.print(" Setpoint: ");
            Serial.println(setpoint);
        }
        // delay(500);
        if (fabs(leftSpeed) > 1 && fabs(rightSpeed) > 1) {
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

    Serial.print("a input: ");
    Serial.print(_pid_a.input);
    Serial.print(" a output: ");
    Serial.print(_pid_a.output);
    Serial.print(" a setpoint: ");
    Serial.print(_pid_a.setpoint);
    Serial.print(" apos: ");
    Serial.println(curr_angle);
    /*Serial.print(" a setpoint: ");
    Serial.print(_pid_a.setpoint / degToRad);
    Serial.print(" curr_angle: ");
    Serial.println(curr_angle / degToRad);*/

    Serial.print("_v_left: ");
    Serial.print(_v_left);
    Serial.print(" _v_right: ");
    Serial.println(_v_right);
}


/* readWalls()
 *
 * Reads each of the short tof sensors values into an array for wall checking.
 *
 * When called, overwrites the values of the left diagonal, front, and right
 * diagonal time-of-flight sensors in an array on Driver object if they
 * fulfill certain conditions (the max/min value seen so far).
 *
 * TODO: improve this heuristic for choosing how to process wall readings.
 */
void Driver::readWalls() {
    /* debug code */
    if (bluetoothOn_) {
        ble.print("Walls read at ");
        ble.print("x=");
        ble.println(curr_xpos);
        ble.print("y=");
        ble.println(curr_ypos);
    }
    /* end debug code */

    for (int i = 0; i < 3; i++) {
        if ((i == 1 && _sensors.readShortTof(i) < shortTofWallReadings[1]) ||
            (_sensors.readShortTof(i) > shortTofWallReadings[i]))
        {
            shortTofWallReadings[i] = _sensors.readShortTof(i);
        }
    }
}


/* clearWallData()
 * Zeroes out the array of wall readings to prepare for another movement.
 */
void Driver::clearWallData() {
    for (int i = 0; i < 3; i++) {
        shortTofWallReadings[i] = 0;
    }
    shortTofWallReadings[1] = 1000;
}


/* minTurn()
 * Calculates the right goal_angle to feed into `go` to get a minimal turn.
 * Because our current angle variable isn't restricted to the [0, 2PI) range,
 * we need to use values within PI of the current angle variable to get
 * minimal turns. Therefore we do this conversion to the proper bracket. */
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


void Driver::calculateInputPWM(bool angle_flag,
    float goal_x, float goal_y, float angle_diff)
{
    // use distance formula to get positive linear velocity
    float lin_velocity = angle_flag ? 0 :
        sqrt(pow(_pid_x.output, 2) + pow(_pid_y.output, 2));
    float ang_velocity = _pid_a.output;

    Serial.print("ang_velocity: ");
    Serial.println(ang_velocity);

    // L is width of robot
    // todo: makes sure ceiling doesnt drown out angle correction
    _v_left = lin_velocity - L * ang_velocity / 2;
    _v_right = lin_velocity + L * ang_velocity / 2;
    _v_left = ceilingPWM(_v_left, _v_right, motorLimit);
    _v_right = ceilingPWM(_v_right, _v_left, motorLimit);

    /*if (angle_flag){
        float mag = (fabs(_v_right) + fabs(_v_right)) / 2;
        _v_right = SIGN(_v_right) * mag;
        _v_left = -SIGN(_v_right) * mag;
    }*/

    if (!angle_flag &&
    (angle_diff <= PI / 2 || angle_diff >= 3 * PI / 2))
    {
        // moving forward - force motors forward
        if (_v_left + _v_right < 0) {
        _v_left = -1 * _v_left;
        _v_right = -1 * _v_right;
    }
    } else if (!angle_flag) {
        // moving backwards - force motors backwards
        if (_v_left + _v_right > 0) {
            _v_left = -1 * _v_left;
            _v_right = -1 * _v_right;
        }
    }
}


/* Moves based on absolute position on a coordinate grid */
void Driver::go(float goal_x, float goal_y, float goal_a, int refreshMs) {
    unsigned int interval = refreshMs;
    elapsedMillis timeElapsed = 1000;
    elapsedMillis bluetoothTimer = 0;
    elapsedMillis pidTimer = 0;
    elapsedMillis sensorTimer = 0;
    elapsedMillis fuckupTimer = 0;
    int sensorCounter = 0;

    if (debug) {
        Serial.print("Old goal_a: ");
        Serial.print(goal_a);
    }
    goal_a = minTurn(goal_a, curr_angle);
    if (debug) {
        Serial.print(" New goal_a: ");
        Serial.println(goal_a);
    }

    _leftMotor._encoder.write(0);
    _rightMotor._encoder.write(0);
    long enc_left = 0;
    long enc_right = 0;
    const float init_xpos = curr_xpos;
    const float init_ypos = curr_ypos;
    _pid_x.setpoint = fabs(goal_x - init_xpos);
    _pid_y.setpoint = fabs(goal_y - init_ypos);
    _pid_a.setpoint = goal_a;

    _pid_x.printTunings();

    int end_iter = 0;
    int overflow_count = floor(curr_angle / (2 * PI));
    bool angle_flag = goal_x == curr_xpos && goal_y == curr_ypos;

    debugPidMovement();

    do {
        /* stores sensor readings to detect walls
         * NOTE: will break if sensors not initialized */
        if ((goal_y != init_ypos || goal_x != init_xpos) &&
            sensorTimer > sensorRefreshTime &&
            sensorCounter < 3)
        {
            readWalls();
            sensorTimer = 0;
            sensorCounter++;
        }

        if (timeElapsed > interval) {
            // do all pid output related on a separate loop
            if (pidTimer > pidSampleTime) {
                pidTimer = 0;

                _pid_x.input = fabs(curr_xpos - init_xpos);
                _pid_y.input = fabs(curr_ypos - init_ypos);
                _pid_a.input = curr_angle;
                computePids();

                // (same calcuation as temp_a in tankGo)
                float travel_angle = atan2(
                    -1*(goal_x - curr_xpos), goal_y - curr_ypos);

                float angle_diff = fabs(fmod(curr_angle, 2 * PI) -
                    fmod(travel_angle, 2 * PI));

                calculateInputPWM(angle_flag, goal_x, goal_y, angle_diff);
                drive(_v_left, _v_right);

                /* Begin debug code */
                if (debug) {
                    Serial.print("Timer: ");
                    Serial.println(fuckupTimer);
                    debugPidMovement();
                }
                if (bluetoothOn_ && bluetoothTimer >= 1000) {
                    ble.print("_v_left: ");
                    ble.print(_v_left);
                    ble.print(" _v_right: ");
                    ble.println(_v_right);
                    bluetoothTimer = 0;
                }
                /* End debug code */

                /* If the movement looks like it's reached the goal position
                or it's converged, stop the movement */
                if ((withinError(goal_x, curr_xpos, errorX) &&
                     withinError(goal_y, curr_ypos, errorY) &&
                     withinError(goal_a, curr_angle, errorA)) ||
                    (_v_left < motorCloseEnough && _v_right < motorCloseEnough) ||
                    // perpendicular to goal direction means it's either
                    // right next to destination, or it's hopeless anyway
                    (!angle_flag &&
                        fabs(angle_diff - PI / 2) <= perpendicular_threshold))
                {
                    end_iter++;
                }
                else {
                    end_iter = 0;
                }
                if (end_iter > convergenceTime) {
                    break;
                }
            }

            // robot state updates
            float sample_t = 1. / interval;
            float true_v_left = (_leftMotor.readTicks() - enc_left) *
                ticksToCm / sample_t;
            enc_left = _leftMotor.readTicks();
            float true_v_right = (_rightMotor.readTicks() - enc_right) *
                ticksToCm / sample_t;
            enc_right = _rightMotor.readTicks();

            float true_ang_v = (true_v_right - true_v_left) / L;
            curr_xpos += (true_v_left + true_v_right) / 2  *
                sample_t * -1 * sin(curr_angle);
            curr_ypos += (true_v_left + true_v_right) / 2 *
                sample_t * cos(curr_angle);
            float imu_rads = (360 - _sensors.readIMUAngle()) * degToRad;

            /* used for if the current angle `a` > 2PI or `a` < 0 to correct
            the IMU angle. */
            float overflow = overflow_count * 2 * PI;
            if (fabs(curr_angle - (imu_rads + overflow)) > PI) {
                if (curr_angle > imu_rads + overflow) {
                    overflow_count++;
                }
                else {
                    overflow_count--;
                }
            }
            curr_angle = angWeight * (imu_rads + overflow_count * 2 * PI) +
                (1-angWeight) * (curr_angle + true_ang_v * sample_t);
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
 // goal_a parameter obselete!!
void Driver::tankGo(float goal_x, float goal_y, float goal_a) {
    float temp_a = atan2(-1*(goal_x - curr_xpos), goal_y - curr_ypos);

    if (bluetoothOn_) {
        ble.print("goal_a: ");
        ble.print(goal_a);
        ble.print(" temp_a: ");
        ble.println(temp_a);
    }
    else if (debug) {
        Serial.print("goal_a: ");
        Serial.print(goal_a);
        Serial.print(" temp_a: ");
        Serial.println(temp_a);
    }

    if (fabs(temp_a - curr_angle) > PI / 12) {
        // Turn
        Serial.println(temp_a);
        go(curr_xpos, curr_ypos, temp_a);
        // Go forward
        go(goal_x, goal_y, temp_a);
    }
    else {
        go(goal_x, goal_y, temp_a);
    }

    // Re-align if near the wall
    if (shortTofWallReadings[1] < 80 &&
        !withinError(_sensors.readShortTof(1), alignDist, 2)) {
        realign(20);
    }
}


void Driver::resetState() {
    this->curr_xpos = 0;
    this->curr_ypos = 0;
}


void Driver::realign(int goal_dist) {
    const int wall_error = 3;
    _pid_front_tof.setpoint = goal_dist;
    int counter = 0;
    while (1) {
        float dist = _sensors.readShortTof(1);
        // end condition
        if (dist < goal_dist + wall_error && dist > goal_dist - wall_error) {
            counter++;
        }
        else {
            counter = 0;
        }

        if (counter >= convergenceTime) {
            break;
        }

        _pid_front_tof.input = dist;
        _pid_front_tof.compute();
        float pwm = -1 * _pid_front_tof.output;
        drive(pwm, pwm);
    }
    brake();

    // correct state based on which wall it realigned on
    int direction = round(fmod(curr_angle, 2 * PI) / (PI / 2));

    // Pointing east or west -> x-axis
    if (direction % 2 == 1) {
        int current_col = round(curr_xpos / cellSize);
        curr_xpos = current_col * cellSize;
    }
    else {
        int current_row = round(curr_ypos / cellSize);
        curr_ypos = current_row * cellSize;
    }
}
