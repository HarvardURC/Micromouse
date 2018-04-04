#include <Arduino.h>
#include <elapsedMillis.h>
#include <math.h>
#include "motors.hh"
#include "bluetooth.hh"
#include "software_config.hh"

using namespace swconst;

const bool debug = false; // set to true for serial debugging statements

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
        debug_print("Encoder Value for motorpin ");
        debug_print(_powerPin);
        debug_print(" ");
        debug_println(_encoder.read());
        drive(ticks > 0 ? speed : -1 * speed);
    }
    drive(0);
}


// Returns the speed the motor should go according to the PID to acheive the
// setpoint.
float Motor::getPIDSpeed(float setpoint) {
    _pidSetpoint = setpoint;
    _pidInput = _encoder.read();
    debug_print("Encoder: ");
    debug_println(_pidInput);
    _pid.Compute();
    return _pidOutput;
}


// Moves the ticks specified (mesaured by the encoder, and assisted by the PID)
void Motor::moveTicksPID(long ticks) {
    _pidSetpoint = ticks;
    _pidInput = 0;
    _encoder.write(0);
    long time = millis();
    while (abs(_pidSetpoint - _pidInput) > encoderError &&
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
    SensorArray sensors) :
    _leftMotor(powerPinL, directionPinL, encoderPinL1, encoderPinL2, sensors),
    _rightMotor(powerPinR, directionPinR, encoderPinR1, encoderPinR2, sensors),
    _sensors(sensors),
    _pid_x(p_l, i_l, d_l),
    _pid_y(p_l, i_l, d_l),
    _pid_a(p_a, i_a, d_a),
    _pid_front_tof(p_tof, i_tof, d_tof),
    _pid_diag_tof(p_diag, i_diag, d_diag)
{
    curr_xpos = 0.0;
    curr_ypos = 0.0;
    curr_angle = 0.0;
    motorLimit = motorLimitM0;
    convergenceTime = convergenceTimeM0;
    pinMode(motorModePin, OUTPUT);
    digitalWrite(motorModePin, HIGH);

    clearWallData();

    if (imu_weight + encoder_weight + rangefinder_weight != 1) {
        debug_println("Angular Weights Do Not Add to 1");
        abort();
    }
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
    movePID(fmod((initialAngle + degrees), 360) - 180);
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
            debug_print("Left motor speed: ");
            debug_print(leftSpeed);
            debug_print(" Right motor speed: ");
            debug_print(rightSpeed);
            debug_print(" Setpoint: ");
            debug_println(setpoint);
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


void Driver::computePids(float init_xpos, float init_ypos,
    float angle_travelled) {
    _pid_x.input = fabs(curr_xpos - init_xpos);
    _pid_y.input = fabs(curr_ypos - init_ypos);
    _pid_a.input = angle_travelled;

    _pid_x.compute();
    _pid_y.compute();
    _pid_a.compute();
}


void Driver::debugPidMovement(float angle_travelled) {
    debug_printvar(_pid_x.input);
    debug_printvar(_pid_x.output);
    debug_printvar(_pid_x.setpoint);
    debug_printvar(curr_xpos);
    debug_println(" ");

    debug_printvar(_pid_y.input);
    debug_printvar(_pid_y.output);
    debug_printvar(_pid_y.setpoint);
    debug_printvar(curr_ypos);
    debug_println(" ");

    this->debugAngle(angle_travelled);

    debug_printvar(_v_left);
    debug_printvar(_v_right);
    debug_println(" ");
}

void Driver::debugAngle(float angle_travelled) {
    debug_printvar(_pid_a.input);
    debug_printvar(_pid_a.output);
    debug_printvar(_pid_a.setpoint);
    debug_printvar(angle_travelled);
    debug_printvar(curr_angle);
    debug_println(" ");
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
    if (debug) {
        debug_print("Walls read at ");
        debug_print("x=");
        debug_print(curr_xpos);
        debug_print(" y=");
        debug_println(curr_ypos);
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
    goal_angle = wrapAngle(goal_angle);
    return wrapAngle(goal_angle - curr_angle + PI) - PI;
}


void Driver::calculateInputPWM(bool angle_flag,
    float goal_x, float goal_y, float angle_diff)
{
    // use distance formula to get positive linear velocity
    float lin_velocity = angle_flag ? 0 :
        sqrt(pow(_pid_x.output, 2) + pow(_pid_y.output, 2));
    float ang_velocity = _pid_a.output;

    // debug_printvar(ang_velocity);

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
void Driver::go(float goal_x, float goal_y, float goal_a, size_t interval) {
    float sample_t = 1. / interval;
    elapsedMillis timeElapsed = 1000;
    elapsedMillis bluetoothTimer = 0;
    elapsedMillis pidTimer = 0;
    elapsedMillis sensorTimer = 0;
    elapsedMillis timer = 0;
    int sensorCounter = 0;

    if (debug) {
        debug_print("Old ");
        debug_printvar(goal_a);
    }
    // between -PI to PI
    goal_a = minTurn(goal_a, curr_angle);
    if (debug) {
        debug_print("New ");
        debug_printvar(goal_a);
        debug_printvar(curr_angle);
    }

    EncoderTicker leftEnc(&_leftMotor._encoder);
    EncoderTicker rightEnc(&_rightMotor._encoder);

    const float init_xpos = curr_xpos;
    const float init_ypos = curr_ypos;
    const float init_angle = curr_angle;
    _pid_x.setpoint = fabs(goal_x - init_xpos);
    _pid_y.setpoint = fabs(goal_y - init_ypos);
    _pid_a.setpoint = goal_a;

    int end_iter = 0;
    bool angle_flag = goal_x == curr_xpos && goal_y == curr_ypos;
    float angle_travelled = 0;
    bool negative_turn = goal_a < 0;

    do {
        /* stores sensor readings to detect walls
         * NOTE: will break if sensors not initialized */
        if ((goal_y != init_ypos || goal_x != init_xpos) &&
            sensorTimer > sensorRefreshTime &&
            sensorCounter < 3)
        {
            sensorTimer = 0;
            sensorCounter++;
            readWalls();
        }

        if (timeElapsed > interval) {
            // reset sample time
            timeElapsed = 0;

            // do all pid output related on a separate loop
            if (pidTimer > pidSampleTime) {
                pidTimer = 0;

                computePids(init_xpos, init_ypos, angle_travelled);

                // (same calcuation as temp_a in tankGo)
                float travel_angle = atan2(
                    -1*(goal_x - curr_xpos), goal_y - curr_ypos);

                float angle_diff = fabs(wrapAngle(curr_angle) -
                    wrapAngle(travel_angle));

                calculateInputPWM(angle_flag, goal_x, goal_y, angle_diff);
                drive(_v_left, _v_right);

                if (debug && bluetoothTimer >= 1000) {
                    // debug_printvar(_v_left);
                    // debug_printvar(_v_right);
                    debug_println(" ");
                    debugAngle(angle_travelled);
                    debug_printvar(init_angle);
                    bluetoothTimer = 0;
                }

                /* If the movement looks like it's reached the goal position
                or it's converged, stop the movement */
                if ((withinError(goal_x, curr_xpos, errorX) &&
                     withinError(goal_y, curr_ypos, errorY) &&
                     withinError(goal_a, angle_travelled, errorA)) ||
                    (_v_left < motorCloseEnough && _v_right < motorCloseEnough) ||
                    // perpendicular to goal direction means it's either
                    // right next to destination, or it's hopeless anyway
                    (!angle_flag &&
                        fabs(angle_diff - PI / 2) <= perpendicularError))
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

            /* Update positional state, curr_xpos and curr_ypos */
            float true_v_left = leftEnc.diffLastRead() *
                ticksToCm / sample_t;
            float true_v_right = rightEnc.diffLastRead() *
                ticksToCm / sample_t;

            curr_xpos += (true_v_left + true_v_right) / 2  *
                sample_t * -1 * sin(curr_angle);
            curr_ypos += (true_v_left + true_v_right) / 2 *
                sample_t * cos(curr_angle);

            // integrates rangefinder offset
            if (!angle_flag) {
                float left_diag_dist = _sensors.readShortTof(0);
                float right_diag_dist = _sensors.readShortTof(2);
                if (left_diag_dist >= 20 && left_diag_dist <= 70) {
                    if (right_diag_dist >= 20 && right_diag_dist <= 70) {
                        // curr_xpos = curr_xpos + 9*(left_diag_dist/right_diag_dist-1);
                    }
                }
                // debug_println(curr_xpos);
            }

            /* Update angular state, curr_angle */
            float true_ang_v = (true_v_right - true_v_left) / L;
            float imu_rads = (360 - _sensors.readIMUAngle()) * degToRad;

            float angle_change = imu_weight * imu_rads +
                encoder_weight * (true_ang_v * sample_t) +
                rangefinder_weight;

            curr_angle = wrapAngle(curr_angle + angle_change);

            // the current angle wrapped from 0 to 2PI
            angle_travelled += angle_change;
        }
    } while (1);

    brake();
    if (debug) debug_println("Done with movement.");
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
// todo remove goal_a
void Driver::tankGo(float goal_x, float goal_y, float goal_a) {
    float temp_a = atan2(-1*(goal_x - curr_xpos), goal_y - curr_ypos);

    if (debug) {
        debug_printvar(temp_a);
    }

    if (fabs(temp_a - curr_angle) > PI / 12) {
        // Turn
        debug_println(temp_a);
        go(curr_xpos, curr_ypos, temp_a);
        // Go forward
        go(goal_x, goal_y, temp_a);
    }
    else {
        go(goal_x, goal_y, temp_a);
    }

    // Re-align if near the wall
    if (_sensors.readShortTof(1) < 80 &&
        !withinError(_sensors.readShortTof(1), 18, 2)) {
        realign(18);
    }
}


void Driver::resetState() {
    this->curr_xpos = 0;
    this->curr_ypos = 0;
    this->curr_angle = 0;
}


void Driver::realign(int goal_dist) {
    const int wall_error = 3;
    const int front_threshold = 30;
    const int diag_correction = 4;
    _pid_front_tof.setpoint = goal_dist;
    // right diag reads less than left diag
    _pid_diag_tof.setpoint = diag_correction;
    int counter = 0;
    while (1) {
        float front_dist = _sensors.readShortTof(1);
        float left_diag_dist = _sensors.readShortTof(0);
        float right_diag_dist = _sensors.readShortTof(2);

        float diag_diff = left_diag_dist - right_diag_dist;

        // end condition
        if (withinError(front_dist, goal_dist, wall_error) &&
            withinError(diag_diff, diag_correction, 6)) {
            counter++;
        }
        else {
            counter = 0;
        }

        if (counter >= convergenceTime) {
            break;
        }

        _pid_front_tof.input = front_dist;
        _pid_diag_tof.input = front_dist < front_threshold ? diag_diff : 0;
        _pid_front_tof.compute();
        _pid_diag_tof.compute();

        float pwm = -1 * _pid_front_tof.output;
        float angle_correction = _pid_diag_tof.output;
        drive(pwm - angle_correction, pwm + angle_correction);
    }
    brake();

    // correct state based on which wall it realigned on
    int direction = round(curr_angle) / (PI / 2);

    // Pointing east or west -> x-axis
    if (direction % 2 == 1) {
        int current_col = round(curr_xpos / cellSize);
        curr_xpos = current_col * cellSize;
    }
    else {
        int current_row = round(curr_ypos / cellSize);
        curr_ypos = current_row * cellSize;
    }

    // float new_angle = direction * PI / 2;
    // curr_angle += (new_angle - curr_angle)
}
