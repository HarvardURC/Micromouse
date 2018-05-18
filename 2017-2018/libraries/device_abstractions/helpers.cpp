#include <Arduino.h>
#include <Encoder.h>
#include "bluetooth.hh"
#include "helpers.hh"
#include "software_config.hh"

using namespace swconst;

#define SIGN(x) 2 * (x > 0) - 1

/* Enforces a minimum PWM input to the motors.
 *
 * `speed` is a raw input PWM value for a motor
 * `floor` is a minimum value that is desirable to send to the motor
 * Returns 0 if the `speed` is lower than the `floor` */
int floorPWM(int speed, int floor) {
    return fabs(speed) >= floor ? fabs(speed) * SIGN(speed) : 0;
}


/* Enforces a maximum PWN input to the motors
 * (scales 2 speeds down so both are at/under the limit)
 * and maintains the same ratio between the two speeds.
 *
 * `speed` is the input PWM motor speed to consider.
 * `otherspeed is the speed of the other motor to compare it to.
 * `limit` is the maximum acceptable PWM values for the motor.
 * Returns the new scaled input PWM value for `speed`. */
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


/* Checks if the two numbers are within a margin of error from each other
 * E.g. Is 4 within a degree of error 3 from 6?
 * fabs(4-6) = 2 < 3 => TRUE */
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
    if (bleReady()) {
        ble.print("Proportion: ");
        ble.print(this->proportion);
        ble.print(" Integral: ");
        ble.print(this->integral);
        ble.print(" Derivative: ");
        ble.println(this->derivative);
    }
}


EncoderTicker::EncoderTicker(Encoder* e_) {
    this->e = e_;
    this->lastVal = 0;
    e->write(0);
}

void DriverConfig::print() {
    debug_println("Current driver config:");
    debug_printvar(motorLimit);
    debug_printvar(convergenceTime);
}
