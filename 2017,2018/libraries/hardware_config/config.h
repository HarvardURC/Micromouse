/**
 * config.h
 * Contains hardware constants for robot.
 */

#ifndef config_h
#define config_h

namespace pins
{
    const int led = 13;
    // motors
    const int motorPowerR = 6; // B
    const int motorPowerL = 4; // A
    const int motorDirectionR = 5; // B2
    const int motorDirectionL = 3; // A
    // ToF sensors
    const int tofRight = 17;
    const int tofRightDiag = 20;
    const int tofFront = 21;
    const int tofLeftDiag = 22;
    const int tofLeft = 23;
    // encoders
    const int encoderPinL1 = 1;
    const int encoderPinL2 = 2;
    const int encoderPinR1 = 7;
    const int encoderPinR2 = 8;
    // button
    const int buttonS6 = 0;
    const int buttonS8 = 9;
}

#endif
