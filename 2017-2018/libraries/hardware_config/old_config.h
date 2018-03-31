/**
 * old_config.h
 * Contains hardware constants for the 2016-2017 robot.
 */

#ifndef config_h
#define config_h

namespace pins
{
    const int led = 13;
    // motors
    const int motorPowerR = 6;
    const int motorPowerL = 9;
    const int motorDirectionL = 10;
    const int motorDirectionR = 12;
    // ToF sensors
    const int tofRight = 19;
    const int tofRightDiag = 20;
    const int tofFront = 21;
    const int tofLeftDiag = 22;
    const int tofLeft = 23;
    // encoders
    const int encoderPinL1 = 7;
    const int encoderPinL2 = 8;
    const int encoderPinR1 = 2;
    const int encoderPinR2 = 1;
    // button
    const int button = 29;
}

#endif
