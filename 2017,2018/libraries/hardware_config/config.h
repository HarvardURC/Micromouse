/**
 * config.h
 * Contains hardware constants for the 2017-2018 experienced-team robot.
 */

#ifndef config_h
#define config_h

namespace pins
{
    // LEDs
    const int frontLedR = 25;
    const int frontLedG = 24;
    const int frontLedB = 26;

    const int backLedR = 31;
    const int backLedG = 30;
    const int backLedB = 32;

    const int cpuLed = 13;

    // motors
    const int motorPowerL = 10;
    const int motorDirectionL = 9;
    const int motorPowerR = 8;
    const int motorDirectionR = 7;
    const int motorMode = 6;

    // encoders
    const int encoderPinL1 = 3;
    const int encoderPinL2 = 2;
    const int encoderPinR1 = 4;
    const int encoderPinR2 = 5;

    // sensors
    const int tofFrontS = 35;
    const int tofRightDiagS = 36;
    const int tofLeftDiagS = 39;
    const int tofFrontL = 22;

    // clock -- i2c
    const int tofSCL = 19;
    // data -- i2c
    const int tofSDA = 18;

    // angular sensor
    // enable pin
    const int imuRST = 33;
    // clock pin
    const int imuSCL = 37;
    // data pin
    const int imuSDA = 38;

    // Bluetooth SPI bus
    const int MISO = 12;
    const int MOSI = 11;
    const int SCK = 14;
    const int CS = 15;
    const int bluetoothRST = 27;
    const int bluetoothIRQ = 28;

    // button
    const int frontButton = 21;
    const int backButton = 20;

    // buzzer
    const int buzzer = 29;

}

#endif
