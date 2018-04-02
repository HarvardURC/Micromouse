#ifndef s_config_hh
#define s_config_hh

namespace swconst {
    /* PID constants */
    const int pidSampleTime = 5; // parameter to PID library
    const float pidLimit = 50.0; // upperlimit on PID output

    /* Error thresholds */
    const int encoderError = 5000; // error threshold for the encoder values

    const float errorX = 0.9; // centimeters .9
    const float errorY = errorX;
    const float errorA = 0.2; // radians .2

    const float perpendicularError = 0.5; // range to 90 degrees to terminate move

    /* Timing constants */
    const unsigned long timeout = 10000; // 10 seconds
    const int sensorRefreshTime = 120; // milliseconds
    const int convergenceTime = 20; // milliseconds

    /* Distance measurements */
    const float cellSize = 18; // size in cm of cell

    const float L = 9.25; // wheel distance in `cm`
    const float frontSensorToWheelAxis = 4.75; // cm

    // the distance from the center of the cell to the wall where sensor reads
    const float distCenterToInnerWall = 8.5;

    // how far from wall to align to be in the center of the cell in cm
    const float alignDist = distCenterToInnerWall - frontSensorToWheelAxis;

    /* Ratios */
    const float ticksToCm = 1. / 8300; // conversion ratio 8360
    const float degToRad = PI / 180; // converstion ratio

    /* PWM limits */
    const int motorLimit = 50; // highest motor PWM value
    const int motorFloor = 20; // lowest motor PWM value (IN TUNING)
    const int motorCloseEnough = 20; // motor value considered close to target (IN TUNING) 15

    /* Sensor constants */
    const float imu_weight = 0;                     // ratio of IMU
    const float encoder_weight = 1;                 // vs. encoder measurements
    const float rangefinder_weight = 0;             // vs. rangefinder measurement for angle

}

#endif
