#ifndef s_config_hh
#define s_config_hh

namespace swconst {
    /* PID controller  constants */
    const int pidSampleTime = 5; // parameter to PID library
    const float pidLimit = 50.0; // upperlimit on PID output

    // motor/encoder PID (ONLY FOR MOTOR CLASS FUNCTIONS)
    const float p_m = 0.002, i_m = 0, d_m = 0;

    /* Error thresholds */
    const int encoderError = 5000; // error threshold for the encoder values

    const float errorX = 0.9; // centimeters
    const float errorY = errorX;
    const float errorA = 0.1; // radians

    const float perpendicularError = 0.5; // range to 90 degrees to terminate move

    /* Timing constants */
    const unsigned long timeout = 10000; // 10 seconds
    const int sensorRefreshTime = 120; // milliseconds

    /* Distance measurements */
    const float cellSize = 18; // size in cm of cell

    const float L = 9.25; // wheel distance in `cm`
    const float frontSensorToWheelAxis = 4.75; // cm

    // the distance from the center of the cell to the wall where sensor reads
    const float distCenterToInnerWall = 8.4;

    // how far from wall to align to be in the center of the cell in cm
    const float alignDist = distCenterToInnerWall - frontSensorToWheelAxis;

    /* Ratios */
    const float ticksToCm = 1. / 8300; // conversion ratio 8360
    const float degToRad = PI / 180; // converstion ratio

    /* PWM limits */
    const int motorFloor = 25; // lowest motor PWM value (IN TUNING)
    const int motorCloseEnough = 20; // motor value considered close to target (IN TUNING) 15

    /* Sensor constants */
    const float imu_w = 0;                     // ratio of IMU
    const float encoder_w = 0;                 // vs. encoder measurements
    const float rangefinder_w = 1;             // vs. rangefinder measurement for angle

    const float nowall_imu_w = 0;
    const float nowall_encoder_w = 1;
    const float nowall_rangefinder_w = 0;

    // wall-following constants: TUNE THESE ON COMPETITION DAY
    const int tof_low_bound = 20;
    const int tof_high_bound = 100;
    const int front_wall_threshold = 80;
    const float wall_follow_dist = 20.;
    const float distance_limit = 17; // if ignoring wall, ignore for 12cm

    // front alignment desired distance
    const int front_wall_align = 18;
    const int wall_error = 3; // error in sensor for reading wall
    const int front_threshold = 30; // maximum distance to start aligning
    const int diag_correction = 4; // difference between left and right diags

    // thresholds for the 4 directions: TUNE THESE ON COMPETITION DAY
    const int irThresholds[4] = {90, 90, 0, 90};
    const int numWallChecks = 3; // number of times to check sensors for walls

    /* Mapping phase constants */
    const int convergenceTimeM0 = 20; // milliseconds
    const int motorLimitM0 = 55; // highest motor PWM value

    /* Mapping phase PID vals */
    const float p_l = 12, i_l = 0.5, d_l = 0.15; // linear PIDs, x and y position
    const float p_a = 20, i_a = 0.5, d_a = 0.1; // angle PID
    const float p_tof = 12, i_tof = 0, d_tof = 0; // front ToF sensor PID
    const float p_diag = 3, i_diag = 0, d_diag = 0; // diagonal ToF sensors PID

    /* Speedrun 1 constants */
    const int convergenceTimeS1 = 10;
    const int motorLimitS1 = 70;
}

#endif
