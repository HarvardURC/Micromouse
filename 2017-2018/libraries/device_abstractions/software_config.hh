#ifndef s_config_hh
#define s_config_hh

namespace swconst {
    /* PID controller  constants */
    const int pidSampleTime = 5; // parameter to PID library
    const float pidLimit = 50.0; // upperlimit on PID output
    const float pidLoopTimeout = 2000; // global timeout ms on pid loops

    // motor/encoder PID (ONLY FOR MOTOR CLASS FUNCTIONS)
    const float p_m = 0.002, i_m = 0, d_m = 0;

    /* Error thresholds */
    const int encoderError = 5000; // error threshold for the encoder values

    const float errorX = 0.7; // centimeters
    const float errorY = errorX;
    const float errorA = 0.3; // radians

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
    const int tof_low_bound = 10;
    const int tof_high_bound = 50;
    const int front_wall_threshold = 90;
    const float wall_follow_dist = 20.;
    const float distance_limit = 18; // if ignoring wall, ignore for 12cm

    // front alignment desired distance
    const int front_wall_align = 18;
    const int wall_error = 3; // error in sensor for reading wall
    const int front_threshold = 30; // maximum distance to start aligning
    const float diag_correction = 3; // difference between left and right diags
    const float angle_correction_ratio = .4; // how much to update angular state on frontalign

    // back alignment Constants
    const int backAlignPWM = -55;
    const float backedOffset = distCenterToInnerWall - 2;

    // thresholds for the 4 directions: TUNE THESE ON COMPETITION DAY
    const int irThresholds[4] = {90, 90, 0, 90}; // front, left, back, right
    const int numWallChecks = 4; // number of times to check sensors for walls

    /* Mapping phase constants */
    const int convergenceTimeM0 = 30; // milliseconds
    const int motorLimitM0 = 50; // highest motor PWM value

    /* Mapping phase PID vals */
    const float p_l_M0 = 12, i_l_M0 = 0, d_l_M0 = 0.05; // linear PIDs, x and y position
    const float p_a_M0 = 16, i_a_M0 = 0, d_a_M0 = 0.1; // angle PID

    const float p_l_M0T = 22, i_l_M0T = 0.5, d_l_M0T = 0.1; // linear PID ONLY FOR TURNS
    const float p_a_M0T = 28, i_a_M0T = 0.5, d_a_M0T = 0.1; // angle PID ONLY FOR TURNS

    const float p_tof = 10, i_tof = 0, d_tof = 0.1; // front ToF sensor PID
    const float p_diag = 150, i_diag = 0, d_diag = 1; // diagonal ToF sensors PID

    /* Speedrun 1 constants */
    const int convergenceTimeS1 = 20;
    const int motorLimitS1 = 60;

    const float p_l_S1 = 12, i_l_S1 = 0.5, d_l_S1 = 0.15; // linear PIDs, x and y position
    const float p_a_S1 = 14, i_a_S1 = 0.5, d_a_S1 = 0.4; // angle PID
}

#endif
