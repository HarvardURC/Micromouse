#include <Encoder.h>
#include <vector>
#include "sensors.hh"
#include "helpers.hh"
#include "io.hh"

#ifndef motors_hh
#define motors_hh

// class for single motor + encoder
class Motor {
    public:
        // Constructor
        Motor(
            int powerPin,
            int directionPin,
            int encoderPin1,
            int encoderPin2,
            SensorArray sensors
        );

        void drive(int speed);

        // Reads the current encoder value
        long readTicks();

        /* Note: Main control logic in Driver class (go, tankGo)
           does not use the following motor functions */
        // Moves until the motor for input ticks
        void moveTicks(long ticks);
        void moveTicksPID(long ticks);

        // Gets the output speed for a motor based on the PID values
        float getPIDSpeed(float setpoint);

        void testPID();

        int _powerPin;
        int _directionPin;

        Encoder _encoder;
        PIDT<float> _pid;
        SensorArray _sensors;

        float _pidSetpoint;
        float _pidInput = 0;
        float _pidOutput = 0;
        float _pidProportion = 0.01;
        float _pidIntegral = 0;//.000000001;
        float _pidDerivative = 0;//.000000001;
};

// class for both motors/encoders - contains most of the control logic
class Driver {
    public:
        // Constructor
        Driver(
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
            RGB_LED rgb
        );

        // Moves the motors forward at the input PWM value. PWMs to motors
        // are floored so they cannot write values that won't move the wheels.
        void drive(int speed);
        void drive(int speedLeft, int speedRight);

        // Moves until the motor for input ticks
        void moveTicks(long ticks);
        void movePID(float setpoint);

        // generate PID outputs for go function
        void computePids(float init_xpos, float init_ypos,
                         float unbounded_angle);
        // use PID outputs and goal orientation to determine motor power
        void calculateInputPWM(bool angle_flag,
            float goal_x, float goal_y, float angle_diff);

        // Clears the robot state variables
        void resetState();
        // Prints out the output, setpoint, and state variables for each pid
        void debugPidMovement(float angle);
        void debugAngle(float angle);

        int heading(float goal_x, float goal_y);

        void clearWallData();

        // single move to absolution coordinates
        void go(float goal_x, float goal_y, float goal_a, size_t interval = 1, bool backwards = false);
        // main navigation function, move via tank turn and straight drive
        void tankGo(float goal_x, float goal_y, bool back_wall = false, bool backwards = false);
        // realign on a front wall using distance sensors
        void realign(int goal_dist);
        // align on back wall by physically backing up
        void backAlign();

        // these functions use the go function to move relative distances/angles
        void forward(float distance);
        void turnLeft(float degrees);
        void turnRight(float degrees);
        void brake();

        // switch to a new set of constants
        void updateConfig(DriverConfig cfg);

        // Robot state variables
        float curr_xpos;
        float curr_ypos;
        float curr_angle;

        // the maximum input PWM to drive motors, changes for speedruns
        float cfgNum;
        float motorLimit;
        int convergenceTime;

        bool encoderOnlyFlag;
        bool imuOn;

        // left, middle, right
        long shortTofWallReadings[4];

        PidController _pid_x;
        PidController _pid_y;
        PidController _pid_a;
        PidController _pid_front_tof;
        PidController _pid_diag_tof;

    private:
        void readWalls();

        Motor _leftMotor;
        Motor _rightMotor;
        SensorArray _sensors;
        RGB_LED _rgb;

        float _v_left = 0;
        float _v_right = 0;
};

extern std::vector<DriverConfig> driverCfgsLinear;

#endif
