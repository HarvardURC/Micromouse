#include <Encoder.h>
#include <PID_v1.h>
#include "sensors.hh"

#ifndef motors_hh
#define motors_hh


class PidController {
    public:
        PidController(
            float proportion,
            float integral,
            float derivative
        );

        void compute();

        float input;
        float output;
        float setpoint;
    private:
        PIDT<float> _pid;
};


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
            bool bluetoothOn=false
        );

        // Moves the motors forward at the input PWM value. PWMs to motors
        // are floored so they cannot write values that won't move the wheels.
        void drive(int speed);
        void drive(int speedLeft, int speedRight);

        // Moves until the motor for input ticks
        void moveTicks(long ticks);
        void moveTicksPID(long ticks);

        void turnDegrees(float degrees);

        void movePID(float setpoint);

        void computePids();

        // Directs to the absolute position at goal_x, goal_y with an
        // angle of goal_a wrt the x-axis
        void go(float goal_x, float goal_y, float goal_a, int refreshMs = 1);
        void tankGo(float goal_x, float goal_y, float goal_a);
        // sets _v_left and _v_right variables based on x, y, a PIDs
        void calculateInputPWM(bool angle_flag,
            float goal_x, float goal_y, float angle_diff);
        // Clears the robot state variables
        void resetState();
        // Prints out the output, setpoint, and state variables for each pid
        void debugPidMovement();

        void clearWallData();

        // Tank turn movement functions
        void forward(float distance);
        void turnLeft(float degrees);
        void turnRight(float degrees);
        void brake();


        // Robot state variables
        float curr_xpos;
        float curr_ypos;
        float curr_angle;

        // left, middle, right
        long shortTofWallReadings[3];

    private:
        void readWalls();

        Motor _leftMotor;
        Motor _rightMotor;
        SensorArray _sensors;
        PidController _pid_x;
        PidController _pid_y;
        PidController _pid_a;
        bool bluetoothOn_;

        float _v_left = 0;
        float _v_right = 0;
};

#endif
