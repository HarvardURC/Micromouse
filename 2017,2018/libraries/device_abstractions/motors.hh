#include <Encoder.h>
#include <PID_v1.h>
#include "sensors.hh"

#ifndef motors_hh
#define motors_hh

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
            SensorArray sensors
        );

        void drive(int speed);
        void drive(int speedLeft, int speedRight);

        // Moves until the motor for input ticks
        void moveTicks(long ticks);
        void moveTicksPID(long ticks);

        void turnDegrees(float degrees);

        void movePID(float setpoint);

        void go(float goal_x, float goal_y, float goal_a, int refreshMs = 1);

        void resetState();

        void forward(float distance);
        void turnLeft(float degrees);
        void turnRight(float degrees);
        void brake();


        // Robot state variables
        float curr_xpos;
        float curr_ypos;
        float curr_angle;

    private:
        Motor _leftMotor;
        Motor _rightMotor;
        SensorArray _sensors;
};

#endif