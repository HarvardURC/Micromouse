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
        double getPIDSpeed();

        void testPID();

    private:
        int _powerPin;
        int _directionPin;

        Encoder _encoder;
        PID _pid;
        SensorArray _sensors;

        double _pidSetpoint;
        double _pidInput;
        double _pidOutput;
        double _pidProportion = 0.001;
        double _pidIntegral = 0.000000001;
        double _pidDerivative = 0.000000001;
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

        void turnDegrees(double degrees);

        void movePID(double setpoint);

    private:
        Motor _leftMotor;
        Motor _rightMotor;
        SensorArray _sensors;
};

#endif
