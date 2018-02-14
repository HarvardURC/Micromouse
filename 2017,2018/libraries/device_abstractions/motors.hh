#include <Encoder.h>
#include <PID_v1.h>

#ifndef motors_hh
#define motors_hh

class Motor {
    public:
        // Constructor
        Motor(int powerPin, int directionPin, int encoderPin1, int encoderPin2);

        void drive(int speed);

        // Reads the current encoder value
        long readTicks();

        // Moves until the motor for input ticks
        void moveTicks(long ticks);
        void moveTicksPID(long ticks);

        // Gets the output speed for a motor based on the PID values
        double getPIDSpeed();

        double pidSetpoint;

    private:
        int _powerPin;
        int _directionPin;

        Encoder _encoder;
        PID _pid;
        double _pidInput;
        double _pidOutput;
        double _pidProportion = 1;
        double _pidIntegral = 0.0018;
        double _pidDerivative = 0.01;
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
            int encoderPinR2
        );

        void drive(int speed);

        // Moves until the motor for input ticks
        void moveTicks(long ticks);
        void moveTicksPID(long ticks);

    private:
        Motor _leftMotor;
        Motor _rightMotor;
};

#endif
