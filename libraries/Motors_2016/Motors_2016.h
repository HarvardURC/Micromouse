/**
 * Motors_2016.h
 * Contains various methods for using the DC motors.
 */

#ifndef Motors_2016_h
#define Motors_2016_h



class Motors_2016
{
    public:
        // Constructor
        Motors_2016(int powerPinL, int directionPinL, int powerPinR,
               int directionPinR, int encoderPinL1, int encoderPinL2,
               int encoderPinR1, int encoderPinR2, int forwardIRPin, 
               int leftIRPin, int rightIRPin, int leftDiagIRPin,
               int rightDiagIRPin);
        /* Move forward one cell (will later incorporate sensors) */
        void forward();
        /* note: following functions must loop for appropriate time
           in addition to setting desired setpoint positions */
        // 90 deg turns
        void turnLeft();
        void turnRight();
        /* Align robot in middle of cell (distance from front wall) */
        void front_align();
    
    private:
        // time
        unsigned long time;
        // all them pins (check if should use smaller types)
        int _powerPinL;
        int _directionPinL;
        int _powerPinR;
        int _directionPinR;
        int _encoderPinL1;
        int _encoderPinL2;
        int _encoderPinR1;
        int _encoderPinR2;
        int _forwardIRPin;
        int _leftIRPin;
        int _rightIRPin;
        int _leftDiagIRPin;
        int _rightDiagIRPin;
        // pid variables
        double SetpointL, InputL, OutputL;
        double SetpointR, InputR, OutputR;
        // helper functions
        void followTicksRight(int ticks);
        //void followTicksLeft(int ticks);
        void moveTicks(int Lticks, int Rticks);
        void commandMotors(double left, double right);
        void stop();
        void wait(int millis);
};

#endif

