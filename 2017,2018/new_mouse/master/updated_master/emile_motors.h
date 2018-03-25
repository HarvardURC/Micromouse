/**
 * Motors_2016.h
 * Contains various methods for using the DC motors.
 */

#ifndef emile_motors_h
#define emile_motors_h

#include "VL6180X.h"

class emile_motors
{
    public:
        // Constructor
        emile_motors(VL6180X* leftIR, VL6180X* rightIR, VL6180X* frontIR,
                    VL6180X* leftDiagIR, VL6180X* rightDiagIR);
        // flag for reset button
        int releaseFlag;
        /* Move forward one cell */
        void forward();
        /* note: following functions must loop for appropriate time
           in addition to setting desired setpoint positions */
        // 90 deg turns
        void turnLeft();
        void turnRight();
        // 180 deg turns
        void turnAroundLeft();
        void turnAroundRight();
        /* Align robot in middle of cell using front wall) */
        void front_align();
        // stop robot
        void stop();
        void advance(int ticks);

    private:
        // sensor pointer local variables
        VL6180X* _frontIR;
        VL6180X* _leftIR;
        VL6180X* _rightIR;
        VL6180X* _leftDiagIR;
        VL6180X* _rightDiagIR;
        // time
        unsigned long time;
        // pid variables for general purpose use
        double SetpointL, InputL, OutputL;
        double SetpointR, InputR, OutputR;
        // subroutine functions
        void followTicksRight(int ticks);
        void followTicksLeft(int ticks);

        void moveTicks(int Lticks, int Rticks);
        // basic functions
        void commandMotors(double left, double right);
        void wait(int millis);
};

#endif
