#ifndef Motors_h
#define Motors_h

#include "Arduino.h"

class Motors
{
  public:
    Motors(int drivepinL, int drivepinR, int tickpinL, 
           int tickpinR, int phasepinL, int phasepinR);
    void turnLeft();
    void turnRight();
    void turnAround();
    void forward(int pwm, int tickDelta);
   private: 
     int _drivepinL;
     int _drivepinR;
     int _phasepinL;
     int _phasepinR;
     volatile int _counterL;
     volatile int _counterR;
     void onTickL();
     void onTickR();
     void oneMotor(int pin, int pwm, int tickDelta);
   
    
    
}

#endif

