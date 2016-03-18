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
  void wallOrientate();
private:
  int _drivepinL;
  int _drivepinR;
  int _phasepinL;
  int _phasepinR;
  void oneMotor(int pin, int* counter, int pwm, int tickDelta);

};

#endif
