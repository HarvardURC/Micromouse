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
  void turnAroundLeft();
  void turnAroundRight();
  void forward(int pwm, int tickDelta);
  void accForward(int start_pwm, int max_pwm, int tickDelta);
  void wallOrientateFwd();
  void wallOrientateBkwd();
private:
  int _drivepinL;
  int _drivepinR;
  int _phasepinL;
  int _phasepinR;
  void oneMotor(int pin, int* counter, int pwm, int tickDelta);
  void bump();
};

#endif
