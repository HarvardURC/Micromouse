#ifndef Motors_h
#define Motors_h

#include "Arduino.h"

class Motors
{
public:
  Motors(int drivePinL, int drivePinR, int tickPinL,
         int tickPinR, int phasePinL, int phasePinR,
         int forwardIRPin, int leftIRPin, int rightIRPin);
  void turnLeft();
  void turnRight();
  void turnAroundLeft();
  void turnAroundRight();
  void forward(int pwm, int tickDelta);
  int forward(int pwm, int tickDelta, int useSensors);
  void accForward(int start_pwm, int max_pwm, int tickDelta);
  int accForward(int start_pwm, int max_pwm, int tickDelta,
                 int useSensors, int forceDecelerate);
  void wallOrientateFwd();
  void wallOrientateBkwd();
  int releaseFlag;
  int pwmRecord;
private:
  int _drivePinL;
  int _drivePinR;
  int _phasePinL;
  int _phasePinR;
  int _forwardIRPin;
  int _leftIRPin;
  int _rightIRPin;
  void stop();
  void oneMotor(int pin, int* counter, int pwm, int tickDelta);
  void bump(int pwm);
  void wiggle();
};

#endif
