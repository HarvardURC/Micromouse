/**
 * Motors.h
 * Contains various methods for using the DC motors.
 */

#ifndef Motors_h
#define Motors_h

#include "Arduino.h"

class Motors
{
public:
  // Constructor. "Drive pins" refer to enable pins.
  Motors(int drivePinL, int drivePinR, int tickPinL,
         int tickPinR, int phasePinL, int phasePinR,
         int forwardIRPin, int leftIRPin, int rightIRPin);
  
  // 90 deg turns
  void turnLeft();
  void turnRight();
  
  // 180 deg turns
  void turnAroundLeft();
  void turnAroundRight();

  /* Move forward using specified pwm value (0-255) for specified number of
   * ticks. 3rd argument useSensors is optional bool; it activates automatic
   * stop when obstacle is detected in front of front sensor. If sensors are
   * used, int return value is the flag 0-4 for whether side sensors determine
   * that a side bump should take place.
   * 0: no bump; 1: right too close; 2: left too close; 3: right too far;
   *   4: left too far */
  void forward(int pwm, int tickDelta);
  int forward(int pwm, int tickDelta, int useSensors);

  /* Moves forward, accelerating at the beginning and possibly decelerating at
   * the end. PWM value starts at start_pwm and increases linearly at a fixed
   * rate, up to a maximum possible of max_pwm. TickDelta and useSensors work as
   * with the forward function. ForceDecelerate is a bool which should be true
   * by default, enabling deceleration at the end of the motion to avoid
   * coasting. ForceDecelerate should be switched off if preloading indicates
   * that the next action will be forward again with no turn. In that case, the
   * robot will not decelerate unless it deems a side bump necessary. */
  void accForward(int start_pwm, int max_pwm, int tickDelta);
  int accForward(int start_pwm, int max_pwm, int tickDelta,
                 int useSensors, int forceDecelerate);

  /* Performs a forward or backward bump. Motors are switched off when they
   * start to stall. */
  void wallOrientateFwd();
  void wallOrientateBkwd();

  /* Upon a long stall, the Motors library will capture control, pausing the
   * main program. Once an interrupt sets releaseFlag to true, the library will
   * relinquish control. It will also refuse to move the motors in response to
   * any method call until releaseFlag is set back to false. */
  int releaseFlag;

  // Stores the last pwm value fed to the motors.
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
