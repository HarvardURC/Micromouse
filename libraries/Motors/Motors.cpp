#include "Arduino.h"
#include "Util.h"
#include "Motors.h"

//initialize counters at 0
volatile int _counterL = 0;
volatile int _counterR = 0;

void onTickL();
void onTickR();
void wait();

Motors::Motors(int drivePinL, int drivePinR, int tickPinL,
               int tickPinR, int phasePinL, int phasePinR,
               int forwardIRPin, int leftIRPin, int rightIRPin)
{
  pinMode(tickPinL, INPUT);
  pinMode(tickPinR, INPUT);
  attachInterrupt(digitalPinToInterrupt(tickPinL), onTickL, RISING);
  attachInterrupt(digitalPinToInterrupt(tickPinR), onTickR, RISING);
  pinMode(drivePinL, OUTPUT);
  pinMode(drivePinR, OUTPUT);
  pinMode(phasePinL, OUTPUT);
  pinMode(phasePinR, OUTPUT);

  _drivePinL = drivePinL;
  _drivePinR = drivePinR;
  _phasePinL = phasePinL;
  _phasePinR = phasePinR;

  _forwardIRPin = forwardIRPin;
  _leftIRPin = leftIRPin;
  _rightIRPin = rightIRPin;

  digitalWrite(phasePinL, LOW);
  digitalWrite(phasePinR, LOW);

  releaseFlag = 0;
  pwmRecord = 60;
}

void Motors::stop()
{
  analogWrite(_drivePinL, 0);
  analogWrite(_drivePinR, 0);
}

void Motors::oneMotor(int pin, int* counter, int pwm, int tickDelta)
{
  *counter = 0;

  /* if too many turns then stop */

  while (*counter < tickDelta) {
    analogWrite(pin, pwm);
  }
  analogWrite(pin, 0);
}

void Motors::accForward(int start_pwm, int max_pwm, int tickDelta)
{
  accForward(start_pwm, max_pwm, tickDelta, 0, 0);
}


int Motors::accForward(int start_pwm, int max_pwm, int tickDelta,
                       int useSensors, int forceDecelerate)
{
  if (releaseFlag)
  {
    return 0;
  }

  int rv = -1;

  _counterL = 0;
  _counterR = 0;

  float acc_pwm = start_pwm;

  float deltaTime = 0;
  int prevTime = millis();
  unsigned long startTime = millis();

  /*check motors are in sync
    if not then stop motor with higher count */

  while ((_counterR < tickDelta || _counterL < tickDelta) &&
         (!useSensors || irReading(_forwardIRPin) <= 300))
  {
    if (millis() - startTime > 5000)
    {
      stop();
      while (!releaseFlag)
      {
        wait(100);
      }
      return 0;
    }

    int pwm = acc_pwm;
    if (pwm > max_pwm)
    {
      pwm = max_pwm;
    }
    /*
    int leftReading = irReading(_leftIRPin);
    int rightReading = irReading(_rightIRPin);

    int maxReading, minReading;
    if (leftReading > rightReading)
    {
      maxReading = leftReading;
      minReading = rightReading;
    }
    else
    {
      maxReading = rightReading;
      minReading = leftReading;
    }

    if (leftReading > 140 && rightReading > 140 &&
        !(minReading > 200 && maxReading > 270) &&
        _counterR + _counterL < 100 && 0)
    {
      int avgCounter = (_counterR + _counterL) / 2;
      _counterR = avgCounter;
      _counterL = avgCounter;

      int adjustment = (leftReading - rightReading) / 2;
      if (pwm + adjustment > 0)
      {
        analogWrite(_drivePinL, pwm + adjustment);
      }
      else
      {
        analogWrite(_drivePinL, 0);
      }
      if (pwm - adjustment > 0)
      {
        analogWrite(_drivePinR, pwm - adjustment);
      }
      else
      {
        analogWrite(_drivePinR, 0);
      }
    }
    else
    {*/
    if (_counterL > _counterR)
    {
      analogWrite(_drivePinL, 0);
      analogWrite(_drivePinR, pwm);
    }
    else if (_counterL < _counterR)
    {
      analogWrite(_drivePinL, pwm);
      analogWrite(_drivePinR, 0);
    }
    else
    {
      analogWrite(_drivePinL, pwm);
      analogWrite(_drivePinR, pwm);
    }
    //}

    int curTime = millis();
    deltaTime = curTime - prevTime;
    prevTime = curTime;
    if ((_counterL + _counterR) < tickDelta ||
        (rv == 0 && !forceDecelerate))
    {
      acc_pwm += deltaTime / 2;
    }
    else if (rv == -1)
    {
      int wallThreshold = 140;
      int lowThreshold = 150;
      int highThreshold = 320;

      int rightReading = irReading(_rightIRPin);
      int leftReading = irReading(_leftIRPin);

      // if too close to or too far from a side wall, bump it
      if (rightReading > highThreshold)
      {
        rv = 1;
      }
      else if (leftReading > highThreshold)
      {
        rv = 2;
      }
      else if (rightReading > wallThreshold && rightReading < lowThreshold)
      {
        rv = 3;
      }
      else if(leftReading > wallThreshold && leftReading < lowThreshold)
      {
        rv = 4;
      }
      else
      {
        rv = 0;
      }
    }
    else
    {
      acc_pwm = start_pwm;
    }
  }
  stop();
  pwmRecord = acc_pwm;
  return rv;
}

void Motors::forward(int pwm, int tickDelta)
{
  accForward(pwm, pwm, tickDelta);
}

int Motors::forward(int pwm, int tickDelta, int useSensors)
{
  return accForward(pwm, pwm, tickDelta, useSensors, 0);
}

/* 110 ticks to go 90 degrees
 * 220 ticks to go 180 degrees */

void Motors::turnLeft()
{
  /* motors initially set HIGH forward.
     to turn left set L to low and then reset to high when complete */

  digitalWrite(_phasePinL, HIGH);
  forward(60, 105);
  digitalWrite(_phasePinL, LOW);

}

void Motors::turnRight()
{
  /* same as above but change phasePinR */

  digitalWrite(_phasePinR, HIGH);
  forward(60, 105);
  digitalWrite(_phasePinR, LOW);

}

void Motors::turnAroundLeft()
{

  digitalWrite(_phasePinL, HIGH);
  forward(60, 215);
  digitalWrite(_phasePinL, LOW);

}

void Motors::turnAroundRight()
{

  digitalWrite(_phasePinR, HIGH);
  forward(60, 215);
  digitalWrite(_phasePinR, LOW);

}

void Motors::bump(int pwm)
{
  if(releaseFlag){
    return;
  }
  analogWrite(_drivePinL, pwm);
  analogWrite(_drivePinR, pwm);
  int leftRunning = 1;
  int rightRunning = 1;
  int prevCounterL;
  int prevCounterR;
  do
  {
    prevCounterL = _counterL;
    prevCounterR = _counterR;

    wait(100);

    if (_counterL - prevCounterL < 3)
    {
      analogWrite(_drivePinL, 0);
      leftRunning = 0;
    }
    if (_counterR - prevCounterR < 3)
    {
      analogWrite(_drivePinR, 0);
      rightRunning = 0;
    }
  }
  while (leftRunning || rightRunning);
}

void Motors::wiggle()
{
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(_phasePinL, HIGH);
    analogWrite(_drivePinL, 20);
    analogWrite(_drivePinR, 40);
    wait(180 - i * 60);
    digitalWrite(_phasePinL, LOW);

    digitalWrite(_phasePinR, HIGH);
    analogWrite(_drivePinL, 40);
    analogWrite(_drivePinR, 20);
    wait(180 - i * 60);
    digitalWrite(_phasePinR, LOW);
  }
  stop();
}

void Motors::wallOrientateFwd()
{
  bump(60);

  wiggle();

  digitalWrite(_phasePinL, HIGH);
  digitalWrite(_phasePinR, HIGH);

  forward(60, 16);

  digitalWrite(_phasePinL, LOW);
  digitalWrite(_phasePinR, LOW);
}

void Motors::wallOrientateBkwd()
{
  digitalWrite(_phasePinL, HIGH);
  digitalWrite(_phasePinR, HIGH);

  bump(40);

  digitalWrite(_phasePinL, LOW);
  digitalWrite(_phasePinR, LOW);

  forward(60, 86);
}

//increment count with each turn

void onTickL()
{
  _counterL++;

}

void onTickR()
{
  _counterR++;
}
