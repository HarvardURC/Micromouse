#include "Arduino.h"
#include "Motors.h"

volatile int _counterL = 0;
volatile int _counterR = 0;

void onTickL();
void onTickR();

Motors::Motors(int drivepinL, int drivepinR, int tickpinL,
               int tickpinR, int phasepinL, int phasepinR)
{
  pinMode(tickpinL, INPUT);
  pinMode(tickpinR, INPUT);
  attachInterrupt(digitalPinToInterrupt(tickpinL), onTickL, RISING);
  attachInterrupt(digitalPinToInterrupt(tickpinR), onTickR, RISING);
  pinMode(drivepinL, OUTPUT);
  pinMode(drivepinR, OUTPUT);
  pinMode(phasepinL, OUTPUT);
  pinMode(phasepinR, OUTPUT);

  _drivepinL = drivepinL;
  _drivepinR = drivepinR;
  _phasepinL = phasepinL;
  _phasepinR = phasepinR;

}


void Motors::oneMotor(int pin, int* counter, int pwm, int tickDelta)
{
  *counter = 0;
  while (*counter < tickDelta) {
    analogWrite(pin, pwm);
  }
  analogWrite(pin, 0);
}

void Motors::forward(int pwm, int tickDelta)
{
  _counterL = 0;
  _counterR = 0;

  while (_counterR < tickDelta || _counterL < tickDelta) {

    analogWrite(_drivepinL, pwm);
    analogWrite(_drivepinR, pwm);

    if (_counterL > _counterR)
    {
      analogWrite(_drivepinL, 0);
    }
    else if (_counterL < _counterR)
    {
      analogWrite(_drivepinR, 0);
    }


  }
  analogWrite(_drivepinL, 0);
  analogWrite(_drivepinR, 0);

}

void Motors::turnLeft()
{

}

void Motors::turnRight()
{

}

void Motors::turnAround()
{

}

void onTickL()
{
  _counterL++;

}

void onTickR()
{
  _counterR++;
}
