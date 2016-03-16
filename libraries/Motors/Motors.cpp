#include "Arduino.h"
#include "Motors.h"

//initialise counters at 0
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

  digitalWrite(phasepinL, LOW);
  digitalWrite(phasepinR, LOW);
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

void Motors::forward(int pwm, int tickDelta)
{
  _counterL = 0;
  _counterR = 0;

  /*check motors are in sync
    if not then stop
    motor with higher count */

  while (_counterR < tickDelta || _counterL < tickDelta) {

    if (_counterL > _counterR)
    {
      analogWrite(_drivepinL, 0);
      analogWrite(_drivepinR, pwm);
    }
    else if (_counterL < _counterR)
    {
      analogWrite(_drivepinL, pwm);
      analogWrite(_drivepinR, 0);
    }
    else
    {
      analogWrite(_drivepinL, pwm);
      analogWrite(_drivepinR, pwm);
    }

  }
  analogWrite(_drivepinL, 0);
  analogWrite(_drivepinR, 0);

}

void Motors::turnLeft()
{
  /*
  digitalWrite(phasepinL, LOW);
  forward(/something, /somthing else)
    digitalWrite(phasepinL, HIGH);
  */

}

void Motors::turnRight()
{
  /*
    digitalWrite(phasepinR, LOW);
    forward(/something, the other thing)
    digitalWrite(phasepinR, HIGH) */

}

void Motors::turnAround()
{

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
