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
    if not then stop motor with higher count */

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

/* 0.786 degrees per tick
 * 115 ticks to go 90 degrees
 * 229 ticks to go 180 degrees */

void Motors::turnLeft()
{
  /* motors initially set HIGH forward.
     to turn left set L to low and then reset to high when complete */

  digitalWrite(_phasepinL, HIGH);
  forward(60, 115);
  digitalWrite(_phasepinL, LOW);

}

void Motors::turnRight()
{
  /* same as above but change phasepinR */

  digitalWrite(_phasepinR, HIGH);
  forward(60, 115);
  digitalWrite(_phasepinR, LOW);

}

void Motors::turnAround()
{

  digitalWrite(_phasepinR, HIGH);
  forward(60, 229);
  digitalWrite(_phasepinR, LOW);

}

void Motors::wallOrientate()
{

  analogWrite(_drivepinL, 255);
  analogWrite(_drivepinR, 255);
  delay(500);
  analogWrite(_drivepinL, 0);
  analogWrite(_drivepinR, 0);

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
