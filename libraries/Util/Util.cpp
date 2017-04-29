#include "Arduino.h"
#include "Util.h"

void wait(int ms)
{
  unsigned long previousMillis = millis();
  unsigned long currentMillis = millis();
  while ((unsigned long)(currentMillis - previousMillis) < ms)
  {
    currentMillis = millis();
  }
}

void microWait(int us)
{
  unsigned long previousMicros = micros();
  unsigned long currentMicros = micros();
  while ((unsigned long)(currentMicros - previousMicros) < us)
  {
    currentMicros = micros();
  }
}
