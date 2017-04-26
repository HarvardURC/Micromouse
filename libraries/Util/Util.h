#ifndef Util_h
#define Util_h

#include "Arduino.h"
#include <VL6180X.h>

void wait(int ms);
void microWait(int us);
int irReading(VL6180X *sensor);

#endif
