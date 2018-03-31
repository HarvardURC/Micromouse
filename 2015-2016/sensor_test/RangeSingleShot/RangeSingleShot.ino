/* This minimal example shows how to get single-shot range
measurements from the VL6180X.

The range readings are in units of mm. */

#include <Wire.h>
#include <VL6180X.h>

VL6180X sensor;

void setup() 
{
  Serial.begin(9600);
  Wire.begin();
  delay(2000);
  Serial.print("blah");
  digitalWrite(22, HIGH);
  digitalWrite(23, LOW);
  sensor.init();
  Serial.print("blah");
  sensor.configureDefault();
  sensor.setTimeout(500);
  sensor.setAddress(1);
  
}

void loop() 
{ 
  Serial.print(sensor.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  
  Serial.println();
}
