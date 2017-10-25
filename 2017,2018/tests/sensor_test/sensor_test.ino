/**
  A time of flight (ToF) sensor test.
  Outputs to Serial console the distance measured
  by the left and right sensors.
*/

#include <VL6180X.h>
#include <i2c_t3.h>
#include <config.h>

VL6180X rightIR;
VL6180X leftIR;

void wait(int ms)
{
  unsigned long previousMillis = millis();
  unsigned long currentMillis = millis();
  while ((currentMillis - previousMillis) < (unsigned long)ms)
  {
    currentMillis = millis();
  }
}

void setup() {
  Serial.begin(9600);
  delay(1000);
  Wire.begin(I2C_MASTER, 0, I2C_PINS_16_17, I2C_PULLUP_EXT, 50000);
  int reset_pin_left = pins::tofLeft;
  int reset_pin_right = pins::tofRight;
  pinMode(reset_pin_right, OUTPUT);
  pinMode(reset_pin_left, OUTPUT);
  
  digitalWrite(reset_pin_right, HIGH);
  digitalWrite(reset_pin_left, LOW);
  Serial.print("trying");
  rightIR.init();
  rightIR.configureDefault();
  rightIR.setScaling(2);
  Serial.print("right connected");
  // set first sensor to different address and set reset pin
  rightIR.setAddress(1);
  digitalWrite(reset_pin_left, HIGH); 
  leftIR.init();
  leftIR.configureDefault();
  leftIR.setScaling(2);
  leftIR.setAddress(2);
  Serial.print("left connected");
  rightIR.setTimeout(500);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("LOOPING");
  Serial.print("right: ");
  // Serial.print(rightIR.readRangeSingle());
  Serial.print(rightIR.readRangeSingleMillimeters());
  wait(10);
  Serial.print(" left: ");
  Serial.print(leftIR.readRangeSingleMillimeters());
  Serial.println();
  wait(10);
}
