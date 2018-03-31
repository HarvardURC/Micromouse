/*
 * This test prints encoder values to the Serial Monitor for both wheels
 * To check, manually spin wheels and see if values change properly
 */

#include <VL6180X.h>
#include <i2c_t3.h>
#include <config.h>
#include <Encoder.h>

Encoder encoderLeft(pins::encoderL1, pins::encoderL2);
Encoder encoderRight(pins::encoderR1, pins::encoderR2);

void setup() {
  // put your setup code here, to run once:
  // quiet the buzzer
  pinMode(pins::buzzer, OUTPUT);
  digitalWrite(pins::buzzer, LOW);
  
  Serial.begin(9600);
  Serial.print("Right + Left Encoder Test:");
}

long positionLeft  = -999;
long positionRight = -999;

void loop() {
  long newLeft, newRight;
  newLeft = encoderLeft.read();
  newRight = encoderRight.read();
  if (newLeft != positionLeft || newRight != positionRight) {
    Serial.print("Left = ");
    Serial.print(newLeft);
    Serial.print(", Right = ");
    Serial.print(newRight);
    Serial.println();
    positionLeft = newLeft;
    positionRight = newRight;
  }
  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    encoderLeft.write(0);
    encoderRight.write(0);
  }
}
