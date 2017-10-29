#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <Util.h>
#include "VL6180X.h"
#include <i2c_t3.h>

// define distance sensor pins
VL6180X *leftIR;
VL6180X *leftDiagIR;
VL6180X *frontIR;
VL6180X *rightDiagIR;
VL6180X *rightIR;

void setup() {
  // put your setup code here, to run once:
  // Initialize sensors
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, 100000);
  //Wire.begin();
  frontIR = new VL6180X;
  leftIR = new VL6180X;
  rightDiagIR = new VL6180X;
  leftDiagIR = new VL6180X;
  rightIR = new VL6180X;
  pinMode(23, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(21, OUTPUT);
  pinMode(20, OUTPUT);
  pinMode(19, OUTPUT);
  digitalWrite(23, HIGH);
  digitalWrite(22, LOW);
  digitalWrite(21, LOW);
  digitalWrite(20, LOW);
  digitalWrite(19, LOW);
  Serial.print("trying IR..."); 
  // left IR
  leftIR->init();
  leftIR->configureDefault();
  leftIR->setScaling(2);
  leftIR->setAddress(1);
  Serial.print("left IR Connected!");
  // leftDiag IR
  digitalWrite(22, HIGH); 
  leftDiagIR->init();
  leftDiagIR->configureDefault();
  leftDiagIR->setScaling(2);
  leftDiagIR->setAddress(2);
  Serial.print("leftDIag IR Connected!");
  // front IR
  digitalWrite(21, HIGH); 
  frontIR->init();
  frontIR->configureDefault();
  frontIR->setScaling(2);
  frontIR->setAddress(3);
  Serial.print("front IR Connected!");
  // right Diag IR
  digitalWrite(20, HIGH); 
  rightDiagIR->init();
  rightDiagIR->configureDefault();
  rightDiagIR->setScaling(2);
  rightDiagIR->setAddress(4);
  Serial.print("rightDIag IR Connected!");
  // right IR
  digitalWrite(19, HIGH); 
  rightIR->init();
  rightIR->configureDefault();
  rightIR->setScaling(2);
  rightIR->setAddress(5);
  Serial.print("All IR Connected!");
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(" left: ");
  Serial.print(leftIR->readRangeSingleMillimeters());
  Serial.println();
  wait(10);
  Serial.print(" right: ");
  Serial.print(rightIR->readRangeSingleMillimeters());
  Serial.println();
  wait(10);
  Serial.print(" leftDiag: ");
  Serial.print(leftDiagIR->readRangeSingleMillimeters());
  Serial.println();
  wait(10);
  Serial.print(" rightDiag: ");
  Serial.print(rightDiagIR->readRangeSingleMillimeters());
  Serial.println();
  wait(10);
  Serial.print(" front: ");
  Serial.print(frontIR->readRangeSingleMillimeters());
  Serial.println();
  wait(1000);
}
