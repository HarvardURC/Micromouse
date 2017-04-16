#include <Arduino.h>
#include "Motors_2016.h"
#include <Encoder.h>
#include <PID_v1.h>
#include <VL6180X.h>
#include <Wire.h>
#define WALL_DISTANCE 0

// helpful library objects
Encoder *encoderLeft;
Encoder *encoderRight;
PID *PIDLeft;
PID *PIDRight;

VL6180X *frontIR;

// change to take pointers to sensor objects instead of int pins
Motors_2016::Motors_2016(int powerPinL, int directionPinL, int powerPinR,
                         int directionPinR, int encoderPinL1, int encoderPinL2,
                         int encoderPinR1, int encoderPinR2, int forwardIRPin, 
                         int leftIRPin, int rightIRPin, int leftDiagIRPin,
                         int rightDiagIRPin) 
{
    // all them pins (check if should use smaller types)
    _powerPinL = powerPinL;
    _directionPinL = directionPinL;
    _powerPinR = powerPinR;
    _directionPinR = directionPinR;
    _encoderPinL1 = encoderPinL1;
    _encoderPinL2 = encoderPinL2;
    _encoderPinR1 = encoderPinR1;
    _encoderPinR2 = encoderPinR2;
    _forwardIRPin = forwardIRPin;
    _leftIRPin = leftIRPin;
    _rightIRPin = rightIRPin;
    _leftDiagIRPin = leftDiagIRPin;
    _rightDiagIRPin = rightDiagIRPin;
    // helpful library objects
    encoderLeft = new Encoder (_encoderPinL1, _encoderPinL2);
    encoderRight = new Encoder (_encoderPinR1, _encoderPinR2);
    PIDLeft = new PID (&InputL, &OutputL, &SetpointL,1,0.0018,.01, DIRECT);
    PIDRight = new PID (&InputR, &OutputR, &SetpointR,1,0.0018,.01, DIRECT);
    // set up motors
    pinMode (_powerPinL, OUTPUT);
    pinMode (_directionPinL, OUTPUT);
    pinMode (_powerPinR, OUTPUT);
    pinMode (_directionPinR, OUTPUT);
    // set up PID controllers
    PIDLeft->SetOutputLimits(-125, 125);
    PIDRight->SetOutputLimits(-150, 150);
    PIDLeft->SetMode(AUTOMATIC);
    PIDRight->SetMode(AUTOMATIC);
    // set up IR sensor (TEMPORARY - PASS OBJECT POINTER LATER)
    frontIR = new VL6180X;
    Wire.begin();
    pinMode(23, OUTPUT);
    pinMode(22, OUTPUT);
    digitalWrite(23, HIGH);
    digitalWrite(22, LOW);
    frontIR->init();
    Serial.print("IR Connected!");
    frontIR->configureDefault();
    frontIR->setScaling(2);
    frontIR->setAddress(1);
}

void Motors_2016::forward()
{   
    // use whatever wall is available to follow forward

    // if not able to wall follow, use odometry
    moveTicks(500,500);
}

void Motors_2016::turnLeft()
{
    moveTicks(-250,250);
}

void Motors_2016::turnRight()
{
    moveTicks(250,-250);
}

void Motors_2016::front_align()
{
   int d = frontIR->readRangeSingleMillimeters();
   double SetpointF, InputF, OutputF;
   SetpointF = 230;
   PID PIDFront (&InputF, &OutputF, &SetpointF,2,0.0018,.01, REVERSE);
   PIDFront.SetOutputLimits(-90, 90);
   PIDFront.SetMode(AUTOMATIC);
   while (abs(d - SetpointF) > 5)
   {
     PIDFront.Compute();
     commandMotors(OutputF, OutputF);
     d = frontIR->readRangeSingleMillimeters();
     Serial.print(d);
     Serial.println();
     InputF = d;
   }
   stop();
}

// follows the right wall for a certain number of ticks
void Motors_2016::followTicksRight(int ticks)
{
  encoderLeft->write(0);
  encoderRight->write(0);
  int distanceL = 0;
  int distanceR = 0;
  while (distanceL + distanceR < ticks * 2)
  {
    distanceL = encoderLeft->read();
    distanceR = encoderRight->read();
    InputR = 0; //read right IR sensor 
    SetpointR = WALL_DISTANCE;
    PIDRight->SetTunings(1.5,0.01,0.01); // determine tunings
    PIDRight->Compute();
    commandMotors(255 - OutputR/2, 255 + OutputR/2);
  }
}

// moves left and right motors given # of ticks, maxes at 10 seconds
void Motors_2016::moveTicks(int Lticks, int Rticks)
{
    encoderLeft->write(0);
    encoderRight->write(0);
    SetpointL = Lticks;
    SetpointR = Rticks;
    //PIDLeft->SetTunings(1.5,0.01,0.01);
    //PIDRight->SetTunings(1.5,0.01,0.01);
    InputL = 0;
    InputR = 0;
    time = millis();
    while ((abs(InputL-SetpointL) > 30 || abs(InputR-SetpointR) > 30)
           && (millis() - time < 10000)){
      InputL = encoderLeft->read();
      InputR = encoderRight->read();
      PIDLeft->Compute();
      PIDRight->Compute();
      if (abs(InputL-SetpointL) > abs(InputR-SetpointR)){
        commandMotors(OutputL, 0);
      }
      else if(abs(InputL-SetpointL) < abs(InputR-SetpointR)){
        commandMotors(0, OutputR);
      }
      else {
        commandMotors(OutputL, OutputR);
      }
      Serial.print(InputL);
      Serial.println();
    }
    stop();
    wait(100);
    time = millis();
    PIDRight->SetTunings(3.5,0.01,0.01);
    PIDLeft->SetTunings(3.5,0.01,0.01);
    while (millis() - time < 1000){
      InputL = encoderLeft->read();
      InputR = encoderRight->read();
      PIDLeft->Compute();
      PIDRight->Compute();
      commandMotors(OutputL, OutputR);
      Serial.print(InputL);
      Serial.println();
    }
    
    stop();
}

void Motors_2016::commandMotors(double left, double right)
{
  if (left >= 0){
     digitalWrite(_directionPinL, LOW);
  }
  else {
    digitalWrite(_directionPinL, HIGH);
  }
  analogWrite(_powerPinL, abs(left)); 
  if (right >= 0){
     digitalWrite(_directionPinR, LOW);
  }
  else {
    digitalWrite(_directionPinR, HIGH);
  }
  analogWrite(_powerPinR, abs(right)); 
}

void Motors_2016::stop()
{
    analogWrite(_powerPinR, 0);
    analogWrite(_powerPinL, 0);
}

void Motors_2016::wait(int ms)
{
    time = millis();
    while (millis() - time < ms){}
}


