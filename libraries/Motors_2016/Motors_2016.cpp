#include <Arduino.h>
#include "Motors_2016.h"
#include <Encoder.h>
#include <PID_v1.h>
#include <VL6180X.h>
#define WALL_DISTANCE 215
#define MOTOR_SPEED 70

// helpful library objects
Encoder *encoderLeft;
Encoder *encoderRight;
PID *PIDLeft;
PID *PIDRight;

// now takes pointers to sensor objects instead of int pins
Motors_2016::Motors_2016(int powerPinL, int directionPinL, int powerPinR,
                         int directionPinR, int encoderPinL1, int encoderPinL2,
                         int encoderPinR1, int encoderPinR2, VL6180X* frontIR, 
                         VL6180X* leftIR, VL6180X* rightIR, VL6180X* leftDiagIR,
                         VL6180X* rightDiagIR) 
{
  // all them pins
  _powerPinL = powerPinL;
  _directionPinL = directionPinL;
  _powerPinR = powerPinR;
  _directionPinR = directionPinR;
  _encoderPinL1 = encoderPinL1;
  _encoderPinL2 = encoderPinL2;
  _encoderPinR1 = encoderPinR1;
  _encoderPinR2 = encoderPinR2;
  _frontIR = frontIR;
  _leftIR = leftIR;
  _rightIR = rightIR;
  _leftDiagIR = leftDiagIR;
  _rightDiagIR = rightDiagIR;
  // helpful library objects
  encoderLeft = new Encoder (_encoderPinL1, _encoderPinL2);
  encoderRight = new Encoder (_encoderPinR1, _encoderPinR2);
  PIDLeft = new PID (&InputL, &OutputL, &SetpointL,1.5,0.0018,.01, DIRECT);
  PIDRight = new PID (&InputR, &OutputR, &SetpointR,1.5,0.0018,.01, DIRECT);
  // set up motors
  pinMode (_powerPinL, OUTPUT);
  pinMode (_directionPinL, OUTPUT);
  pinMode (_powerPinR, OUTPUT);
  pinMode (_directionPinR, OUTPUT);
  // set up PID controllers
  PIDLeft->SetOutputLimits(-1*MOTOR_SPEED, MOTOR_SPEED);
  PIDRight->SetOutputLimits(-1*MOTOR_SPEED, MOTOR_SPEED);
  PIDLeft->SetMode(AUTOMATIC);
  PIDRight->SetMode(AUTOMATIC);
}

// function to advance 1 cell, using wall follow if possible
void Motors_2016::forward()
{   
  // use whatever wall is available to follow forward
  int right_peek = rightDiagIR->readRangeSingleMillimeters();
  if (right_peek < WALL_THRESHOLD_DIAG && 
      rightIR->readRangeSingleMillimeters() < WALL_THRESHOLD) 
  {
    followTicksRight(5000);
  }
  // if not able to wall follow, use odometry
  tmoveTicks(1000,1000);
}

// 90 degree left turn
void Motors_2016::turnLeft()
{
  moveTicks(-400,400);
}

// 90 degree right turn
void Motors_2016::turnRight()
{
  moveTicks(400,-400);
}

// aligns robot to the wall in front, straightening position
// and leaving room so a 90 degree turn will result in center
void Motors_2016::front_align()
{
  int d = _frontIR->readRangeSingleMillimeters();
  double SetpointF, InputF, OutputF;
  SetpointF = 200;
  PID PIDFront (&InputF, &OutputF, &SetpointF,1.0,0.0018,.01, REVERSE);
  PIDFront.SetOutputLimits(-50, 50);
  PIDFront.SetMode(AUTOMATIC);
  while (abs(d - SetpointF) > 5)
  {
    PIDFront.Compute();
    int rd = _rightDiagIR->readRangeSingleMillimeters();
    int ld = _leftDiagIR->readRangeSingleMillimeters();
    if ((OutputF > 0 && rd > ld) || (OutputF < 0 && ld > rd)){
      commandMotors(0, OutputF);
    }
    else if ((OutputF > 0 && rd < ld) || (OutputF < 0 && ld > rd)){
      Serial.print('asdfaa;kdf;oasdjf;lksadj;aflasdjklfj');
      commandMotors(OutputF, 0);
    }
    else {
      commandMotors(OutputF, OutputF);
    }
    d = _frontIR->readRangeSingleMillimeters();
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
    InputR = _rightIR->readRangeSingleMillimeters(); //read right IR sensor 
    SetpointR = WALL_DISTANCE;
    PIDRight->SetTunings(.5,0.01,0.01); // determine tunings
    PIDRight->Compute();
    //commandMotors(100,100);
    Serial.println(MOTOR_SPEED + OutputR);
    commandMotors((MOTOR_SPEED - OutputR)/2, (MOTOR_SPEED + OutputR)/2);
  }
  stop();
}

// moves left and right motors given # of ticks, maxes at 10 seconds
void Motors_2016::moveTicks(int Lticks, int Rticks)
{
  encoderLeft->write(0);
  encoderRight->write(0);
  SetpointL = Lticks;
  SetpointR = Rticks;
  PIDLeft->SetTunings(1.5,0.01,0.01);
  PIDRight->SetTunings(1.5,0.01,0.01);
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
    //commandMotors(70, 70);
    Serial.print(InputL);
    Serial.println();
  }
  stop();
  wait(100);
  time = millis();
  PIDRight->SetTunings(2,0.01,0.01);
  PIDLeft->SetTunings(2,0.01,0.01);
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

// send power commands to left and right motors
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

// stop both motors
void Motors_2016::stop()
{
  analogWrite(_powerPinR, 0);
  analogWrite(_powerPinL, 0);
}

// wait for milliseconds
void Motors_2016::wait(int ms)
{
  time = millis();
  while (millis() - time < ms){}
}


