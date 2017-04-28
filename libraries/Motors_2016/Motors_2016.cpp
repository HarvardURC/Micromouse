#include <Arduino.h>
#include "Motors_2016.h"
#include <Encoder.h>
#include <PID_v1.h>
#include <VL6180X.h>
#define WALL_DISTANCE 225
#define WALL_THRESHOLD 270
#define WALL_THRESHOLD_DIAG 288
#define MOTOR_SPEED 70
#define TICKS_CELL 1610
#define TICKS_TURN 525

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
  // stage 1: move ~3 cm to stick head out of cell   
  // use whatever wall is available to follow forward
  advance(TICKS_CELL * .2);
  moveTicks(TICKS_CELL * .25, TICKS_CELL * .25);
  // stage 2: move to have wheels centered in next cell
  int right_peek = _rightDiagIR->readRangeSingleMillimeters();
  int left_peek = _leftDiagIR->readRangeSingleMillimeters();
  if (right_peek < WALL_THRESHOLD_DIAG){
    followTicksRight(TICKS_CELL * .55);
  }
  else if (left_peek < WALL_THRESHOLD_DIAG){
    followTicksLeft(TICKS_CELL * .55);
  }
  // if not able to wall follow, use odometry
  else{
    moveTicks(TICKS_CELL * .65, TICKS_CELL * .65);
  }
}

// 180 degree left turn
void Motors_2016::turnAroundLeft()
{
  turnLeft();
  turnLeft();
}

// 180 degree right turn
void Motors_2016::turnAroundRight()
{
  turnRight();
  turnRight();
}

// 90 degree left turn
void Motors_2016::turnLeft()
{
  if (_frontIR -> readRangeSingleMillimeters() < WALL_THRESHOLD){
    front_align();
  }
  moveTicks(-1 * TICKS_TURN, TICKS_TURN);
}

// 90 degree right turn
void Motors_2016::turnRight()
{
  front_align();
  moveTicks(TICKS_TURN,-1 * TICKS_TURN);
}

// aligns robot to the wall in front, straightening position
// and leaving room so a 90 degree turn will result in center
void Motors_2016::front_align()
{
  int d = _frontIR->readRangeSingleMillimeters();
  int rd = _rightDiagIR->readRangeSingleMillimeters();
  int ld = _leftDiagIR->readRangeSingleMillimeters();
  // stage 1: move close to desired range (unconfirmed)
  int ticks_offset = (d - WALL_THRESHOLD) / .5;
  // use whatever wall is available to follow forward
  moveTicks(ticks_offset, ticks_offset);
  // stage 2: finish moving to desired distance while straightening
  double SetpointF, InputF, OutputF;
  SetpointF = WALL_DISTANCE - 35;
  PID PIDFront (&InputF, &OutputF, &SetpointF,1.7,0.0018,.01, REVERSE);
  PIDFront.SetOutputLimits(-50, 50);
  PIDFront.SetMode(AUTOMATIC);
  time = millis();
  // loop until desired eistance achieved or 5 seconds pass
  while ((abs(d - SetpointF) > 10 || abs(rd - ld) > 25)
          && (millis() - time < 4000))
  {
    PIDFront.Compute();
    int rd = _rightDiagIR->readRangeSingleMillimeters();
    int ld = _leftDiagIR->readRangeSingleMillimeters();
    if ((OutputF > 0 && rd > ld) || (OutputF < 0 && ld > rd)){
      commandMotors(-20, OutputF + 20);
    }
    else if ((OutputF > 0 && rd < ld) || (OutputF < 0 && ld > rd)){
      commandMotors(OutputF + 20, -20);
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
  PIDRight->SetTunings(1.2,0.01,0.01); // determine tunings
  // follow until combined motor ticks double the ticks parameter
  while (distanceL + distanceR < ticks * 2)
  {
    distanceL = encoderLeft->read();
    distanceR = encoderRight->read();
    InputR = _rightIR->readRangeSingleMillimeters(); //read right IR sensor 
    SetpointR = WALL_DISTANCE;
    PIDRight->Compute();
    commandMotors((MOTOR_SPEED - OutputR)/2, (MOTOR_SPEED + OutputR)/2);
  }
  stop();
}


// follows the right wall for a certain number of ticks
void Motors_2016::followTicksLeft(int ticks)
{
  encoderLeft->write(0);
  encoderRight->write(0);
  int distanceL = 0;
  int distanceR = 0;
  PIDLeft->SetTunings(1.2,.01,0.01); // determine tunings
  // follow until combined motor ticks double the ticks parameter
  while (distanceL + distanceR < ticks * 2)
  {
    distanceL = encoderLeft->read();
    distanceR = encoderRight->read();
    InputL = _leftIR->readRangeSingleMillimeters(); //read left IR sensor 
    SetpointL = WALL_DISTANCE;
    PIDLeft->Compute();
    commandMotors((MOTOR_SPEED + OutputL)/2, (MOTOR_SPEED - OutputL)/2);
  }
  stop();
}

void Motors_2016::advance(int ticks)
{
  // use right wall if available
  if (_rightIR->readRangeSingleMillimeters() < WALL_THRESHOLD){
    Serial.print("ASADASDASDA");
    followTicksRight(ticks);
  }
  // otherwise, use left wall if available
  else if (_leftIR->readRangeSingleMillimeters() < WALL_THRESHOLD){
    followTicksLeft(ticks);
  }
  // if not able to wall follow, use odometry
  else{
    moveTicks(ticks,ticks);
  }
}

// moves left and right motors given # of ticks, maxes at 10 seconds
void Motors_2016::moveTicks(int Lticks, int Rticks)
{
  encoderLeft->write(0);
  encoderRight->write(0);
  SetpointL = Lticks;
  SetpointR = Rticks;
  PIDLeft->SetTunings(1.4,0.01,0.01);
  PIDRight->SetTunings(1.4,0.01,0.01);
  InputL = 0;
  InputR = 0;
  time = millis();
  while ((abs(InputL-SetpointL) > 10 || abs(InputR-SetpointR) > 10)
         && (millis() - time < 4000)){
    InputL = encoderLeft->read();
    InputR = encoderRight->read();
    PIDLeft->Compute();
    PIDRight->Compute();
    int lrem = abs(InputL-SetpointL);
    int rrem = abs(InputR-SetpointR);
    if (lrem - rrem > 0){
      commandMotors(OutputL, 0);
    }
    // advance the desired ticks for each wheel, moving only
    // the wheel furthest away from its destination
    else if(rrem - lrem > 0){
      commandMotors(0, OutputR);
    }
    else {
      commandMotors(OutputL, OutputR);
    }
    Serial.print(InputR);
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


