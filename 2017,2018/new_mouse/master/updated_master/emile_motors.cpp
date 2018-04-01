#include <Arduino.h>
#include "emile_motors.h"
#include <Encoder.h>
#include <PID_v1.h>
#include <config.h>
#define WALL_DISTANCE_RIGHT 235
#define WALL_DISTANCE_LEFT 218
#define WALL_DISTANCE_FRONT 205
#define WALL_THRESHOLD 275
#define WALL_THRESHOLD_DIAG 288
#define MOTOR_SPEED 70
#define TICKS_CELL 1645
#define TICKS_TURN 552

// helpful library objects
Encoder *encoderLeft;
Encoder *encoderRight;
PID *PIDLeft;
PID *PIDRight;

// now takes pointers to sensor objects instead of int pins
emile_motors::emile_motors(VL6180X* leftIR, VL6180X* leftDiagIR, VL6180X* frontIR,
                         VL6180X* rightDiagIR, VL6180X* rightIR)
{
  // init reset flag
  releaseFlag = 0;
  _frontIR = frontIR;
  _leftIR = leftIR;
  _rightIR = rightIR;
  _leftDiagIR = leftDiagIR;
  _rightDiagIR = rightDiagIR;
  // helpful library objects
  encoderLeft = new Encoder (pins::encoderPinL1, pins::encoderPinL2);
  encoderRight = new Encoder (pins::encoderPinR1, pins::encoderPinR2);
  PIDLeft = new PID (&InputL, &OutputL, &SetpointL,1.5,0.0018,.01, DIRECT);
  PIDRight = new PID (&InputR, &OutputR, &SetpointR,1.5,0.0018,.01, DIRECT);
  // set up motors
  pinMode (pins::motorPowerL, OUTPUT);
  pinMode (pins::motorDirectionL, OUTPUT);
  pinMode (pins::motorPowerR, OUTPUT);
  pinMode (pins::motorDirectionR, OUTPUT);
  // set up PID controllers
  PIDLeft->SetOutputLimits(-1*MOTOR_SPEED, MOTOR_SPEED);
  PIDRight->SetOutputLimits(-1*MOTOR_SPEED, MOTOR_SPEED);
  PIDLeft->SetMode(AUTOMATIC);
  PIDRight->SetMode(AUTOMATIC);
}

// function to advance 1 cell, using wall follow if possible
void emile_motors::forward()
{
  // turn calibration
  /*
  moveTicks(-1 * TICKS_TURN, TICKS_TURN);
  wait(1000);
  moveTicks(-1 * TICKS_TURN, TICKS_TURN);
  wait(1000);
  moveTicks(-1 * TICKS_TURN, TICKS_TURN);
  wait(1000);
  moveTicks(-1 * TICKS_TURN, TICKS_TURN);
  wait(10000);
  */
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
  // if not able to wall follow, use odometry (trimmed down)
  else{
    moveTicks(TICKS_CELL * .52, TICKS_CELL * .52);
  }
}

// 180 degree left turn
void emile_motors::turnAroundLeft()
{
  turnLeft();
  turnLeft();
}

// 180 degree right turn
void emile_motors::turnAroundRight()
{
  turnRight();
  turnRight();
}

// 90 degree left turn
void emile_motors::turnLeft()
{
  if (_frontIR -> readRangeSingleMillimeters() < WALL_THRESHOLD){
    front_align();
  }
  moveTicks(-1 * TICKS_TURN, TICKS_TURN);
}

// 90 degree right turn
void emile_motors::turnRight()
{
  if (_frontIR -> readRangeSingleMillimeters() < WALL_THRESHOLD){
    front_align();
  }
  moveTicks(TICKS_TURN,-1 * TICKS_TURN);
}

// aligns robot to the wall in front, straightening position
// and leaving room so a 90 degree turn will result in center
void emile_motors::front_align()
{
  int d = _frontIR->readRangeSingleMillimeters();
  int rd = _rightDiagIR->readRangeSingleMillimeters();
  int ld = _leftDiagIR->readRangeSingleMillimeters();
  time = millis();
  do{
    // stage 1: move close to desired range (unconfirmed)
    int ticks_offset = (d - WALL_DISTANCE_FRONT) / .1;
    moveTicks(ticks_offset, ticks_offset);
    // correct angle
    rd = _rightDiagIR->readRangeSingleMillimeters();
    ld = _leftDiagIR->readRangeSingleMillimeters();
    int ticks_tilt = (rd - ld) / .5;
    moveTicks(-1 * ticks_tilt, ticks_tilt);
    d = _frontIR->readRangeSingleMillimeters();
  } while (abs(d - WALL_DISTANCE_FRONT) > 10 && (millis() - time < 3000));
  stop();
}

// follows the right wall for a certain number of ticks
void emile_motors::followTicksRight(int ticks)
{
  if (releaseFlag){
    return;
  }
  encoderLeft->write(0);
  encoderRight->write(0);
  int distanceL = 0;
  int distanceR = 0;
  PIDRight->SetTunings(4.20,0.01,0.01); // determine tunings
  // follow until combined motor ticks double the ticks parameter
  while (distanceL + distanceR < ticks * 2)
  {
    distanceL = encoderLeft->read();
    distanceR = encoderRight->read();
    InputR = _rightIR->readRangeSingleMillimeters(); //read right IR sensor
    SetpointR = WALL_DISTANCE_RIGHT;
    PIDRight->Compute();
    commandMotors((MOTOR_SPEED - OutputR)/2, (MOTOR_SPEED + OutputR)/2);
  }
  int off = (distanceL - distanceR) / 2;
  moveTicks(-1 * off, off);
  stop();
}


// follows the right wall for a certain number of ticks
void emile_motors::followTicksLeft(int ticks)
{
  if (releaseFlag){
    return;
  }
  encoderLeft->write(0);
  encoderRight->write(0);
  int distanceL = 0;
  int distanceR = 0;
  PIDLeft->SetTunings(4.20,.01,0.01); // determine tunings
  // follow until combined motor ticks double the ticks parameter
  while (distanceL + distanceR < ticks * 2)
  {
    distanceL = encoderLeft->read();
    distanceR = encoderRight->read();
    InputL = _leftIR->readRangeSingleMillimeters(); //read left IR sensor
    SetpointL = WALL_DISTANCE_LEFT;
    PIDLeft->Compute();
    commandMotors((MOTOR_SPEED + OutputL)/2, (MOTOR_SPEED - OutputL)/2);
    //commandMotors(OutputL, OutputL);
  }
  int off = (distanceL - distanceR) / 2;
  moveTicks(-1 * off, off);
  stop();
}

void emile_motors::advance(int ticks)
{
  // use right wall if available
  if (_rightIR->readRangeSingleMillimeters() < WALL_THRESHOLD){
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
void emile_motors::moveTicks(int Lticks, int Rticks)
{
  if (releaseFlag){
    return;
  }
  encoderLeft->write(0);
  encoderRight->write(0);
  SetpointL = Lticks;
  SetpointR = Rticks;
  PIDLeft->SetTunings(2.9,0.01,0.01);
  PIDRight->SetTunings(2.9,0.01,0.01);
  InputL = 0;
  InputR = 0;
  time = millis();
  while ((abs(InputL-SetpointL) > 10 || abs(InputR-SetpointR) > 10)
         && (millis() - time < 2000)){
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
  }
  stop();
}

// send power commands to left and right motors
void emile_motors::commandMotors(double left, double right)
{
  if (left >= 0){
    digitalWrite(pins::motorDirectionL, LOW);
  }
  else {
    digitalWrite(pins::motorDirectionL, HIGH);
  }
  analogWrite(pins::motorPowerL, abs(left));
  if (right >= 0){
    digitalWrite(pins::motorDirectionR, LOW);
  }
  else {
    digitalWrite(pins::motorDirectionR, HIGH);
  }
  analogWrite(pins::motorPowerR, abs(right));
}

// stop both motors
void emile_motors::stop()
{
  analogWrite(pins::motorPowerR, 0);
  analogWrite(pins::motorPowerL, 0);
}

// wait for milliseconds
void emile_motors::wait(int ms)
{
  time = millis();
  while (millis() - time < ms){}
}