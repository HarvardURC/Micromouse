/*
    Tests if the PID can successfully and smoothly reach
    a goal motor rotation as measued by the encoder.
    (bare bones)
*/

#include <PID_v1.h>
#include <Encoder.h>
#include "config.h"

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
Encoder encoderLeft(pins::encoderL1, pins::encoderL2);

//Specify the links and initial tuning parameters
int goal = 1000000;
double proportion = 0.001;
double integral = 0;
double derivative = 0;

PID myPID(&Input, &Output, &Setpoint, proportion, integral, derivative, DIRECT);

void setup()
{
    Serial.begin(9600);
    delay(1000);

    //set up motors
    pinMode(pins::motorDirectionL, OUTPUT);
    pinMode(pins::motorPowerL, OUTPUT);
    pinMode(pins::motorMode, OUTPUT);
    digitalWrite(pins::motorMode, HIGH);

    //initialize the variables we're linked to
    Input = 0;
    encoderLeft.write(0);
    Setpoint = goal;
    myPID.SetOutputLimits(-50, 50);

    //turn the PID on
    myPID.SetMode(AUTOMATIC);
}

void loop()
{
    Input = encoderLeft.read();
    myPID.Compute();

    Serial.println(Output);

    digitalWrite(pins::motorDirectionL, Output >= 0 ? HIGH : LOW);
    analogWrite(pins::motorPowerL, abs(Output));
}


