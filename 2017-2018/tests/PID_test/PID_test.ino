/*
    Tests if the PID can successfully and smoothly reach
    a goal motor rotation as measued by the encoder.
    (bare bones)
*/

#include <PID_v1.h>
#include <Encoder.h>
#include "config.h"

using namespace pins;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
Encoder encoderLeft(encoderL1, encoderL2);
Encoder encoderRight(encoderR1, encoderR2);

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
    pinMode(motorDirectionR, OUTPUT);
    pinMode(motorPowerR, OUTPUT);

    pinMode(motorDirectionL, OUTPUT);
    pinMode(motorPowerL, OUTPUT);

    pinMode(motorMode, OUTPUT);
    digitalWrite(motorMode, HIGH);

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
    while(millis() < 5000) {
        Input = encoderLeft.read();
        myPID.Compute();

        Serial.print("Left motor: ");
        Serial.println(Output);

        digitalWrite(motorDirectionL, Output >= 0 ? HIGH : LOW);
        analogWrite(motorPowerL, abs(Output));
    }
    while(millis() < 10000) {
        Input = encoderRight.read();
        myPID.Compute();

        Serial.print("Right motor: ");
        Serial.println(Output);

        digitalWrite(motorDirectionR, Output >= 0 ? HIGH : LOW);
        analogWrite(motorPowerR, abs(Output));
    }
    Serial.println("Finished.");
}


