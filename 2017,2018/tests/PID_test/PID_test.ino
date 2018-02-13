/*
    Tests if the PID can successfully and smoothly reach
    a goal motor rotation as measued by the encoder. 
*/

#include <PID_v1.h>
#include <Encoder.h>
#include "config.h"

//Define Variables we'll be connecting to
double Setpoint, Input, Output, direction_pin, power_pin;
Encoder encoderLeft(pins::encoderPinL1, pins::encoderPinL2);

//Specify the links and initial tuning parameters
int goal = 87000;
double proportion = .0006;
double integral = 0;
double derivative = 0;

PID myPID(&Input, &Output, &Setpoint, proportion, integral, derivative, DIRECT);
unsigned long time;

void setup()
{
    Serial.begin(9600);
    delay(2000);

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

    //timeout
    startTime = millis();
}

void loop()
{
    time = millis();
    Input = encoderLeft.read();
    myPID.Compute();

    if (millis() - startTime > timeout) {
      Output = 0;
    }

    Serial.println(Input);

    if (Output >= 0){
    direction_pin = HIGH;
    }
    else {
    direction_pin = LOW;
    }
  
    power_pin = abs(Output); 
    Serial.println(Output);
    //Serial.println(direction_pin);
    digitalWrite(pins::motorDirectionL, direction_pin);
    analogWrite(pins::motorPowerL, power_pin);
}

