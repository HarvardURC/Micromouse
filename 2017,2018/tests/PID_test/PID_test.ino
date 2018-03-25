/*
    Tests if the PID can successfully and smoothly reach
    a goal motor rotation as measued by the encoder. 
*/

#include <PID_v1.h>
#include <Encoder.h>
#include "config.h"

//NOTE: need to change the PID library to use floats, not doubles!
//Define Variables we'll be connecting to
float Setpoint, Input, Output, direction_pin, power_pin;
Encoder encoderLeft(pins::encoderPinL1, pins::encoderPinL2);
Encoder encoderRight(pins::encoderPinR1, pins::encoderPinR2);

//Specify the links and initial tuning parameters
int goal = 1000;
float proportion = 1;
float integral = 0.0018;
float derivative = 0.01;

PID myPID(&Input, &Output, &Setpoint, proportion, integral, derivative, DIRECT);
unsigned long time;

void setup()
{
    Serial.begin(9600);
    delay(2000);

    //set up motors
    pinMode(pins::motorDirectionR, OUTPUT);
    pinMode(pins::motorPowerR, OUTPUT);

    pinMode(pins::motorDirectionL, OUTPUT);
    pinMode(pins::motorPowerL, OUTPUT);

    //initialize the variables we're linked to
    Input = 0;
    //encoderLeft.write(0);
    //encoderRight.write(0);
    Setpoint = goal;
    myPID.SetOutputLimits(-100, 100);

    //turn the PID on
    myPID.SetMode(AUTOMATIC);
}

void loop()
{
    time = millis();
    Input = encoderLeft.read();
    myPID.Compute();

    Serial.println(Input);

    if (Output >= 0){
    direction_pin = LOW;
    }
    else {
    direction_pin = HIGH;
    }
  
    power_pin = abs(Output);
    Serial.print("Power: ");
    Serial.print(power_pin);
    Serial.print("  Direction: ");
    Serial.print(direction_pin);
    Serial.print("  Left Encoder: ");
    Serial.print(encoderLeft.read());
    Serial.print("  Right Encoder: ");
    Serial.print(encoderRight.read());
    digitalWrite(pins::motorDirectionR, direction_pin);
    digitalWrite(pins::motorDirectionL, direction_pin);
    analogWrite(pins::motorPowerR, power_pin);
    analogWrite(pins::motorPowerL, power_pin);
}

