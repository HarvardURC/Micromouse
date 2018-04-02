/*
    Tests if both the PIDs can successfully and smoothly reach
    a goal motor rotation as measued by the encoder. 
*/

#include <PID_v1.h>
#include <Encoder.h>
#include "config.h"

//NOTE: need to change the PID library to use floats, not doubles!
//Define Variables we'll be connecting to
float SetpointL, InputL, OutputL, direction_pin_L, power_pin_L;
float SetpointR, InputR, OutputR, direction_pin_R, power_pin_R;
Encoder encoderLeft(pins::encoderPinL1, pins::encoderPinL2);
Encoder encoderRight(pins::encoderPinR1, pins::encoderPinR2);

//Specify the links and initial tuning parameters
int goal = 1193;
float proportion = 0.95;
float integral = 0.0;
float derivative = 0.01;

PID PID_L(&InputL, &OutputL, &SetpointL, proportion, integral, derivative, DIRECT);
PID PID_R(&InputR, &OutputR, &SetpointR, proportion, integral, derivative, DIRECT);
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
    InputL = 0;
    InputR = 0;
    //encoderLeft.write(0);
    //encoderRight.write(0);
    SetpointL = goal;
    SetpointR = goal;
    PID_L.SetOutputLimits(-100, 100);
    PID_R.SetOutputLimits(-100, 100);

    //turn the PID on
    PID_L.SetMode(AUTOMATIC);
    PID_R.SetMode(AUTOMATIC);
}

void loop()
{
    time = millis();
    InputL = encoderLeft.read();
    InputR = encoderRight.read();
    PID_L.Compute();
    PID_R.Compute();

    Serial.print("Left Wheel Input: ");
    Serial.println(InputL);

    Serial.print("Right Wheel Input: ");
    Serial.println(InputR);

    if (OutputL >= 0){
    direction_pin_L = LOW;
    }
    else {
    direction_pin_L = HIGH;
    }

    if (OutputR >= 0){
    direction_pin_R = LOW;
    }
    else {
    direction_pin_R = HIGH;
    }
  
    power_pin_L = abs(OutputL);
    power_pin_R = abs(OutputR);
    Serial.print("Power Left: ");
    Serial.print(power_pin_L);
    Serial.print("  Direction Left: ");
    Serial.print(direction_pin_L);
    Serial.print("Power Right: ");
    Serial.print(power_pin_R);
    Serial.print("  Direction Right: ");
    Serial.print(direction_pin_R);
    Serial.print("  Left Encoder: ");
    Serial.print(encoderLeft.read());
    Serial.print("  Right Encoder: ");
    Serial.print(encoderRight.read());
    digitalWrite(pins::motorDirectionR, direction_pin_R);
    digitalWrite(pins::motorDirectionL, direction_pin_L);
    analogWrite(pins::motorPowerR, power_pin_R);
    analogWrite(pins::motorPowerL, power_pin_L);
}

