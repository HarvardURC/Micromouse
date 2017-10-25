#include <PID_v1.h>
#include <Encoder.h>
#include "config.h"

//Define Variables we'll be connecting to
double Setpoint, Input, Output, direction_pin, power_pin;
Encoder knobLeft(1, 2);

//Specify the links and initial tuning parameters
//1,.0017,.01
int goal = -2000;
double proportion = 1;
double integral = 0.0018;
double derivative = 0.01;

PID myPID(&Input, &Output, &Setpoint, proportion, integral, derivative, DIRECT);
unsigned long time;

void setup()
{
    Serial.begin(9600);
    delay(2000);

    //set up motors
    pinMode(pins::motorDirectionR, OUTPUT);
    pinMode(pins::motorPowerR, OUTPUT);

    //initialize the variables we're linked to
    Input = 0;
    knobLeft.write(0);
    Setpoint = goal;
    myPID.SetOutputLimits(-100, 100);

    //turn the PID on
    myPID.SetMode(AUTOMATIC);
}

void loop()
{
    time = millis();
    Input = knobLeft.read();
    myPID.Compute();

    Serial.println(Input);

    if (Output >= 0){
    direction_pin = HIGH;
    }
    else {
    direction_pin = LOW;
    }
  
    power_pin = abs(Output); 
    Serial.println(power_pin);
    Serial.println(direction_pin);
    digitalWrite(pins::motorDirectionR, direction_pin);
    analogWrite(pins::motorPowerR, power_pin);
}

