/**
    A motor, sensor, and PID test.
    Directs the robot to maintain a position equal to a specified 
    distance from a stationary or moving wall.
*/

#include <VL6180X.h>
#include <i2c_t3.h>
#include <PID_v1.h>
#include "config.h"

VL6180X *frontIR;

int testingDistance = 50; // 50 millimeters
int minNum = 180;

// Define Variables we'll be connecting to for PID
float Setpoint, Input, Output, directionPin, powerPin;
PID myPID(&Input, &Output, &Setpoint, 2.0, 0.0018, 0, DIRECT);
unsigned long time;

void setup() {
    frontIR = new VL6180X;
    Serial.begin(9600);
    delay(1000);
    Serial.println("Initializing");
    // Initialize connection bus
    Wire.begin();

    // Initialize two motors
    pinMode(pins::motorPowerL, OUTPUT);
    pinMode(pins::motorDirectionL, OUTPUT);
    pinMode(pins::motorPowerR, OUTPUT);
    pinMode(pins::motorDirectionR, OUTPUT);

    // Initialize PID controllers
    Input = 0;
    Setpoint = testingDistance;
    myPID.SetOutputLimits(-50, 50);
    //turn the PID on
    myPID.SetMode(AUTOMATIC);

    // Initialize forward sensor
    int front = pins::tofFront;
    pinMode(front, OUTPUT);
    digitalWrite(front, HIGH);
    frontIR->init();
    frontIR->configureDefault();
    frontIR->setScaling(2);
    Serial.println("front sensor connected");

}

void loop() {
    time = millis();

    // Read sensor
    Serial.print("front sensor: ");
    Input = frontIR->readRangeSingleMillimeters() - 180;
    Serial.print(Input);
    Serial.print(" Setpoint: ");
    Serial.println(Setpoint);

    // Enact correction with motors
    myPID.Compute();
    if (Output >= 0) {
        directionPin = HIGH;
    } else {
        directionPin = LOW;
    }
    powerPin = abs(Output) * 2;

    digitalWrite(pins::motorDirectionL, directionPin);
    digitalWrite(pins::motorDirectionR, directionPin);
    analogWrite(pins::motorPowerL, powerPin);
    analogWrite(pins::motorPowerR, powerPin);
    delay(10);
}
