/**
    A motor, sensor, and PID test.
    Directs the robot to maintain a position equal to a specified
    distance from a stationary or moving wall.
*/

#include <VL6180X.h>
#include <i2c_t3.h>
#include <PID_v1.h>
#include "config.h"

VL6180X* tofFrontS = new VL6180X;

int testingDistance = 50; // 50 millimeters
int minNum = 180;

// Define Variables we'll be connecting to for PID
double Setpoint, Input, Output, directionPin, powerPin;
PID myPID(&Input, &Output, &Setpoint, 2.0, 0.0018, 0, DIRECT);
unsigned long time;

void setup() {
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
    int front = pins::tofFrontS;
    pinMode(front, OUTPUT);
    digitalWrite(front, HIGH);
    tofFrontS->init();
    tofFrontS->configureDefault();
    tofFrontS->setScaling(2);
    Serial.println("front sensor connected");

}

void loop() {
    time = millis();

    // Read sensor
    Serial.print("front sensor: ");
    Input = tofFrontS->readRangeSingleMillimeters() - 180;
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