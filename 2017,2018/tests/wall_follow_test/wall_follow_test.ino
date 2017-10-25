/**
    A motor, sensor, and PID test.
    Directs the robot to maintain a position equal to a specified 
    distance from a stationary or moving wall.
*/

#include <VL6180X.h>
#include <i2c_t3.h>
#include <PID_v1.h>
#include "config.h"

VL6180X frontIR;

int testingDistance = 50; // 50 millimeters
int minNum = 180;

//Define Variables we'll be connecting to for PID
double Setpoint, Input, Output, directionPin, powerPin;
PID myPID(&Input, &Output, &Setpoint,2.0,0.0018,0, DIRECT);
unsigned long time;

void setup() {
    Serial.begin(9600);
    delay(1000);
    Serial.println("Initializing");
    // Initialize connection bus
    Wire.begin(I2C_MASTER, 0, I2C_PINS_16_17, I2C_PULLUP_EXT, 50000);

    // Initialize forward sensor
    int reset_pin_front = pins::tofFront;
    pinMode(reset_pin_front, OUTPUT);
    digitalWrite(reset_pin_front, HIGH);
    frontIR.init();
    frontIR.configureDefault();
    frontIR.setScaling(2);
    Serial.print("front sensor connected");

    // Initialize two motors
    pinMode(pins::motorPowerL, OUTPUT);
    pinMode(pins::motorDirectionL, OUTPUT);
    pinMode(pins::motorPowerR, OUTPUT);
    pinMode(pins::motorDirectionR, OUTPUT);

    // Initialize PID controllers
    Input = 0;
    Setpoint = testingDistance;
    myPID.SetOutputLimits(-200, 200);
    //turn the PID on
    myPID.SetMode(AUTOMATIC);

}

void loop() {
    time = millis();

    // Read sensor
    Serial.print("front sensor: ");
    Input = frontIR.readRangeSingleMillimeters();
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
}
