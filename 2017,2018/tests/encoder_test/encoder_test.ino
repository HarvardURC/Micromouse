/*
 * This test prints encoder values to the Serial Monitor for both wheels
 * To check, manually spin wheels and see if values change properly
 */

#include <i2c_t3.h>
#include <config.h>
#include "motors.hh"
#include "sensors.hh"

using namespace pins;

Motor* motorL;
Motor* motorR;
SensorArray* sensorArr;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    Serial.print("Right + Left Encoder Test:");

    motorL = new Motor(
        motorPowerL,
        motorDirectionL,
        encoderL1,
        encoderL2,
        *sensorArr);

    motorR = new Motor(
        motorPowerR,
        motorDirectionR,
        encoderR1,
        encoderR2,
        *sensorArr);

    pinMode(motorMode, OUTPUT);
    digitalWrite(motorMode, HIGH);
}


void loop() {
    Serial.print("Left encoder: ");
    Serial.print(motorL->readTicks());
    Serial.print(" Right encoder: ");
    Serial.println(motorR->readTicks());
    delay(20);
}
