/*
    Test for motors. Runs left wheel, then right wheel,
    then both wheels.
*/

#include <vector>
#include <config.h>
#include <Encoder.h>

#define FORWARD 1
#define BACKWARD 0
#define STOP -1

struct motor {
    String name;
    int powerPin;
    int directionPin;
} leftMotor, rightMotor;

void move(std::vector<motor> motors, int direction);
void wait(int ms);

Encoder encoderLeft(pins::encoderL1, pins::encoderL2);
Encoder encoderRight(pins::encoderR1, pins::encoderR2);

// Constants for test
const int speed = 20;
int flag = 0;

void setup() {
    Serial.begin(9600);
    delay(1000);
    
    // quiet the buzzer
    pinMode(pins::buzzer, OUTPUT);
    digitalWrite(pins::buzzer, LOW);

    leftMotor.name = "Left";
    leftMotor.powerPin = pins::motorPowerL;
    leftMotor.directionPin = pins::motorDirectionL;
    rightMotor.name = "Right";
    rightMotor.powerPin = pins::motorPowerR;
    rightMotor.directionPin = pins::motorDirectionR;

    pinMode(pins::motorPowerL, OUTPUT);
    pinMode(pins::motorDirectionL, OUTPUT);
    pinMode(pins::motorPowerR, OUTPUT);
    pinMode(pins::motorDirectionR, OUTPUT);
    pinMode(pins::motorMode, OUTPUT);
    digitalWrite(pins::motorMode, HIGH);

    Serial.println("Motors initialized.");
}

void loop() {
    if (flag == 0) {
        // get the motor started up
        move({rightMotor, leftMotor}, FORWARD, 100);
        // start reading encoders
        encoderLeft.write(0);
        encoderRight.write(0);
        move({rightMotor, leftMotor}, FORWARD, 1000);
        Serial.println(encoderLeft.read());
        Serial.println(encoderRight.read());
        move({rightMotor, leftMotor}, STOP, 0);
    }
}

void move(std::vector<motor> motors, int direction, int time) {
    for (unsigned int i = 0; i < motors.size(); i++) {
        if (direction == STOP) {
            analogWrite(motors[i].powerPin, 0);
        }
        else {
            Serial.print(motors[i].name);
            Serial.print(" motor moving ");
            String stringDirection = direction ? "forwards" : "backwards";
            Serial.print(stringDirection);
            Serial.println("...");

            analogWrite(motors[i].powerPin, speed);
            digitalWrite(motors[i].directionPin, direction);
        }
    }
    wait(time);
}

// wait for milliseconds
void wait(int ms)
{
  int t = millis();
  while (millis() - t < ms){}
}

