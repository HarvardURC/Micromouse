/*
    Test for motors. Runs left wheel, then right wheel,
    then both wheels.
*/

#include <vector>
#include <config.h>

#define FORWARD 1
#define BACKWARD 0
#define STOP -1

struct motor {
    String name;
    int powerPin;
    int directionPin;
} leftMotor, rightMotor;

void move(std::vector<motor> motors, int direction);

// Constants for test
int speed = 200;
int time = 2000;
int flag = 0;

void setup() {
    Serial.begin(9600);
    delay(1000);

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
;
    Serial.println("Motors initialized.");
}

void loop() {
    if (flag == 0) {
        move({rightMotor}, BACKWARD);
        move({rightMotor}, STOP);
        move({rightMotor}, FORWARD);
        move({rightMotor}, STOP);

        move({leftMotor}, BACKWARD);
        move({leftMotor}, STOP);
        move({leftMotor}, FORWARD);
        move({leftMotor}, STOP);

        move({leftMotor, rightMotor}, BACKWARD);
        move({leftMotor, rightMotor}, STOP);
        move({leftMotor, rightMotor}, FORWARD);
        move({leftMotor, rightMotor}, STOP);
        flag = 1;
    }
}

void move(std::vector<motor> motors, int direction) {
    for (unsigned int i = 0; i < motors.size(); i++) {
        if (direction == STOP) {
            analogWrite(motors[i].powerPin, 0);
        }
        else {
            Serial.print(motors[i].name);
            Serial.print(" motor moving ");
            String stringDirection = direction ? "backwards" : "forwards";
            Serial.print(stringDirection);
            Serial.println("...");

            analogWrite(motors[i].powerPin, speed);
            digitalWrite(motors[i].directionPin, direction);
        }
    }
    delay(time);
}
