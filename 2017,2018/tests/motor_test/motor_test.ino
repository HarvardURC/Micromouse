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
const int speed = 150;
const int time = 1000;
int flag = 0;

#include <i2c_t3.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <config.h>
  
Adafruit_BNO055 bno = Adafruit_BNO055(55);//pins::imuRST);
 
uint8_t sys;
uint8_t gyro;
uint8_t accel;
uint8_t mag;

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

    /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
    Serial.println("Motors initialized.");

      delay(1000);
    
    bno.setExtCrystalUse(true);
}

void loop() {
    move({leftMotor, rightMotor}, FORWARD);
    bno.getCalibration(&sys, &gyro, &accel, &mag);

    Serial.print("Calibration: ");
    Serial.print("sys: ");
    Serial.print(sys);
    Serial.print("gyro: ");
    Serial.print(gyro);
    Serial.print("accel: ");
    Serial.print(accel);
    Serial.print("mag: ");
    Serial.print(mag);
    /*if (flag == 0) {
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
    }*/
}

void move(std::vector<motor> motors, int direction) {
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
    delay(time);
}
