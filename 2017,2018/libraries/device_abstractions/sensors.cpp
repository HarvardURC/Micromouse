#include <Arduino.h>
#include <i2c_t3.h>
#include <VL53L0X.h>
#include <VL6180X.h>
#include <vector>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "config.h"
#include "sensors.hh"

std::vector<int> sensor_pins = {pins::tofLeftDiagS, pins::tofRightDiagS, pins::tofFrontS};
std::vector<VL6180X*> sensors = {new VL6180X, new VL6180X, new VL6180X};
std::vector<String> sensor_names = {"leftDiagS", "rightDiagS", "frontS"};
auto long_sensor = new VL53L0X;

Adafruit_BNO055 bno = Adafruit_BNO055(55);//pins::imuRST);


SensorArray::SensorArray(int tofLeftDiagSPin, int tofRightDiagSPin,
        int tofFrontSPin, int tofFrontLPin, int imuRSTPinn) {
    for (unsigned int i = 0; i < sensor_pins.size(); i++) {
      pinMode(sensor_pins[i], OUTPUT);
      digitalWrite(sensor_pins[i], LOW);
    }
    // Special initialization for long distance sensor
    pinMode(pins::tofFrontL, OUTPUT);
    digitalWrite(pins::tofFrontL, LOW);

    delay(500);

    Wire.begin();

    // Set sensor addresses
    for (unsigned int i = 0; i < sensor_pins.size(); i++) {
      digitalWrite(sensor_pins[i], HIGH);
      delay(400);
      sensors[i]->setAddress(2 * i);
      // Uncomment to debug addresses of sensors
      // Serial.println(sensors[i]->readReg(0x212));
    }
    digitalWrite(pins::tofFrontL, HIGH);
    delay(400);
    long_sensor->setAddress(sensor_pins.size() * 2);

    // Initializes sensors
    for (unsigned int i = 0; i < sensor_pins.size(); i++) {
      _initSensor(i);
    }
    long_sensor->init();
    long_sensor->setTimeout(500);
    long_sensor->startContinuous();
    Serial.println("frontL online.");

    Serial.println("IMU online."); Serial.println("");

    /* Initialise the IMU */
    if(!bno.begin())
    {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while(1);
    }

    delay(1000);

    bno.setExtCrystalUse(true);
}

SensorArray::SensorArray(const SensorArray& sensors) {
  return;
}

void SensorArray::readToSerial() {
    unsigned long start = millis();
    // Prints debug distances for sensors
    for (unsigned int i = 0; i < sensors.size(); i++) {
      Serial.print(sensor_names[i]);
      Serial.print(": ");
      Serial.print(sensors[i]->readRangeContinuous());
      Serial.print(" ");
      if (sensors[i]->timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    }
    Serial.print("frontL: ");
    Serial.print(long_sensor->readRangeContinuousMillimeters());
    Serial.print(" Time taken: ");
    Serial.println(millis() - start);

    /* Get a new sensor event */
    sensors_event_t event;
    bno.getEvent(&event);

    /* Display the floating point data */
    Serial.print("X: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(event.orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.print(event.orientation.z, 4);
    Serial.println("");
    delay(10);

}


int SensorArray::readShortTof(int sensor_index) {
    return sensors[sensor_index]->readRangeContinuous();
}


int SensorArray::readLongTof() {
    return long_sensor->readRangeContinuousMillimeters();
}


// Returns angle from 0 to 360 depending on rotation on flat plane
double SensorArray::readIMUAngle() {
    sensors_event_t event;
    bno.getEvent(&event);
    return event.orientation.x;
}


void SensorArray::_initSensor(int idx) {
    VL6180X* sensor = sensors[idx];
    sensor->init();
    sensor->configureDefault();
    sensor->setScaling(2);
    sensor->setTimeout(500);
    sensor->writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 20);
    sensor->stopContinuous();
    delay(300);
    sensor->startRangeContinuous(30);
    Serial.print(sensor_names[idx]);
    Serial.println(" online.");
    delay(1000);
}


// Calculates a correction factor for the angular velocity to reduce drift
// x < 1 when turned left and x > 1 when turned right
// float SensorArray::correctionFactor() {
//     float left = readShortTof(0);
//     float right = readShortTof(1);
//     return left > short_range || right > short_range ? 0 : right / left;
// }
