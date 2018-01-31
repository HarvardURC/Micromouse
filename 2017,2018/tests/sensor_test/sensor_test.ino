/**
  A time of flight (ToF) sensor test.
  Outputs to Serial console the distance measured
  by the left and right sensors.
*/

#include <vector>
#include <VL6180X.h>
#include <i2c_t3.h>
#include <config.h>
#include <VL53L0X.h>

// VL6180X Sensors
std::vector<int> sensor_pins = {pins::tofLeftDiagS, pins::tofRightDiagS, pins::tofFrontS};
std::vector<VL6180X*> sensors = {new VL6180X, new VL6180X, new VL6180X};
std::vector<String> sensor_names = {"leftDiagS", "rightDiagS", "frontS"};

void wait(int ms);
void initSensor(int pin, VL6180X *sensor, int address);

void setup() {
  // Sets all sensors to low for initialization
  for (unsigned int i = 0; i < sensor_pins.size(); i++) {
    pinMode(sensor_pins[i], OUTPUT);
    digitalWrite(sensor_pins[i], LOW);
  }
  Serial.begin(9600);
  delay(1000);
  Serial.println("Initializing:");
  Wire.begin();

  // Set sensor addresses
  for (unsigned int i = 0; i < sensor_pins.size(); i++) {
    digitalWrite(sensor_pins[i], HIGH);
    delay(500);
    sensors[i]->setAddress(2 * i);
    Serial.println(sensors[i]->readReg(0x212));
  }

  // Initializes sensors
  for (unsigned int i = 0; i < sensor_pins.size(); i++) {
    initSensor(sensor_pins[i], sensors[i]);
  }
}

void loop() {
  unsigned long start = millis();
  // Prints debug distances for sensors
  for (unsigned int i = 0; i < sensors.size(); i++) {
    Serial.print(sensor_names[i]);
    Serial.print(": ");
    Serial.print(sensors[i]->readRangeContinuous());
    Serial.print(" ");
    if (sensors[i]->timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  }
  Serial.print("Time taken: ");
  Serial.println(millis() - start);
  wait(10);
}


/* Helper Functions */
void wait(int ms)
{
  unsigned long previousMillis = millis();
  unsigned long currentMillis = millis();
  while ((currentMillis - previousMillis) < (unsigned long)ms)
  {
    currentMillis = millis();
  }
}

void initSensor(int pin, VL6180X *sensor) {
    sensor->init();
    sensor->configureDefault();
    sensor->setScaling(2);
    sensor->setTimeout(500);
    sensor->writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 20);
    sensor->stopContinuous();
    delay(300);
    sensor->startRangeContinuous(30);
    Serial.print(pin);
    Serial.println(" connected.");
    delay(1000);
}
