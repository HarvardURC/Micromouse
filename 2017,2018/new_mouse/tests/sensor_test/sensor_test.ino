/**
  A time of flight (ToF) sensor test.
  Outputs to Serial console the distance measured
  by the left and right sensors.
*/

#include <vector>
#include <VL6180X.h>
#include <i2c_t3.h>
#include <config.h>

std::vector<int> sensor_pins = {pins::tofLeft, pins::tofLeftDiag, pins::tofFront, pins::tofRightDiag, pins::tofRight};
std::vector<VL6180X*> sensors = {new VL6180X, new VL6180X, new VL6180X, new VL6180X, new VL6180X};
std::vector<String> sensor_names = {"left", "leftDiag", "front", "rightDiag", "right"};

void wait(int ms);
void initSensor(int pin, VL6180X *sensor, int address);

void setup() {
  Wire.begin();
  Serial.begin(9600);
  delay(1000);
  Serial.println("Initializing:");
  
  // Sets all sensors to low for initialization
  for (unsigned int i = 0; i < sensor_pins.size(); i++) {
    pinMode(sensor_pins[i], OUTPUT);
    digitalWrite(sensor_pins[i], LOW);
  }

  // Initializes sensors
  for (unsigned int i = 0; i < sensor_pins.size(); i++) {
    initSensor(sensor_pins[i], sensors[i], i + 1);
    Serial.println(i);
  }
}

void loop() {
  unsigned long start = millis();
  // Prints debug distances for sensors
  for (unsigned int i = 0; i < sensors.size(); i++) {
    Serial.print(sensor_names[i]);
    Serial.print(": ");
    Serial.print(sensors[i]->readRangeContinuousMillimeters());
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

void initSensor(int pin, VL6180X *sensor, int address) {
    digitalWrite(pin, HIGH);
    sensor->init();
    sensor->configureDefault();
    sensor->setScaling(2);
    sensor->setAddress(address);
    sensor->setTimeout(500);
    sensor->writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 20);
    sensor->stopContinuous();
    delay(300);
    sensor->startRangeContinuous(30);
    Serial.print(pin);
    Serial.println(" connected.");
}
