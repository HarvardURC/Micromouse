#include <QueueArray.h>
#include <vector>
#include <emile_motors.h>
#include <VL6180X.h>
#include <i2c_t3.h>
#include <config.h>

std::vector<int> sensor_pins = {pins::tofLeft, pins::tofLeftDiag, pins::tofFront, pins::tofRightDiag, pins::tofRight};
std::vector<VL6180X*> sensors = {new VL6180X, new VL6180X, new VL6180X, new VL6180X, new VL6180X};

emile_motors* motors = new emile_motors(sensors[0], sensors[4], sensors[2], sensors[1], sensors[3]);

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
    Serial.println("Start loop");
    motors->forward();
    delay(500);
    motors->turnRight();
    delay(500);
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
