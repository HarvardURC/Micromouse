#include <VL6180X.h>
#include <Arduino.h>
#include <i2c_t3.h>
#include <VL6180X.h>
#include <vector>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#ifndef sensors_hh
#define sensors_hh

#define LEFTDIAG 0
#define LEFTFRONT 1
#define RIGHTFRONT 2
#define RIGHTDIAG 3

extern Adafruit_BNO055 bno;

class SensorArray {
    public:
        SensorArray(
            int tofDiagLPin,
            int tofFrontLPin,
            int tofFrontRPin,
            int tofDiagRPin,
            int imuRSTPin
        );
        SensorArray(const SensorArray& sensors);

        enum Sensor {
            leftDiag = 0, rightDiag, front
        };

        void readToSerialTof();
        void readToSerial();
        // Index = 0 for
        int readShortTof(int sensor_index);
        int readLongTof();
        double readIMUAngle();

    private:
        void _initSensor(int idx);
};

#endif
