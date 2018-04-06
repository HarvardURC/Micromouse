#include <VL6180X.h>

#ifndef sensors_hh
#define sensors_hh

#define LEFTDIAG 0
#define LEFTFRONT 1
#define RIGHTFRONT 2
#define RIGHTDIAG 3

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
