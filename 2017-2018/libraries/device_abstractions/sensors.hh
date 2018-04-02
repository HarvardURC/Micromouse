#include <VL6180X.h>

#ifndef sensors_hh
#define sensors_hh

class SensorArray {
    public:
        SensorArray(
            int tofLeftDiagSPin,
            int tofRightDiagSPin,
            int tofFrontSPin,
            int tofFrontLPin,
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