#include <VL6180X.h>

class SensorArray {
    public:
        SensorArray(
            int tofLeftDiagSPin,
            int tofRightDiagSPin,
            int tofFrontSPin,
            int tofFrontLPin,
            int imuRSTPin
        );

        enum Sensor {
            leftDiag = 0, rightDiag, front
        };

        void readToSerial();
        // Index = 0 for
        int readShortTof(int sensor_index);
        int readLongTof();
        double readIMUAngle();

    private:
        void _initSensor(int pin, VL6180X* sensor);
};
