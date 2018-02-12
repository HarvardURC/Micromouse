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

        void readToSerial();

    private:
        void _initSensor(int pin, VL6180X* sensor);
};
