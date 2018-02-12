class Motor {
    public:
        // Constructor
        Motor(int powerPin, int directionPin);

        void drive(int speed);

    private:
        int _powerPin;
        int _directionPin;
};

class Driver {
    public:
        // Constructor
        Driver(
            int powerPinL,
            int directionPinL,
            int powerPinR,
            int directionPinR,
            int motorModePin
        );

        void drive(int speed);

    private:
        Motor _leftMotor;
        Motor _rightMotor;
};
