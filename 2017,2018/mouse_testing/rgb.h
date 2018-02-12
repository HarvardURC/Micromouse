#ifndef RGB_LED_h
#define RGB_LED_h

class RGB_LED
{
    public:
        // Constructor
        RGB_LED(int redPin, int greenPin, int bluePin);

        // Flashes the light on an off
        void flashLED(int color);

        // Switches the light on if off and vice versa
        void switchLED(int color);

        int rgbPins[3];
        int rgbState[3];

    private:
        void _turnOn(int color);
        void _turnOff(int colo);
};

#endif
