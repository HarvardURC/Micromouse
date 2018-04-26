/*
 * Contains header information for input and output devices
 * on the robot including the RGB LEDs, the buzzer, and the buttons.
 */

#ifndef io_hh
#define io_hh

class RGB_LED
{
    public:
        // Constructor
        RGB_LED(int redPin, int greenPin, int bluePin);

        // Flashes the light on an off
        void flashLED(int color);

        // Switches the light on if off and vice versa
        void switchLED(int color);

        void turnOn(int color);
        void turnOff();

        int rgbPins[3];
        int rgbState[3];

    // private:
        void _turnOn(int color);
        void _turnOff(int color);
};


class Button {
    public:
        // Constructor
        Button(int buttonPin);

        int read();

    private:
        int _buttonPin;
};


class Buzzer {
    public:
        // Constructor
        Buzzer(int buzzerPin);

        void siren();
        void on();
        void off();

    private:
        int _buzzerPin;
};

#endif
