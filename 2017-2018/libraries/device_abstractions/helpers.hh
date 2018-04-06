/* Helper functions */
#include <Arduino.h>
#include <PID_v1.h>
#include <Encoder.h>
#include "bluetooth.hh"
#include "sensors.hh"

#ifndef helpers_hh
#define helpers_hh

 /* Prints the variable's name followed by the value.
  * Ex. debug_printvar(v_left) => (to log) "v_left: -17.9" */
#define debug_printvar(var) debug_print(#var); debug_print(": "); debug_println(var)

/* Math functions */
bool withinError(float a, float b, float error);
inline float wrapAngle(float angle) {
    return angle - 2 * PI * floor(angle / (2 * PI));
}


/* PWM value functions */
int floorPWM(int speed, int floor);
float ceilingPWM(float speed, float otherspeed, int limit);


/* Debugging functions
 *
 * Prints to bluetooth if connected, otherwise prints to Serial monitor. */
template<typename T>
void debug_print(T arg) {
    if (bleReady()) {
        ble.print(arg);
    } else {
        Serial.print(arg);
    }
}

template<typename T>
void debug_println(T arg) {
    if (bleReady()) {
        ble.println(arg);
    } else {
        Serial.println(arg);
    }
}


/* A wrapper class to improve the usability of the Arduino PID library. */
class PidController {
    public:
        PidController(
            float proportion,
            float integral,
            float derivative
        );

        void operator=(const PidController& pid) {
            proportion = pid.proportion;
            integral = pid.integral;
            derivative = pid.derivative;
        }

        void compute();
        void setTunings(float p, float i, float d);
        void printTunings();

        float proportion;
        float integral;
        float derivative;

        float input;
        float output;
        float setpoint;
    private:
        PIDT<float> _pid;
};


class Ticker {
    public:
        Ticker() {};

        virtual float diffLastRead() {
            float curr = read();
            float r = curr - lastVal;
            lastVal = curr;
            return r;
        };
        virtual float read() { return 0; }

        float lastVal;
};


/* A wrapper class for encoders to keep track of last accessed tick value */
class EncoderTicker : public Ticker {
    public:
        EncoderTicker(Encoder* e_);
        float read() override { return (float)e->read(); }
    private:
        Encoder* e;
};

#endif
