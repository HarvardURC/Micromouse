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

/* A wrapper class for encoders to keep track of last accessed tick value */
class EncoderTicker {
    public:
        EncoderTicker(Encoder* e_);
        long diffLastRead() {
            long curr = read();
            long r = curr - lastVal;
            lastVal = curr;
            return r;
        };
        long read() { return e->read(); }
    private:
        Encoder* e;
        long lastVal;
};


/* A structure for containing different sets of speedrun constants. */
struct DriverConfig {
    DriverConfig(int mL, int cT, float pl, float il, float dl,
        float pa, float ia, float da)
    : motorLimit(mL),
    convergenceTime(cT),
    p_l(pl),
    i_l(il),
    d_l(dl),
    p_a(pa),
    i_a(ia),
    d_a(da) {};

    // config items
    int motorLimit;
    int convergenceTime;

    float p_l;
    float i_l;
    float d_l;

    float p_a;
    float i_a;
    float d_a;

    void print();
};

#endif
