/* Helper functions */
#include <PID_v1.h>

#ifndef helpers_hh
#define helpers_hh

/* Math functions */
// Function for taking the modulus of a double e.g. `200.56 % 10` = 0.56
// From https://stackoverflow.com/questions/9138790/cant-use-modulus-on-doubles
template<typename T, typename U>
constexpr T dmod (T x, U mod)
{
    return !mod ? x :
        static_cast<long long>(x) % mod + x - static_cast<long long>(x);
}

bool withinError(float a, float b, float error);


/* PWM value functions */
int floorPWM(int speed, int floor);
float ceilingPWM(float speed, float otherspeed, int limit);


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

#endif
