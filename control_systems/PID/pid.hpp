#ifndef PID_HPP
#define PID_HPP

#include <algorithm>

class PID {
public:
    // === Parameters ===
    double Kp = 7.0;  // proportional gain (was 5.0 as a prior tuning note)
    double Ki = 1.5;  // integral gain (was 0.5 as a prior tuning note)
    double Kd = 0.2;  // derivative gain (was 0.1 as a prior tuning note)

    double outputMin = 0.0;     // minimum PWM
    double outputMax = 30.0;    // maximum PWM
    double maxStepChange = 2.0; // max PWM step per cycle (%)

    // Anti-windup: absolute limit for the integral term (in output units)
    // Example: 30 => integral contribution is limited to ±30% PWM
    double Imax = 30.0;

    // Derivative filter (0 < alpha <= 1). 1.0 = no filter, 0.2 = strong filtering
    double dAlpha = 0.3;

    PID();
    double compute(double setpoint, double measurement, double dt);
    void reset();

private:
    // State
    double iTerm;        // accumulated integral contribution (output units)
    double prevMeasFilt; // previous filtered measurement (for derivative)
    double measFilt;     // current filtered measurement
    double lastOutput;   // last limited output

    bool first = true;
};

#endif