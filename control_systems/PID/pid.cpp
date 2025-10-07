#include "pid.hpp"
#include <cmath>

PID::PID()
    : iTerm(0.0), prevMeasFilt(0.0), measFilt(0.0), lastOutput(0.0), first(true) {}

double PID::compute(double setpoint, double measurement, double dt) {
    if (dt <= 0.0) return lastOutput;

    // First-order filter on measurement for derivative term (reduces noise)
    if (first) {
        measFilt = measurement;
        prevMeasFilt = measurement;
        first = false;
    } else {
        measFilt = measFilt + dAlpha * (measurement - measFilt);
    }

    // Error for P and I
    double error = setpoint - measurement;

    // Derivative on measurement (avoids derivative kick on setpoint changes)
    double dMeas = (measFilt - prevMeasFilt) / dt;
    double dTerm = -Kd * dMeas; // negative sign since it is d(measurement)/dt
    prevMeasFilt = measFilt;

    // Proportional
    double pTerm = Kp * error;

    // Candidate integral: applied to iTerm only if anti-windup conditions hold
    double iCandidate = iTerm + Ki * error * dt;

    // Unsaturated output using candidate integral
    double outUnsat = pTerm + iCandidate + dTerm;

    // Conditional-integration anti-windup:
    // 1) If outUnsat is within limits -> accept iCandidate.
    // 2) If saturated, integrate only if it drives the output back toward the valid range.
    bool within = (outUnsat >= outputMin && outUnsat <= outputMax);
    bool helpsUpper = (outUnsat > outputMax) && (error < 0.0); // negative error pulls down
    bool helpsLower = (outUnsat < outputMin) && (error > 0.0); // positive error pushes up

    if (within || helpsUpper || helpsLower) {
        iTerm = iCandidate;
        // Limit integral contribution
        iTerm = std::clamp(iTerm, -Imax, Imax);
    }

    // Recompute with final iTerm
    double rawOutput = pTerm + iTerm + dTerm;

    // Output saturation
    if (rawOutput > outputMax) rawOutput = outputMax;
    else if (rawOutput < outputMin) rawOutput = outputMin;

    // Per-cycle slew limiting (extra smoothing)
    double limitedOutput = std::clamp(
        rawOutput,
        lastOutput - maxStepChange,
        lastOutput + maxStepChange
    );

    lastOutput = limitedOutput;
    return limitedOutput;
}

void PID::reset() {
    iTerm = 0.0;
    lastOutput = 0.0;
    first = true;
}
