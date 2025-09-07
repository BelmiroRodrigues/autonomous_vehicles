#ifndef PID_HPP
#define PID_HPP

#include <chrono>  // Para medir tempo (dt)

class PID {
public:
    PID() = default;
    double compute(double setpoint, double measured_value);
    void reset();

private:
    double kp_ = 14.0;    // Ganho proporcional
    double ki_ = 4.0;    // Ganho integral
    double kd_ = 2.0;    // Ganho derivativo

    double integral_ = 0.0;  // Acumulador do erro integral
    double previous_error_ = 0.0;  // Erro do ciclo anterior (para derivativo)

    std::chrono::time_point<std::chrono::steady_clock> last_time_ = std::chrono::steady_clock::now();
};

#endif
