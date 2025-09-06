#include "pid.hpp"
#include <algorithm>  // Para std::clamp (limitar valores)

double PID::compute(double setpoint, double measured_value) {
    // Calcula tempo delta (dt) desde o último ciclo
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_time_).count();
    last_time_ = now;

    // Se dt for zero ou muito pequeno, evita divisão por zero
    if (dt <= 0.0) dt = 0.001;  // Valor mínimo arbitrário

    // Calcula erro atual
    double error = setpoint - measured_value;

    // Termo proporcional
    double proportional = kp_ * error;

    // Termo integral (acumula erro ao longo do tempo)
    integral_ += error * dt;
    double integral_term = ki_ * integral_;

    // Termo derivativo (taxa de mudança do erro)
    double derivative = kd_ * (error - previous_error_) / dt;
    previous_error_ = error;

    // Saída total do PID
    double output = proportional + integral_term + derivative;

    // Limita a saída (ex.: para PWM entre 0 e 255, ajusta conforme teus motores)
    output = std::clamp(output, 0.0, 255.0);

    return output;
}

void PID::reset() {
    integral_ = 0.0;
    previous_error_ = 0.0;
    last_time_ = std::chrono::steady_clock::now();
}
