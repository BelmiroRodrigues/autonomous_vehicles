#include "SpeedPIDController.hpp"
#include <algorithm>
#include <cmath>

SpeedPIDController::SpeedPIDController(float kp, float ki, float kd,
                                       float pwm_min, float pwm_max,
                                       bool use_backcalc,
                                       float kb,
                                       float Tf)
: kp_(kp), ki_(ki), kd_(kd),
  pwm_min_(pwm_min), pwm_max_(pwm_max),
  kb_(kb), Tf_(Tf),
  prev_v_(0.0f), prev_error_(0.0f),
  integral_(0.0f), d_filt_(0.0f),
  use_backcalc_(use_backcalc), initialized_(false)
{
    if (pwm_max_ < pwm_min_) std::swap(pwm_min_, pwm_max_);
    if (Tf_ < 0.0f) Tf_ = 0.0f;
}

void SpeedPIDController::reset() {
    prev_v_ = 0.0f;
    prev_error_ = 0.0f;
    integral_ = 0.0f;
    d_filt_ = 0.0f;
    initialized_ = false;
}

float SpeedPIDController::update(float v_current, float v_target, float dt) {
    // Proteção básica de dt
    if (dt <= 0.0f) dt = 1e-3f;

    // Inicialização bumpless
    if (!initialized_) {
        prev_v_ = v_current;
        prev_error_ = v_target - v_current;
        d_filt_ = 0.0f;
        initialized_ = true;
    }

    // Erro
    const float error = v_target - v_current;

    // Derivada na medição com filtro de 1ª ordem:
    // dv = (v - v_prev)/dt; filtro: y_f = alpha*y_f + (1-alpha)*dv, alpha = Tf/(Tf+dt)
    const float dv = (v_current - prev_v_) / dt;
    const float alpha = (Tf_ > 0.0f ? Tf_ / (Tf_ + dt) : 0.0f);
    d_filt_ = alpha * d_filt_ + (1.0f - alpha) * dv;

    // Termo D (sobre medição): sinal negativo evita "derivative kick"
    const float d_term = -kd_ * d_filt_;

    // Saída não saturada
    const float u_raw = kp_ * error + ki_ * integral_ + d_term;

    // Saturação
    const float u_sat = std::clamp(u_raw, pwm_min_, pwm_max_);

    // Anti-windup
    if (use_backcalc_) {
        // Back-calculation: integra erro + kb*(u_sat - u_raw)
        const float anti = kb_ * (u_sat - u_raw);
        integral_ += (error + anti) * dt;
    } else {
        // Conditional integration: pausa integral quando saturado e erro empurra mais a saturação
        const bool at_hi = (u_raw >= pwm_max_) && (error > 0.0f);
        const bool at_lo = (u_raw <= pwm_min_) && (error < 0.0f);
        if (!(at_hi || at_lo)) {
            integral_ += error * dt;
        }
    }

    // Atualiza estados para próximo passo
    prev_v_ = v_current;
    prev_error_ = error;

    return u_sat;
}
