#ifndef SPEED_PID_CONTROLLER_HPP
#define SPEED_PID_CONTROLLER_HPP

#include <cstdint>

/*
  SpeedPIDController (forma paralela)
  u = Kp*e + Ki*∫e dt + Kd * de/dt

  Melhorias:
  - Anti-windup (back-calculation): integra erro + kb*(u_sat - u_raw)
  - Derivada na medição com filtro de 1ª ordem (tempo de filtro Tf)
  - Compatível com o construtor antigo (5 argumentos) via valores por defeito

  Notas:
  - pwm_min/pwm_max na mesma unidade da saída (ex.: %), tipicamente [0, 100]
  - Ki e Kd em forma paralela (Ki multiplica a integral; Kd multiplica a derivada)
  - Tf (s) define quão “suavizada” é a derivada (maior Tf ⇒ mais filtrada)
  - kb ≈ 1/Tt (Tt: constante do anti-windup); ajustar conforme saturação do atuador
*/

class SpeedPIDController {
public:
    // Construtor (mantém compatibilidade com chamada de 5 parâmetros).
    // use_backcalc, kb e Tf têm defaults; chamadas antigas continuam a compilar.
    SpeedPIDController(float kp, float ki, float kd,
                       float pwm_min, float pwm_max,
                       bool use_backcalc = true,
                       float kb = 1.0f,
                       float Tf = 0.02f);

    // Reinicia estados internos (integral, derivada filtrada, etc.)
    void reset();

    // Atualiza saída do controlador dado estado atual e dt (segundos).
    // v_current: velocidade atual (m/s)
    // v_target : referência de velocidade (m/s)
    // dt       : período de amostragem (s)
    // Retorna: comando saturado entre [pwm_min, pwm_max] (mesma unidade do PWM de alto nível, ex.: %)
    float update(float v_current, float v_target, float dt);

    // Setters opcionais (se desejar ajustar após construção)
    void setGains(float kp, float ki, float kd) { kp_ = kp; ki_ = ki; kd_ = kd; }
    void setOutputLimits(float pwm_min, float pwm_max) { pwm_min_ = pwm_min; pwm_max_ = pwm_max; }
    void setAntiWindup(bool enable, float kb) { use_backcalc_ = enable; kb_ = kb; }
    void setDerivativeFilter(float Tf) { Tf_ = (Tf > 0.0f ? Tf : 0.0f); }

private:
    // Ganhos (forma paralela)
    float kp_, ki_, kd_;

    // Limites de saída (mesma unidade do comando externo, ex.: %)
    float pwm_min_, pwm_max_;

    // Anti-windup (back-calculation): kb ≈ 1/Tt
    float kb_;

    // Filtro da derivada (tempo de filtro em segundos)
    float Tf_;

    // Estados internos
    float prev_v_;       // última medição
    float prev_error_;   // último erro
    float integral_;     // estado do integrador
    float d_filt_;       // derivada filtrada da medição
    bool  use_backcalc_;
    bool  initialized_;
};

#endif // SPEED_PID_CONTROLLER_HPP
