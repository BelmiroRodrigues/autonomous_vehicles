// SpeedPIDTuner.cpp**
#include "SpeedPIDTuner.hpp"


float speed_from_zermq = 0.0f; // Placeholder for the speed from ZMQ service


static float real_velocity(float v_current, float pwm_input, float dt) {
	return (speed_from_zermq);
}

static float simulate_velocity(float v_current, float pwm_input, float dt) {
	float damping = 0.1f;
    float max_accel = 3.0f;
    float accel = pwm_input * max_accel / 100.0f - damping * v_current;
    return v_current + accel * dt;
}

static float get_velocity(float v_current, float pwm_input, float dt, bool real = true) {
    return (real ? real_velocity(v_current, pwm_input, dt) : simulate_velocity(v_current, pwm_input, dt));
}



static inline void sleep_dt(float dt_s){
    std::this_thread::sleep_for(std::chrono::duration<float>(dt_s));
}

struct PIDGains { float kp, ki, kd; };

struct SafetyOptions {
    float v_max_ms = 5.0f;           // limite de segurança de velocidade
    float overshoot_max_ms = 1.0f;   // overshoot máximo permitido absoluto
    float sat_time_max_s = 0.5f;     // tempo máximo em saturação acumulado
    float deadband_ms = 0.02f;       // banda morta para detetar cruzamentos
};

struct CostWeights {
    float w_itae = 1.0f;      // peso do ITAE
    float w_overshoot = 10.0f;// peso da penalização de overshoot
    float w_effort = 0.5f;    // peso do esforço ∫u^2 dt (u normalizado)
    float w_sat = 5.0f;       // peso do tempo em saturação
    float w_osc = 2.0f;       // peso do indicador de oscilação
};

struct EvaluateResult {
    float score;
    float itae;
    float max_overshoot;
    float effort_u2;
    float t_sat;
    int oscillation_crossings;
};

// Função de avaliação com ITAE + salvaguardas
static EvaluateResult evaluate_pid_real_itae(BackMotors& backMotors,
                                             std::atomic<double>& current_speed_ms,
                                             const PIDGains& g,
                                             float dt, float sim_time, float v_target,
                                             float pwm_max_percent,
                                             const SafetyOptions& safe,
                                             const CostWeights& W)
{
    SpeedPIDController pid(g.kp, g.ki, g.kd, 0.0f, pwm_max_percent); // assumir reset interno
    backMotors.setSpeed(0);

    float v = static_cast<float>(current_speed_ms.load());
    float itae = 0.0f, max_overshoot = 0.0f, effort_u2 = 0.0f, t_sat = 0.0f;
    int crossings = 0;
    float prev_e = v_target - v;
    bool prev_e_valid = false;

    const int steps = std::max(1, static_cast<int>(sim_time / dt));
    for (int k = 0; k < steps; ++k) {
        float pwm_percent = pid.update(v, v_target, dt);
        pwm_percent = std::clamp(pwm_percent, 0.0f, pwm_max_percent);

        // saturação (considera 2% como margem para “colado” no limite)
        bool sat_hi = (pwm_percent >= pwm_max_percent * 0.98f);
        bool sat_lo = (pwm_percent <= pwm_max_percent * 0.02f);
        if (sat_hi || sat_lo) t_sat += dt;

        // esforço normalizado 0..1
        float u_norm = (pwm_max_percent > 0.0f ? pwm_percent / pwm_max_percent : 0.0f);
        effort_u2 += (u_norm * u_norm) * dt;

        int pwm_hw = static_cast<int>(std::round(pwm_percent / 100.0f * 255.0f));
        pwm_hw = std::clamp(pwm_hw, 0, 255);
        backMotors.setSpeed(pwm_hw);

        sleep_dt(dt);
        v = static_cast<float>(current_speed_ms.load());

        float e = v_target - v;
        float t = (k + 1) * dt;
        itae += t * std::fabs(e);

        if (v > v_target) max_overshoot = std::max(max_overshoot, v - v_target);

        // contagem de cruzamentos fora da deadband
        if (prev_e_valid) {
            if ((prev_e > safe.deadband_ms && e < -safe.deadband_ms) ||
                (prev_e < -safe.deadband_ms && e > safe.deadband_ms)) {
                crossings++;
            }
        }
        prev_e = e; prev_e_valid = true;

        // abortos de segurança
        if (v > safe.v_max_ms || max_overshoot > safe.overshoot_max_ms || t_sat > safe.sat_time_max_s) {
            // penalização pesada e aborta cedo
            itae *= 2.0f;
            max_overshoot *= 2.0f;
            t_sat += (sim_time - t);
            break;
        }
    }

    backMotors.setSpeed(0);

    // custo composto
    float score = W.w_itae * itae
                + W.w_overshoot * max_overshoot
                + W.w_effort * effort_u2
                + W.w_sat * t_sat
                + W.w_osc * static_cast<float>(crossings);

    return {score, itae, max_overshoot, effort_u2, t_sat, crossings};
}

// Ensaio de relé com histerese para estimar Ku e Pu; retorna ganhos ZN paralelos
struct RelayOptions {
    float pwm_lo_percent = 15.0f;     // nível baixo do relé (% do máximo)
    float pwm_hi_percent = 60.0f;     // nível alto do relé (% do máximo)
    float hysteresis_ms = 0.05f;      // banda em m/s ao redor do setpoint
    int   min_cycles = 4;             // medir pelo menos 4 ciclos
    float settle_time_s = 1.0f;       // tempo antes de iniciar medição
    float max_test_time_s = 6.0f;     // tempo máximo do teste
};

static PIDGains relay_seed_zn(BackMotors& backMotors,
                              std::atomic<double>& current_speed_ms,
                              float dt, float v_target, float pwm_max_percent,
                              const RelayOptions& R,
                              const SafetyOptions& safe)
{
    backMotors.setSpeed(0);
    std::cout << "Realay seed " << std::endl;

    // configurações
    const float pwm_lo = std::clamp(R.pwm_lo_percent, 0.0f, pwm_max_percent);
    const float pwm_hi = std::clamp(R.pwm_hi_percent, 0.0f, pwm_max_percent);
    float pwm_cmd = pwm_hi; // começa "acima" para subir até a banda superior

    // variáveis de medição
    float v = static_cast<float>(current_speed_ms.load());
    float t = 0.0f;
    int cycles = 0;
    std::vector<float> crossing_times; crossing_times.reserve(16);
    float v_max = -1e9f, v_min = 1e9f;

    auto set_pwm = [&](float pwm_percent){
        pwm_percent = std::clamp(pwm_percent, 0.0f, pwm_max_percent);
        int pwm_hw = static_cast<int>(std::round(pwm_percent / 100.0f * 255.0f));
        
        backMotors.setSpeed(std::clamp(pwm_hw, 0, 255));
    };

    // fase de assentamento
    for (int k = 0; t < R.settle_time_s; ++k) {
        set_pwm(pwm_cmd);
        sleep_dt(dt); t += dt;
        v = static_cast<float>(current_speed_ms.load());
    }

    float last_cross_up = -1.0f;
    float last_cross_down = -1.0f;

    // laço de relé
    for (int k = 0; t < R.max_test_time_s; ++k) {
        // histerese: alterna u entre níveis quando cruza as bandas
        if (pwm_cmd == pwm_hi && v >= v_target + R.hysteresis_ms) {
            pwm_cmd = pwm_lo;
            crossing_times.push_back(t);
            last_cross_down = t;
            cycles++;
            v_max = std::max(v_max, v);
        } else if (pwm_cmd == pwm_lo && v <= v_target - R.hysteresis_ms) {
            pwm_cmd = pwm_hi;
            crossing_times.push_back(t);
            last_cross_up = t;
            v_min = std::min(v_min, v);
        }

        set_pwm(pwm_cmd);
        sleep_dt(dt); t += dt;
        v = static_cast<float>(current_speed_ms.load());

        // abortos
        if (v > safe.v_max_ms) break;
    }

    backMotors.setSpeed(0);

    // checagem de ciclos
    if (crossing_times.size() < 2 || cycles < R.min_cycles) {
        // fallback: ganhos pequenos e estáveis
        return PIDGains{4.0f, 0.5f, 0.2f};
    }

    // período médio Pu a partir de alternâncias (duas alternâncias ~ meia-período)
    std::vector<float> halfPeriods;
    for (size_t i = 2; i < crossing_times.size(); ++i) {
        float hp = crossing_times[i] - crossing_times[i-2];
        std::cout << "Half Periods " << hp << std::endl;

        if (hp > 0) halfPeriods.push_back(hp);
    }
    float Pu = 0.0f;
    for (float hp : halfPeriods) Pu += (2.0f * hp);
    Pu = (halfPeriods.empty() ? 1.0f : Pu / halfPeriods.size());

    // amplitude a (meia amplitude de v em torno do setpoint) a partir de picos e vales
    float a = 0.5f * std::max(0.0f, (v_max - v_min));

    // amplitude do relé Δ em % (meia distância entre níveis)
    float Delta = 0.5f * std::fabs(pwm_hi - pwm_lo);

    // estimativa Ku (modelo clássico do relé sem atraso explícito)
    // Ku ≈ (4Δ)/(π a) — Δ em %, a em m/s; unidade relativa serve para seed
    float Ku = (a > 1e-3f) ? (4.0f * Delta) / (3.1415926f * a) : 1.0f;

    // Ziegler–Nichols PID (forma paralela): Kp=0.6Ku, Ti=Pu/2, Td=Pu/8
    float Kp = 0.6f * Ku;
    float Ti = std::max(1e-3f, Pu / 2.0f);
    float Td = Pu / 8.0f;
    float Ki = Kp / Ti;         // paralelo
    float Kd = Kp * Td;         // paralelo

    // clamps básicos
    Kp = std::clamp(Kp, 0.1f, 30.0f);
    Ki = std::clamp(Ki, 0.0f, 30.0f);
    Kd = std::clamp(Kd, 0.0f, 15.0f);

    return PIDGains{Kp, Ki, Kd};
}

// Otimizador local simples (pattern search) com limites e passos decrescentes
struct Bounds { float kp_min, kp_max, ki_min, ki_max, kd_min, kd_max; };

static PIDGains local_optimize_ITAE(BackMotors& backMotors,
                                    std::atomic<double>& current_speed_ms,
                                    PIDGains seed,
                                    float dt, float sim_time, float v_target,
                                    float pwm_max_percent,
                                    const SafetyOptions& safe,
                                    const CostWeights& W,
                                    const Bounds& B,
                                    int max_iters = 20)
{
    auto eval = [&](const PIDGains& g){
        return evaluate_pid_real_itae(backMotors, current_speed_ms, g,
                                      dt, sim_time, v_target, pwm_max_percent,
                                      safe, W).score;
    };

    std::cout << "Local optimize ITAE" << std::endl;


    PIDGains best = seed;
    float bestScore = eval(best);

    float step_kp = std::max(0.1f, 0.2f * std::max(1.0f, best.kp));
    float step_ki = std::max(0.05f, 0.2f * std::max(1.0f, best.ki));
    float step_kd = std::max(0.05f, 0.2f * std::max(1.0f, best.kd));

    for (int it = 0; it < max_iters; ++it) {
        bool improved = false;
        const PIDGains dirs[10] = {
            {+step_kp, 0, 0}, {-step_kp, 0, 0},
            {0, +step_ki, 0}, {0, -step_ki, 0},
            {0, 0, +step_kd}, {0, 0, -step_kd}
        };
        for (const auto& d : dirs) {
            PIDGains g{
                std::clamp(best.kp + d.kp, B.kp_min, B.kp_max),
                std::clamp(best.ki + d.ki, B.ki_min, B.ki_max),
                std::clamp(best.kd + d.kd, B.kd_min, B.kd_max)
            };
            float s = eval(g);
            std::cout << " EVAL: " << s << std::endl;

            if (s < bestScore) {
                best = g; bestScore = s; improved = true;
            }
        }
        if (!improved) {
            step_kp *= 0.5f; step_ki *= 0.5f; step_kd *= 0.5f;
            if (std::max({step_kp, step_ki, step_kd}) < 0.01f) break;
        }
    }
    return best;
}

// Orquestração completa do autotune
std::tuple<float,float,float> autotune_speed_two_stage(BackMotors& backMotors,
                                                       std::atomic<double>& current_speed_ms,
                                                       float dt, float sim_time, float v_target,
                                                       float pwm_max_percent)
{
    SafetyOptions safe;
    CostWeights   W;
    std::cout << "Autotune spees two stage" << std::endl;

    // 1) Semente com relé
    RelayOptions R;
    PIDGains seed = relay_seed_zn(backMotors, current_speed_ms, dt, v_target, pwm_max_percent, R, safe);

    // 2) Refinamento local
    Bounds B{0.1f, 30.0f, 0.0f, 30.0f, 0.0f, 15.0f};
    PIDGains best = local_optimize_ITAE(backMotors, current_speed_ms, seed,
                                        dt, sim_time, v_target, pwm_max_percent,
                                        safe, W, B, /*max_iters*/ 18);

    backMotors.setSpeed(0);
    return {best.kp, best.ki, best.kd};
}


 const float PWM_MIN = 0.0f; // Minimum PWM value
const float PWM_MAX = 100.0f; // Maximum PWM value
