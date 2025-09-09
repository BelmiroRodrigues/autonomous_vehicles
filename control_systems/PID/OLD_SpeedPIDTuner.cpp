// SpeedPIDTuner.cpp**
#include "SpeedPIDTuner.hpp"


float speed_from_zermq = 0.0f; // Placeholder for the speed from ZMQ service


static float real_velocity(float v_current, float pwm_input, float dt) {
	// Fetch the real speed from the vehicle's sensors
	// This is a placeholder function and should be replaced with actual sensor reading
	// For example, you can use a function like get_speed_from_sensor() to get the real speed
	// This function should return the current speed of the vehicle
	// For example, if you have a speed sensor, you can read the speed from it
	// and return it as a float value
	// In this example, we will just return the current speed from the zero mq service
	return (speed_from_zermq);
}

static float simulate_velocity(float v_current, float pwm_input, float dt) {
	// Simulate the velocity based on PWM input and current velocity
	// This is a simple model and can be adjusted for more accuracy
	// Damping factor and max acceleration can be tuned
	// For example, damping = 0.1f, max_accel = 3.0f
	// These values can be adjusted based on the vehicle's characteristics
	// and the desired simulation accuracy
	// The damping factor simulates the resistance to acceleration
	// The max_accel simulates the maximum acceleration based on PWM input
	// The formula used is a simple Euler integration step
	// where the new velocity is calculated based on the current velocity,
	// the acceleration (based on PWM input), and the time step (dt)
	// The acceleration is calculated as:
	// accel = pwm_input * max_accel / 100.0f - damping * v_current
	// This means that the acceleration is proportional to the PWM input
	// and inversely proportional to the current velocity, simulating a damping effect
	float damping = 0.1f;
    float max_accel = 3.0f;
    float accel = pwm_input * max_accel / 100.0f - damping * v_current;
    return v_current + accel * dt;
}

static float get_velocity(float v_current, float pwm_input, float dt, bool real = true) {
    return (real ? real_velocity(v_current, pwm_input, dt) : simulate_velocity(v_current, pwm_input, dt));
}

static float evaluate_pid(float kp, float ki, float kd, float dt, float sim_time, float v_target, bool real) {
    std::cout << real << std::endl;
    SpeedPIDController pid(kp, ki, kd, 0.0f, 100.0f);
    float v = 0.0f;
    float total_error = 0.0f;
    float max_overshoot = 0.0f;
    float final_error = 0.0f;

    for (float t = 0.0f; t <= sim_time; t += dt) {
        float pwm = pid.update(v, v_target, dt);
		v = get_velocity(v, pwm, dt, real);
        float error = std::abs(v_target - v);
        total_error += error * dt;

        if (v > v_target) {
            max_overshoot = std::max(max_overshoot, v - v_target);
        }

        if (t >= sim_time - 1.0f) {
            final_error += error * dt;
        }
    }

    return total_error + 10.0f * max_overshoot + 20.0f * final_error;
}

std::tuple<float, float, float> auto_tune_pid(float dt, float sim_time, float v_target, bool real) {
    float best_score = std::numeric_limits<float>::max();
    float best_kp = 0, best_ki = 0, best_kd = 0;

    for (float kp = 0.1f; kp <= 1.0f; kp += 0.1f) {
        for (float ki = 0.0f; ki <= 0.2f; ki += 0.02f) {
            for (float kd = 0.0f; kd <= 0.2f; kd += 0.02f) {
                float score = evaluate_pid(kp, ki, kd, dt, sim_time, v_target, real);
                if (score < best_score) {
                    best_score = score;
                    best_kp = kp;
                    best_ki = ki;
                    best_kd = kd;
                }
            }
        }
    }

    return {best_kp, best_ki, best_kd};
}



static inline void sleep_dt(float dt_s){
    std::this_thread::sleep_for(std::chrono::duration<float>(dt_s));
}

/* // Ensaiar 1 tripla (Kp,Ki,Kd) em “real”: aplica PWM, espera dt, lê current_speed_ms
static float evaluate_pid_real(BackMotors& backMotors,
                               std::atomic<double>& current_speed_ms,
                               float kp, float ki, float kd,
                               float dt, float sim_time, float v_target,
                               float pwm_max_percent)
{
    SpeedPIDController pid(kp, ki, kd, 0.0f, pwm_max_percent);
    backMotors.setSpeed(0);

    // Warm-up opcional (vencer deadzone) — 0.4 s a ~20% PWM antes de medir custo
    int warm_pwm = int(std::round(0.20f * 255.0f));
    backMotors.setSpeed(warm_pwm);
    sleep_dt(0.4f);

    float v = float(current_speed_ms.load());
    float total_error = 0.0f, max_overshoot = 0.0f, final_error = 0.0f;

    const float final_window = 1.0f;
    int steps = std::max(1, int(sim_time / dt));
    int steps_final = std::max(1, int(final_window / dt));

    // Early-stop thresholds
    const float dv_min = 0.05f;        // variação mínima de velocidade
    const int early_check_steps = std::max(1, int(1.0f / dt));
    float v0 = v;

    for (int k = 0; k < steps; ++k) {
        float pwm_percent = pid.update(v, v_target, dt);
        int pwm_hw = int(std::round(std::clamp(pwm_percent, 0.0f, pwm_max_percent) / 100.0f * 255.0f));
        pwm_hw = std::clamp(pwm_hw, 0, 255);
        backMotors.setSpeed(pwm_hw);

        sleep_dt(dt);
        v = float(current_speed_ms.load());

        // Early-stop: sem mudança de velocidade suficiente após ~1 s ou saturação prolongada
        if (k == early_check_steps) {
            if (std::fabs(v - v0) < dv_min) { backMotors.setSpeed(0); return 1e9f; }
        }

        float err = std::fabs(v_target - v);
        total_error += err * dt;
        if (v > v_target) max_overshoot = std::max(max_overshoot, v - v_target);
        if (k >= steps - steps_final) final_error += err * dt;
    }

    backMotors.setSpeed(0);
    // Custo composto: seguimento + overshoot + erro final médio
    return total_error + 10.0f * max_overshoot + 20.0f * (final_error / std::max(final_window, dt));
}

 */

/*  // Grid “grossa→fina” para encontrar (Kp,Ki,Kd) em hardware
std::tuple<float,float,float> auto_tune_pid_real(BackMotors& backMotors,
                                                 std::atomic<double>& current_speed_ms,
                                                 float dt, float sim_time, float v_target,
                                                 float pwm_max_percent)
{
    float best_score = std::numeric_limits<float>::infinity();
    float best_kp = 0.f, best_ki = 0.f, best_kd = 0.f;

    // Fase grossa (triagem rápida)
    std::vector<float> Kp_grid = {4.f, 8.f, 10.f, 12.f, 14.f};
    std::vector<float> Ki_grid = {0.00f, 0.20f, 0.50f, 1.00f};
    std::vector<float> Kd_grid = {0.00f, 0.10f, 0.30f, 0.60f};

    for (float kp : Kp_grid)
      for (float ki : Ki_grid)
        for (float kd : Kd_grid) {
            float score = evaluate_pid_real(backMotors, current_speed_ms,
                                            kp, ki, kd, dt, 3.0f,
                                            v_target, pwm_max_percent);
            if (score < best_score) { best_score=score; best_kp=kp; best_ki=ki; best_kd=kd; }
        }

    // Se Kp ficou numa borda, expandir e repetir triagem
    if (best_kp <= Kp_grid.front() || best_kp >= Kp_grid.back()) {
        std::vector<float> Kp2 = {2.f, 5.f, 10.f, 20.f, 40.f, 60.f, 80.f};
        for (float kp : Kp2)
          for (float ki : Ki_grid)
            for (float kd : Kd_grid) {
                float score = evaluate_pid_real(backMotors, current_speed_ms,
                                                kp, ki, kd, dt, 3.0f,
                                                v_target, pwm_max_percent);
                if (score < best_score) { best_score=score; best_kp=kp; best_ki=ki; best_kd=kd; }
            }
    }

    // Fase fina (refino local)
    std::vector<float> Kp_ref = {std::max(1.f, best_kp*0.85f), best_kp, best_kp*1.15f};
    std::vector<float> Ki_ref = {std::max(0.f, best_ki-0.05f), best_ki, best_ki+0.05f};
    std::vector<float> Kd_ref = {std::max(0.f, best_kd-0.05f), best_kd, best_kd+0.05f};

    for (float kp : Kp_ref)
      for (float ki : Ki_ref)
        for (float kd : Kd_ref) {
            float score = evaluate_pid_real(backMotors, current_speed_ms,
                                            kp, ki, kd, dt, 5.0f,
                                            v_target, pwm_max_percent);
            if (score < best_score) { best_score=score; best_kp=kp; best_ki=ki; best_kd=kd; }
        }

    backMotors.setSpeed(0);
    return {best_kp, best_ki, best_kd};
}

 */

// Avaliação em “real”: aplica PWM, espera dt, lê velocidade via ZMQ
static float evaluate_pid_real(BackMotors& backMotors,
                               std::atomic<double>& current_speed_ms,
                               float kp, float ki, float kd,
                               float dt, float sim_time, float v_target,
                               float pwm_max_percent)
{
    SpeedPIDController pid(kp, ki, kd, 0.0f, pwm_max_percent);

    backMotors.setSpeed(0);

    float v = static_cast<float>(current_speed_ms.load());
    float total_error = 0.0f, max_overshoot = 0.0f, final_error = 0.0f;

    const float final_window = 1.0f;
    int steps = std::max(1, static_cast<int>(sim_time / dt));
    int steps_final = std::max(1, static_cast<int>(final_window / dt));

    for (int k = 0; k < steps; ++k) {
        float pwm_percent = pid.update(v, v_target, dt);
        int pwm_hw = static_cast<int>(std::round(std::clamp(pwm_percent, 0.0f, pwm_max_percent) / 100.0f * 255.0f));
        pwm_hw = std::clamp(pwm_hw, 0, 255);
        backMotors.setSpeed(pwm_hw);                 // aplicar PWM no hardware [2]

        sleep_dt(dt);                                // esperar período de amostragem [2]
        v = static_cast<float>(current_speed_ms.load()); // ler velocidade “real” do ZMQ [1]

        float err = std::fabs(v_target - v);
        total_error += err * dt;
        if (v > v_target) max_overshoot = std::max(max_overshoot, v - v_target);
        if (k >= steps - steps_final) final_error += err * dt;
    }

    backMotors.setSpeed(0);
    return total_error + 10.0f * max_overshoot + 20.0f * (final_error / std::max(final_window, dt));
}

std::tuple<float,float,float> auto_tune_pid_real(BackMotors& backMotors,
                                                 std::atomic<double>& current_speed_ms,
                                                 float dt, float sim_time, float v_target,
                                                 float pwm_max_percent)
{
    float best_score = std::numeric_limits<float>::infinity();
    float best_kp = 0.0f, best_ki = 0.0f, best_kd = 0.0f;

    for (float kp = 4.0f; kp <= 16.0f; kp += 2.0f) {
        for (float ki = 0.0f; ki <= 10.0f; ki += 1.0f) {
            for (float kd = 0.0f; kd <= 5.0f; kd += 0.5f) {
                std::cout << "Testing || Kp= " << kp << "|| Ki= " << ki << "|| Kd= " << kd << std::endl;
                float score = evaluate_pid_real(backMotors, current_speed_ms,
                                                kp, ki, kd, dt, sim_time, v_target,
                                                pwm_max_percent);
                if (score < best_score) {
                    best_score = score; best_kp = kp; best_ki = ki; best_kd = kd;
                }
            }
        }
    }
    backMotors.setSpeed(0);
    return {best_kp, best_ki, best_kd};
}


 const float PWM_MIN = 0.0f; // Minimum PWM value
const float PWM_MAX = 100.0f; // Maximum PWM value

void mainAutoTunePID() {
	float dt = 0.03f;//Time step
	float sim_time = 10.0f; // Simulation time
	float v_target = 2.0f; // Target speed
	float v_current = 0.0f; // Initial speed
	float pwm_input = PWM_MIN; // Initial PWM input

	auto [kp, ki, kd] = auto_tune_pid(dt, sim_time, v_target);;

    std::cout << "Best PID gains found:\n";
    std::cout << "Kp = " << kp << "\n";
    std::cout << "Ki = " << ki << "\n";
    std::cout << "Kd = " << kd << "\n";
	std::cout << "Simulating with tuned PID controller...\n";
	SpeedPIDController traction(kp, ki, kd, PWM_MIN, PWM_MAX);
	traction.reset();
	v_current = 0.0f;//go fetch current speed from ZMQ
	v_target =0.0f;//test
	traction.update(v_current, v_target, dt);
	std::cout << "Current speed: " << v_current << "\n";
}