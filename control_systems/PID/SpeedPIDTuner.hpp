// SpeedPIDTuner.hpp**
#pragma once

#include <tuple>
#include "SpeedPIDTuner.hpp"
#include "SpeedPIDController.hpp"
#include "../BackMotors/BackMotors.hpp"
#include <algorithm>
#include <chrono>
#include <thread>
#include <limits>
#include <iostream>
#include "SpeedPIDController.hpp"
#include <cmath>
#include <limits>
#include <iostream>
#include <atomic>
#include <vector>



static float simulate_velocity(float v_current, float pwm_input, float dt);
static float evaluate_pid(float kp, float ki, float kd, float dt, float sim_time, float v_target);
std::tuple<float, float, float> auto_tune_pid(float dt = 0.1f, float sim_time = 10.0f, float v_target = 2.0f, bool real = false);
void mainAutoTunePID();

class BackMotors;

// Grid-search em “real” lendo ZMQ e aplicando PWM
std::tuple<float,float,float> auto_tune_pid_real(BackMotors& backMotors,
                                                 std::atomic<double>& current_speed_ms,
                                                 float dt, float sim_time, float v_target,
                                                 float pwm_max_percent = 40.0f);
