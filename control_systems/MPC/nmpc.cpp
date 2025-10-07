#include "nmpc.hpp"
#include <iostream>
#include <cmath>
#include <limits> 

using namespace casadi;

NMPCController::NMPCController(double L, double dt, int N_pred, double max_delta_rad,
                               double Q_offset, double Q_psi, double R_delta_rate)
    : L(L), dt(dt), N_pred(N_pred),
      Q_offset(Q_offset), Q_psi(Q_psi), R_delta_rate(R_delta_rate),
      max_delta_rad(max_delta_rad),
      prev_delta(0.0), prev_x(0.0), prev_y(0.0), prev_theta(0.0) {
    setupCasADiProblem();
}

void NMPCController::setupCasADiProblem() {
    SX x = SX::sym("x");         // x position
    SX y = SX::sym("y");         // y position
    SX theta = SX::sym("theta"); // orientation (yaw)
    SX delta = SX::sym("delta"); // steering angle
    SX v = SX::sym("v");         // linear velocity

    // State vector [x, y, theta]
    SX states = SX::vertcat({x, y, theta});
    // Control vector [delta]
    SX controls = SX::vertcat({delta});

    // Optimization parameters
    // Initial state and current speed are updated every iteration
    SX initial_state = SX::sym("initial_state", 3); // [x0, y0, theta0]
    SX current_velocity = SX::sym("current_velocity"); // v_actual

    // Discrete-time system model
    // Predicts next state (x_next, y_next, theta_next)
    SX rhs = SX::vertcat({
        x + current_velocity * cos(theta) * dt,
        y + current_velocity * sin(theta) * dt,
        theta + (current_velocity / L) * tan(delta) * dt
    });

    // Create CasADi function for the model (f: {states, controls, v} -> {next_states})
    Function f("f", {states, controls, current_velocity}, {rhs});

    // ------ Optimization problem ------
    // Decision variables:
    // State trajectory over horizon (N_pred+1 states)
    // Control sequence (N_pred controls)
    SX X = SX::sym("X", 3, N_pred + 1); // X = [x0, x1, ..., xN_pred]
                                       //     [y0, y1, ..., yN_pred]
                                       //     [theta0, theta1, ..., thetaN_pred]
    SX U = SX::sym("U", 1, N_pred);     // U = [delta0, delta1, ..., deltaN_pred-1]

    // Packed parameters P
    SX P = SX::vertcat({initial_state, current_velocity});

    // Initial constraint: first predicted state equals initial_state
    SX g = SX::vertcat({X(Slice(), 0) - initial_state}); 

    // Cost function (J)
    SX obj = 0;

    // Loop over prediction horizon
    for (int k = 0; k < N_pred; ++k) {
        SX state_k = X(Slice(), k);
        SX control_k = U(Slice(), k);

        SX next_state_k = f(SXVector{state_k, control_k, current_velocity})[0];

        // Equality constraint: predicted next state must match X(:,k+1)
        g = SX::vertcat({g, next_state_k - X(Slice(), k+1)});

        // Cost terms
        // Penalize lateral offset (y) and heading error (theta)
        // Target is y=0 and theta=0 (aligned with X axis / lane centerline)
        obj += Q_offset * pow(X(1, k), 2); // lateral offset (y_k)
        obj += Q_psi * pow(X(2, k), 2);    // heading error (theta_k)

        // Penalize steering rate
        if (k > 0) {
            obj += R_delta_rate * pow(control_k(0) - U(0, k-1), 2);
        }
    }

    SX prev_delta_param = SX::sym("prev_delta_param");
    P = SX::vertcat({P, prev_delta_param});

    // Ajustar a penalidade do primeiro delta
    obj += R_delta_rate * pow(U(0, 0) - prev_delta_param, 2);


    // Bounds on control (delta)
    // Use DM::ones() and scale to fill bounds
    DM lbu = DM::ones(1, N_pred) * (-max_delta_rad); // filled with -max_delta_rad
    DM ubu = DM::ones(1, N_pred) * max_delta_rad;    // filled with +max_delta_rad

    // Bounds on states (x, y, theta) - use +/- infinity
    DM lbx_states = DM::ones(3, N_pred + 1) * (-DM::inf()); // lower bounds for states
    DM ubx_states = DM::ones(3, N_pred + 1) * DM::inf();    // upper bounds for states

    // Equality constraints g = 0
    DM lbg = DM::zeros(g.size1(), g.size2()); // lower bound for g (zeros)
    DM ubg = DM::zeros(g.size1(), g.size2()); // upper bound for g (zeros)

    // Create NLP problem
    Dict opts;
    opts["print_time"] = 0;
    opts["ipopt.print_level"] = 0;
    opts["ipopt.print_timing_statistics"] = "no";
    opts["ipopt.max_iter"] = 100;
    opts["ipopt.warm_start_init_point"] = "yes";

    // Build NLP dict
    casadi::SXDict nlp = {{"x", SX::vertcat({SX::reshape(X, -1, 1), SX::reshape(U, -1, 1)})},
                          {"p", P},
                          {"f", obj},
                          {"g", g}};

    solver = casadi::nlpsol("solver", "ipopt", nlp, opts);

    // Initialize solver arguments
    arg["lbg"] = lbg;
    arg["ubg"] = ubg;
    // Stack state and control bounds
    arg["lbx"] = DM::vertcat({SX::reshape(lbx_states, -1, 1), SX::reshape(lbu, -1, 1)});
    arg["ubx"] = DM::vertcat({SX::reshape(ubx_states, -1, 1), SX::reshape(ubu, -1, 1)});

    // Initialize warm start for decision variables
    arg["x0"] = DM::zeros(X.size1() * X.size2() + U.size1() * U.size2());
}

// Parameter 'current_theta_rad' was removed from the function signature
double NMPCController::computeControl(double offset_m, double psi_rad, double current_velocity_mps) {
    DM initial_state_val = DM({prev_x, offset_m, psi_rad});
    DM p_val = DM::vertcat({initial_state_val, DM(current_velocity_mps), DM(prev_delta)});
    arg["p"] = p_val;

    // Defina o ponto inicial para as variáveis de otimização (warm start)
    if (!res.empty()) {
        arg["x0"] = res.at("x");
    }

    // Solve the optimization problem
    res = solver(arg);

    // Extract first control (delta)
    DM opt_controls_flat = res.at("x")(Slice(3 * (N_pred + 1), 3 * (N_pred + 1) + N_pred), 0);
    double optimal_delta = static_cast<double>(opt_controls_flat(0));

    prev_delta = optimal_delta;
    DM optimal_states_flat = res.at("x")(Slice(0, 3 * (N_pred + 1)), 0);
    prev_x = static_cast<double>(optimal_states_flat(3));

    return optimal_delta;
}