#ifndef NMPC_HPP
#define NMPC_HPP

#include <vector>
#include <map>
#include <string>
#include <casadi/casadi.hpp>

class NMPCController {
public:
    NMPCController(
        double L = 0.15,        // Wheelbase (m)
        double dt = 0.1,        // Time step (s)
        int N_pred = 10,        // Prediction horizon
        double max_delta_rad = 40.0 * M_PI / 180.0, // Max steering angle (rad)
        double Q_offset = 20000.0, // Peso para erro lateral
        double Q_psi = 6000.0,   // Peso para erro de orientação
        double R_delta_rate = 80.0 // Peso para taxa de mudança de delta
    );

    // Compute steering command (delta).
    // offset_m: lateral error in meters.
    // psi_rad: heading error in radians.
    // current_velocity_mps: current vehicle speed in m/s.
    // Note: parameter 'current_theta_rad' was removed as unused by the MPC logic.
    double computeControl(double offset_m, double psi_rad, double current_velocity_mps);

private:
    double L;      // Wheelbase
    double dt;     // Time step
    int N_pred;    // Prediction horizon

    // Cost function weights (ordered to match constructor)
    double Q_offset;
    double Q_psi;
    double R_delta_rate;

    double max_delta_rad; // Max steering angle in radians

    casadi::Function solver;                       // CasADi NLP solver
    std::map<std::string, casadi::DM> arg;        // Solver arguments
    std::map<std::string, casadi::DM> res;        // Solver results

    double prev_delta;                 // Previous steering command to penalize rate of change
    double prev_x, prev_y, prev_theta; // Initial state used for warm start (prev_x is most relevant)

    // Initialize the CasADi optimization problem
    void setupCasADiProblem();
};

#endif // NMPC_HPP
