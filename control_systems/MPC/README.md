# MPC (Model Predictive Control) — Lateral Control Specification

This document specifies the lateral Model Predictive Control (MPC, Model Predictive Control) module for lane keeping on Jetson Nano, covering scope, interfaces, plant model, discretization, horizons, objective, constraints, solver configuration, warm‑start strategy, real‑time (RT, real‑time) behavior, tuning rationale, safety, verification and validation (V&V, Verification and Validation), and studied future work not yet implemented.

## Purpose and scope
The controller minimizes lateral offset and yaw error subject to discrete kinematic vehicle dynamics, issuing a steering command that respects actuator limits for lane‑center tracking in typical on‑road conditions. Lane‑center following is used as the immediate reference, i.e., the target is \(y=0\) and \(\psi=0\) at each prediction step rather than consuming an external preview path at this stage.

## Interfaces
Inputs per control cycle are lateral offset \(y\) [m], yaw error \(\psi\) [rad], and the measured longitudinal speed \(v\) [m/s] passed each iteration to the controller; the output is the steering angle \(\delta\) [rad], later converted to degrees and saturated to ±30° in the actuator layer.

## Kinematic bicycle model
A discrete kinematic bicycle model is used with states \(x, y, \psi\) and control \(\delta\), while the measured speed \(v\) is treated as a constant parameter across the prediction horizon within each iteration to reflect current operating conditions with reduced decision dimension. The component‑wise state update is:

$$x_{k+1} = x_k + v_k \cos(\psi_k) \cdot \Delta t$$
$$y_{k+1} = y_k + v_k \sin(\psi_k) \cdot \Delta t$$
$$\psi_{k+1} = \psi_k + \frac{v_k}{L} \tan(\delta_k) \cdot \Delta t$$

If longitudinal dynamics are also modeled in a coupled NMPC (Nonlinear Model Predictive Control), the speed update can be included as:

$$v_{k+1} = v_k + a_k \cdot \Delta t$$

Here \(L\) is the wheelbase and \(\Delta t\) is the sampling time; in the current implementation \(v_k\) is the measured speed used as a parameter along the horizon of that iteration rather than a decision variable.

## Discretization and horizons
Forward‑Euler discretization is applied via a symbolic model assembled in CasADi (CasADi, Computer Algebra for Automatic Differentiation and Optimization) and compiled to a Nonlinear Programming (NLP, Nonlinear Programming) problem with equality constraints enforcing stepwise dynamics across the prediction horizon. The prediction horizon uses \(N\) steps with decision variables for states at \(k=0\ldots N\) and controls at \(k=0\ldots N-1\), balancing look‑ahead and feasibility on embedded hardware.

## Objective function
The stage cost penalizes lateral offset, yaw error, and steering rate changes for smoothness, including an initial term relative to the previously applied steering to avoid a first‑step jump. The cost used is:

$$
J = \sum_{k=0}^{N-1}\Big( Q_{\text{offset}}\; y_k^2 + Q_{\psi}\; \psi_k^2 + R_{\Delta\delta}\; (\delta_k-\delta_{k-1})^2 \Big)
$$

Weights \(Q_{\text{offset}}, Q_{\psi}, R_{\Delta\delta}\) are empirically calibrated to balance tracking accuracy and command smoothness under typical operating conditions.

## Constraints
A hard bound on steering is enforced as \(|\delta_k|\leq 40^\circ\) (applied in radians internally), aligned with actuator saturation downstream. There are currently no state constraints (e.g., lateral corridor) and no hard bound on the steering rate \(|\Delta\delta_k|\), with smoothness handled by the rate penalty only.

## Solver and formulation
The optimization is posed as an NLP with stacked state and input trajectories, equality constraints for dynamics, and bounds for inputs, solved by IPOPT (IPOPT, Interior Point OPTimizer) via CasADi’s nlpsol interface; typical configurations include capped iterations and print suppression for embedded use. Primal warm‑start of the decision vector between cycles is used to reduce interior‑point iteration counts and improve timing stability.

## Initialization and warm start
Per cycle, parameters include the initial state \([x_0,y_0,\psi_0]\), the measured speed \(v\), and the previously applied steering \(\delta_{-1}\), while the primal initial guess is taken from the previous solution to accelerate convergence. Dual warm‑starts (lam_x0/lam_g0, Lagrange multipliers initial guesses) are not currently provided but are supported and can further reduce interior‑point iterations if needed .

## Real‑time behavior
The sampling time \(\Delta t\) should match end‑to‑end perception plus optimization latency so the closed loop meets real‑time timing on Jetson Nano; empirical profiling of average/worst solve time, IPOPT iteration counts, and jitter should be recorded to demonstrate budget compliance. The measured speed is passed every cycle and held constant across the horizon for that iteration to reduce problem size while reflecting current operating conditions.

## Tuning rationale
Weights \(Q_{\text{offset}}, Q_{\psi}, R_{\Delta\delta}\) are tuned empirically using lateral RMS error, yaw RMS error, and command smoothness to avoid oscillations and minimize actuator stress . The horizon \(N\) balances preview and computation, trading anticipation of curvature against solve time on the embedded target.

## Safety and fallbacks
Actuator saturation should be enforced both in the solver and in the actuator layer to avoid interface mismatches that could cause clipping or unintended behavior. It is recommended to check solver status before applying the new steering and to fall back to the last valid filtered command on failure, consistent with ISO 26262 (ISO 26262, Road vehicles — Functional safety) supporting processes for fault handling and safe state strategies.

## Verification and validation (V&V)
Recommended tests include unit tests for dynamics constraint satisfaction and bound enforcement, integration tests in closed loop on recorded scenarios, and system tests with requirements‑tied metrics (e.g., lateral RMS error thresholds and maximum allowed steering slew) archived with parameter versions and solver logs, per ISO 26262‑6 (Product development at the software level) expectations. Capturing solver objective values, iteration counts, and timing per run provides auditable evidence for calibration changes and supports regression control.

## Known limitations
- No terminal cost/set (terminal quadratic cost or invariant terminal set), which limits formal stability/recursive feasibility guarantees commonly used in stabilizing MPC (stabilizing MPC, stabilizing Model Predictive Control).  
- No lateral corridor (state constraints) derived from lane geometry/obstacles, reducing safety margins in narrow or curved roads compared to corridor‑constrained MPC formulations.  
- No hard bound on steering rate \(|\Delta\delta_k|\), relying solely on a rate penalty that can still allow rapid steps if locally optimal under current weights .

## Studied future work
- Add terminal cost \(V_f(x_N)\) and/or terminal set to improve stability margins and recursive feasibility under constraints for constrained tracking MPC.
- Introduce a road‑aligned lateral corridor (hard/soft) from lane geometry and obstacle envelopes to enforce state safety bounds with comfort.
- Enforce hard bounds on steering rate \(|\Delta\delta_k|\) to reflect actuator slew limits alongside the existing smoothness penalty .  
- Warm‑start dual variables (lam_x0/lam_g0) in IPOPT to reduce iteration counts; CasADi nlpsol exposes these initializations for interior‑point warm starts (IPM, Interior‑Point Method warm starts) .  
- Implement solver‑status checks and safe fallbacks (e.g., hold last valid filtered command) to handle occasional non‑convergence without unsafe transients .  
- Add structured logging of parameters, objectives, iterations, and timings per cycle for traceability, calibration audits, and regression testing under safety processes .  
- Evaluate a convex Quadratic Program (QP, Quadratic Program) formulation with linearization (LTV, Linear Time‑Varying) and an operator‑splitting solver such as OSQP (OSQP, Operator Splitting Quadratic Program) for more predictable timing where applicable, while preserving NMPC for higher fidelity .

## Parameters 
| Parameter | Value | |
|---|---:|---|
| Wheelbase L | 0.15 m |  |
| Sample time dt | 0.1 s |  |
| Prediction horizon N | 10 steps |  |
| Steering limit | ±30° (rad internally) |  |
| Q_offset | 3.0 | |
| Q_psi | 0.60 | |
| R_delta_rate | 0.1 | |

## API and usage
Call computeControl(offset_m, psi_rad, current_velocity_mps) each control tick with SI units; the function returns steering \(\delta\) [rad] that is subsequently converted to degrees and saturated for the actuator interface. The controller maintains internal warm‑start state (previous solution and steering), so calls should be sequential within a single control loop for consistent performance.

## ISO 26262 alignment note
To align with ISO 26262‑6 (Product development at the software level), maintain work products for software safety requirements, architectural design, unit and integration test evidence, parameter versioning, configuration/change management, and tool usage documentation tied to solver and tuning changes; the present code implements the control logic but compliance depends on the surrounding lifecycle, documentation, and verification artifacts. These “Known limitations” and “Studied future work” sections document design decisions and planned mitigations, supporting safety case traceability when features are deferred due to schedule constraints.

