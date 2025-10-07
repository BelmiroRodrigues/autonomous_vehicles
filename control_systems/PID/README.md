# PID (Proportional–Integral–Derivative) — Longitudinal Speed Control Specification

This document specifies the longitudinal PID (Proportional–Integral–Derivative) controller used to track a target speed via PWM actuation, including scope, interfaces, equations, derivative filtering, anti‑windup, output slew limiting, tuning, safety, and V&V (Verification and Validation) suitable for automotive documentation.

## Purpose and scope
The controller minimizes speed tracking error using proportional, integral, and derivative actions with embedded‑oriented robustness features (noise filtering, anti‑windup, slew limiting) for stable and comfortable longitudinal control.

## Interfaces
- Input (per cycle): setpoint speed r [m/s], measured speed y [m/s], and loop interval Δt [s], passed to compute(setpoint, measurement, dt).  
- Output (per cycle): actuator command u (PWM) within configured limits [outputMin, outputMax], with additional per‑cycle slew limiting (±Δu_max)[7].

## Discrete equations (as implemented)

- Error and proportional term:
$$e_k = r_k - y_k$$
$$P_k = K_p \cdot e_k$$

- Derivative on measurement with 1st‑order low‑pass filter (avoids derivative kick and attenuates noise):
$$y^{f}_k = y^{f}_{k-1} + \alpha \cdot (y_k - y^{f}_{k-1})$$
$$D_k = -K_d \cdot \frac{y^{f}_k - y^{f}_{k-1}}{\Delta t}$$

- Integral candidate, conditional integration, and clamping:
$$I^{cand}_k = I_{k-1} + K_i \cdot e_k \cdot \Delta t$$
$$I_k =
\begin{cases}
\mathrm{clip}(I^{cand}_k,\,-I_{\max},\,I_{\max}), & \text{if } u^{unsat}_k \in [u_{\min},u_{\max}] \text{ or integration helps desaturate} \\
I_{k-1}, & \text{otherwise}
\end{cases}
$$

- Output assembly, saturation, and slew limiting:
$$u^{unsat}_k = P_k + I_k + D_k$$
$$u^{sat}_k = \mathrm{clip}(u^{unsat}_k,\,u_{\min},\,u_{\max})$$
$$u_k = \mathrm{clip}\!\left(u^{sat}_k,\,u_{k-1}-\Delta u_{\max},\,u_{k-1}+\Delta u_{\max}\right)$$

Implementation notes: derivative on measurement reduces sensitivity to setpoint steps (“derivative kick”) and high‑frequency noise; conditional integration (clamping) mitigates integral windup on saturation; per‑cycle slew limiting bounds command changes for actuator protection and comfort[16][17][7].

## Anti‑windup design
- Conditional integration (clamping): only accept integral growth if the unsaturated output is inside the limits, or if the sign of the error helps bring the saturated output back within limits; additionally clamp |I_k| ≤ I_max for safety margins in embedded contexts[2].  
- Alternative (studied): back‑calculation with a reset gain \(k_t\), adding a tracking term \(k_t(u^{sat}_k - u^{unsat}_k)\) to the integral path to achieve more predictable recovery from saturation in cascaded/complex actuators[1].

## Derivative filtering
- 1st‑order low‑pass filter on the measured speed precedes differencing, improving robustness to sensor noise and avoiding derivative kick on setpoint steps; α ∈ (0, 1] is configurable and trades noise rejection vs. lag[16].  
- Option (studied): 2nd‑order filter for stronger attenuation at the expense of additional phase lag and tuning complexity in fast loops[18].

## Output limiting and slew control
- Saturation enforces the actuator’s PWM envelope and prevents commands beyond safe capability.  
- Slew limiting caps per‑cycle change (±Δu_max), reducing wear and improving drivability; Δu_max should consider actuator slew rate and loop period Δt[7].

## Safety and operational notes
- Robustness to bad dt: compute() ignores dt ≤ 0 and returns lastOutput to avoid numerical spikes; reset() clears integral/filters for safe re‑entry.  
- Fault handling (recommendation): log saturation flags, integral clamp events, and slew limiting activations to detect unsafe operating envelopes during testing and in field logs.

## Tuning and calibration procedure
- Sequence: increase Kp to desired rise time without sustained oscillation; add Ki to eliminate steady‑state error while checking saturation and windup behavior; add Kd with filtering to damp overshoot and attenuate measurement noise; adjust α and Δu_max to the actuator dynamics and noise level[19][16].  
- Metrics to record: RMS tracking error, overshoot, settling time, percentage of time saturated, and distribution of per‑cycle slew; store alongside parameter versions for traceability.

## Verification and validation (V&V)
- Unit tests: derivative kick immunity on setpoint steps, correct conditional integration at limits, integral clamp enforcement, slew limiter correctness, and dt ≤ 0 handling[2].  
- Closed‑loop tests: step and ramp setpoints; injected disturbances; collect RMS error, overshoot, settling, saturation duty, and slew statistics; archive logs for calibration audits.

## Known limitations (current build)
- No setpoint weighting (β, γ) on P/D; adding weighting can further reduce sensitivity to setpoint steps in some drive cycles[11].  
- No back‑calculation anti‑windup path; conditional integration is effective, but back‑calculation can yield more predictable desaturation with complex actuators[1].  
- Derivative uses a 1st‑order filter; a 2nd‑order filter can improve noise rejection at the cost of lag[18].  
- Bumpless transfer (manual/auto or gain‑set changes) is not explicitly handled; align internal states on mode/tuning transitions to avoid transients if mode switching is required.

## Parameters (from code)

| Parameter | Value | Notes |
|---|---:|---|
| \(K_p\) | 6.0 | Proportional gain [20] |
| \(K_i\) | 2.0 | Integral gain [20] |
| \(K_d\) | 0.5 | Derivative gain [20] |
| \(u_{\min}\) (outputMin) | 0.0 | Minimum PWM [20] |
| \(u_{\max}\) (outputMax) | 40.0 | Maximum PWM [20] |
| \(\Delta u_{\max}\) (maxStepChange) | 2.0 | Per‑cycle slew limit [20] |
| \(I_{\max}\) (Imax) | 30.0 | Integral clamp [20] |
| \(\alpha\) (filter coeff.) | configurable | 1st‑order LPF for D term |

## API
- compute(setpoint, measurement, dt) → returns \(u_k\) with saturation and slew limiting; ignores non‑positive dt and returns last output for robustness.  
- reset() → clears integral, last output, and initialization flags for safe re‑entry and test reproducibility.

