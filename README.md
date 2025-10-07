# Autonomous Lane-Detection and Control on Jetson Nano

End-to-end lane perception and control stack on NVIDIA Jetson Nano: lane and object inference (TensorRT), lane geometry extraction (offset, ψ), lateral control via MPC (Model Predictive Control), and longitudinal control via PID (Proportional–Integral–Derivative), designed for deterministic real-time on constrained hardware.

## Features

- Lane segmentation inference (TensorRT) with ROI-focused post-processing and low-overhead geometry extraction (linear fits + row-dependent scale s(y)=a·y+b).
- MPC-based steering (kinematic bicycle, measured speed per cycle, input bounds, rate smoothing, warm start).
- PID-based speed control (derivative on measurement, conditional anti-windup, output slew limiting).
- ZeroMQ (PUB/SUB) telemetry and control messaging (middleware hosted in a separate repository).

## Requirements

- Hardware: NVIDIA Jetson Nano, CSI/USB camera, steering servo, drivetrain actuators.
- Software: CUDA, cuDNN, TensorRT (Jetson image), OpenCV, C++17 toolchain (CMake ≥ 3.16).
- Engines: prebuilt TensorRT engines for lanes and objects placed under ./engines/.

Notes:
- Ensure the camera device is accessible and the process has permissions to access actuators.
- Engines must match the Jetson’s TensorRT version and the model’s expected input layout.

## Configuration

- MPC: wheelbase L, sample time Δt, horizon N, weights (Q_offset, Q_ψ, R_Δδ), steering bounds.
- PID: Kp, Ki, Kd, integral clamp, PWM bounds, per-cycle slew limit.
- Lanes: ROI (height fraction), morphology kernel, thresholds, continuity parameters, lane width W.
- Paths: engines/, optional logs/, and ZMQ endpoints (if applicable in this repo).

## Messaging (ZeroMQ)

The application publishes/subscribes runtime telemetry and control topics via ZeroMQ (PUB/SUB). Message schemas, endpoints, QoS (HWM/LINGER/CONFLATE), and security are maintained in a separate middleware repository:

- Middleware README: https://github.com/ORG/REPO#readme

## Documentation

Module-level technical docs with formulas and implementation details:

- Lateral MPC: control_systems/MPC/README.md (or docs/MPC.md)
- Longitudinal PID: control_systems/PID/README.md (or docs/PID.md)
- Lane geometry: control_systems/LanesGeometry/README.md (or docs/Lanes.md)
- Inference (TensorRT): object_detection/README.md and/or docs/Inference.md

GitHub supports relative links; keep docs alongside code or under docs/ and link as:
- [MPC](control_systems/MPC/README.md)
- [PID](control_systems/PID/README.md)
- [Lanes](control_systems/LanesGeometry/README.md)
- [Inference](object_detection/README.md)
- [ZMQ](control_systems/ZMQ/README.md)

## Testing and validation

- Unit: controller math (MPC/PID), bounds, anti-windup, lane geometry invariants.
- Integration: closed-loop tests logging latency/fps, solver iterations, offset/ψ stability.
- System: track runs with acceptance metrics (lateral RMS, yaw RMS, steering slew, speed tracking).
- Archive calibration versions and logs for traceability.

## Safety notice

Operate in controlled environments with supervision; actuator limits are enforced in software but must be validated on the target platform. Review module limitations and planned improvements in the respective docs before deployment.

## Contributing

Open issues/PRs with clear descriptions, test evidence, and updated docs. Follow conventional commit messages and run build/tests locally before submission.

## License

See LICENSE for terms governing use and redistribution.