# ekf_project

A modular 2D land-vehicle EKF workspace for GNSS/IMU/wheel-speed/constraint fusion.

## Academic alignment
This workspace follows the standard integrated-navigation structure used in the literature:
- IMU-driven propagation core
- modular aiding updates
- vehicle-specific constraints kept optional and conditional

That matches the modular multisensor-navigation approach discussed by Groves and the land-vehicle GPS + low-cost IMU + onboard-sensor Kalman-filter architecture discussed by Gao et al.

## Package layout
```text
ekf_project/
    config/
    data/
    models/
    core/
    runners/
    analysis/
```

## State
`[px, py, psi, vx, vy, bax, bay, bgz, sws, bdelta]`

## Default staged updates
Phase 1 defaults are intentionally conservative:
1. GPS position enabled
2. Wheel speed disabled
3. NHC disabled
4. Steering constraint disabled
5. GPS speed disabled

Later, enable modules in this order:
`GPS pos -> wheel speed -> NHC -> steering constraint`

## ROS2 simulator wiring
The simulation runner subscribes to the four topics published by `ekf_publisher.py`:
- `/robot/datalogger_gt`
- `/robot/datalogger_noisy`
- `/robot/mcu_gt`
- `/robot/mcu_noisy`

Noisy topics drive the EKF.
GT topics are stored only for diagnostics/reference.

Published outputs:
- `/robot/ekf_state` as `Float64MultiArray` with state-order values
- `/robot/ekf_diag` as `Float64MultiArray` with basic diagnostics

## Startup initialization
At startup, the runner collects a stationary IMU window and estimates:
- `bax0`
- `bay0`
- `bgz0`

These are STARTUP_ESTIMATE quantities and should be recomputed every run.

## Install / run (ROS2 Python package)
From a ROS2 workspace `src/` folder, place this package and build with:
```bash
colcon build --packages-select ekf_project
source install/setup.bash
ros2 run ekf_project run_simulation
```

## Parameter provenance markers
Comments in config mark whether parameters are expected from:
- DESIGN
- SPEC
- EXPERIMENT
- STARTUP_ESTIMATE
- SIM_ONLY


## Live RMSE plotter
The package also includes `live_rmse_plotter`, which compares `/robot/estimated_odom` against `/model/bgr/odometry` and displays live RMSE.
