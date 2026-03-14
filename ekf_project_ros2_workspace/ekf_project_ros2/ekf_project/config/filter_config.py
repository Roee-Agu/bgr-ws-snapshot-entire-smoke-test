from __future__ import annotations

"""Central EKF configuration schema.

Parameter provenance tags used in comments:
- DESIGN: fixed by model/architecture choice
- SPEC: from sensor documentation / datasheet
- EXPERIMENT: from bench or field calibration
- STARTUP_ESTIMATE: re-estimated every run from stationary startup data
- SIM_ONLY: only meaningful for simulator-side publishing
"""

from dataclasses import dataclass, field
from enum import Enum
import math
import numpy as np


class WheelSelector(str, Enum):
    REAR_AVG = 'rear_avg'
    FRONT_AVG = 'front_avg'
    ALL_AVG = 'all_avg'
    RL_ONLY = 'rl_only'
    RR_ONLY = 'rr_only'
    FL_ONLY = 'fl_only'
    FR_ONLY = 'fr_only'


class PsiInitStrategy(str, Enum):
    ZERO = 'zero'
    FROM_MOTION = 'from_motion'


@dataclass(slots=True)
class ConventionsConfig:
    # DESIGN
    heading_positive_ccw: bool = True
    body_frame: str = 'forward_left'
    nav_frame: str = 'local_enu'
    angle_wrap_interval: str = 'pi'

    # DESIGN / adapter bridge
    yaw_rate_sign_to_heading: float = 1.0


@dataclass(slots=True)
class StateSpecConfig:
    # DESIGN
    state_order: tuple[str, ...] = (
        'px', 'py', 'psi', 'vx', 'vy', 'bax', 'bay', 'bgz', 'sws', 'bdelta'
    )

    @property
    def size(self) -> int:
        return len(self.state_order)

    def idx(self, name: str) -> int:
        return self.state_order.index(name)


@dataclass(slots=True)
class VehicleParamsConfig:
    # SPEC / EXPERIMENT
    wheelbase_L: float = 1.6
    rpm_to_speed_gain_Krpm: float = 0.01
    wheel_selector: WheelSelector = WheelSelector.REAR_AVG

    # DESIGN / EXPERIMENT
    min_speed_for_nhc: float = 1.0
    min_speed_for_steering_constraint: float = 1.5
    max_abs_steering_for_constraint: float = math.radians(25.0)


@dataclass(slots=True)
class InitializationConfig:
    # STARTUP_ESTIMATE / DESIGN
    startup_stationary_duration_s: float = 30.0
    min_samples_for_bias_init: int = 200
    psi0_strategy: PsiInitStrategy = PsiInitStrategy.ZERO
    psi0_default: float = 0.0

    # DESIGN defaults for slowly learned states
    sws0: float = 1.0
    bdelta0: float = 0.0
    vx0_default: float = 0.0
    vy0_default: float = 0.0


@dataclass(slots=True)
class InitialCovarianceConfig:
    sigma_px0: float = 3.0
    sigma_py0: float = 3.0
    sigma_psi0: float = math.radians(45.0)
    sigma_vx0: float = 1.0
    sigma_vy0: float = 1.0
    sigma_bax0: float = 0.5
    sigma_bay0: float = 0.5
    sigma_bgz0: float = math.radians(2.0)
    sigma_sws0: float = 0.20
    sigma_bdelta0: float = math.radians(5.0)


@dataclass(slots=True)
class ProcessNoiseConfig:
    # SPEC or EXPERIMENT
    sigma_ax: float = 0.5
    sigma_ay: float = 0.5
    sigma_wz: float = math.radians(0.2)

    # EXPERIMENT
    q_bax: float = 1e-3
    q_bay: float = 1e-3
    q_bgz: float = math.radians(0.02) ** 2
    q_sws: float = 1e-5
    q_bdelta: float = math.radians(0.05) ** 2


@dataclass(slots=True)
class MeasurementNoiseConfig:
    # SPEC / EXPERIMENT / tuning
    R_gps_pos_nominal: np.ndarray = field(default_factory=lambda: np.diag([2.0**2, 2.0**2]))
    R_gps_speed_nominal: float = 0.5**2
    R_wheel_speed: float = 0.5**2
    R_nhc: float = 0.5**2
    R_steering_constraint: float = math.radians(5.0) ** 2

@dataclass(slots=True)
class EnableFlagsConfig:
    # DESIGN. Defaults are phase-1 safe: IMU + GPS position only.
    use_gps_pos: bool = True
    use_gps_speed: bool = False
    use_wheel_speed: bool = False
    use_nhc: bool = False
    use_steering_constraint: bool = False

    # DESIGN
    use_pos_accuracy_if_available: bool = True
    use_spd_accuracy_if_available: bool = True
    use_sats_if_available: bool = True


@dataclass(slots=True)
class ValidationConfig:
    # SPEC / EXPERIMENT / tuning
    gps_min_sats: int = 4
    gps_max_pos_accuracy_hard: float = 20.0
    gps_max_spd_accuracy_hard: float = 10.0

    nhc_min_speed: float = 1.0
    steering_min_speed: float = 1.5
    steering_max_abs_delta: float = math.radians(25.0)

    max_abs_rpm: float = 20000.0
    max_abs_wz: float = math.radians(400.0)
    max_abs_delta: float = math.radians(45.0)

    innovation_gate_gps_pos: float = 9.21
    innovation_gate_gps_speed: float = 6.63
    innovation_gate_wheel_speed: float = 6.63
    innovation_gate_nhc: float = 6.63
    innovation_gate_steering: float = 6.63

    low_speed_wheel_inflation_threshold: float = 0.5
    low_speed_wheel_inflation_factor: float = 4.0
    moderate_consistency_inflation_factor: float = 4.0
    large_consistency_residual_threshold: float = math.radians(20.0)
    moderate_consistency_residual_threshold: float = math.radians(8.0)


@dataclass(slots=True)
class AdapterConfig:
    # Adapter-layer only
    lat0_deg: float | None = None
    lon0_deg: float | None = None

    real_accel_in_g: bool = True
    real_gyro_in_deg_s: bool = True
    real_steering_in_deg: bool = False

    yaw_rate_sign: float = 1.0
    accel_x_sign: float = 1.0
    accel_y_sign: float = 1.0
    steering_sign: float = 1.0
    rpm_fl_sign: float = 1.0
    rpm_fr_sign: float = 1.0
    rpm_rl_sign: float = 1.0
    rpm_rr_sign: float = 1.0


@dataclass(slots=True)
class RosTopicsConfig:
    # DESIGN / adapter wiring for ROS2.
    datalogger_gt_topic: str = '/robot/datalogger_gt'
    datalogger_noisy_topic: str = '/robot/datalogger_noisy'
    mcu_gt_topic: str = '/robot/mcu_gt'
    mcu_noisy_topic: str = '/robot/mcu_noisy'
    ekf_state_topic: str = '/robot/ekf_state'
    ekf_diag_topic: str = '/robot/ekf_diag'
    estimated_odom_topic: str = '/robot/estimated_odom'
    qos_depth: int = 10
    status_period_s: float = 1.0


@dataclass(slots=True)
class EKFConfig:
    conventions: ConventionsConfig = field(default_factory=ConventionsConfig)
    state_spec: StateSpecConfig = field(default_factory=StateSpecConfig)
    vehicle: VehicleParamsConfig = field(default_factory=VehicleParamsConfig)
    init: InitializationConfig = field(default_factory=InitializationConfig)
    P0: InitialCovarianceConfig = field(default_factory=InitialCovarianceConfig)
    Q: ProcessNoiseConfig = field(default_factory=ProcessNoiseConfig)
    R: MeasurementNoiseConfig = field(default_factory=MeasurementNoiseConfig)
    enable: EnableFlagsConfig = field(default_factory=EnableFlagsConfig)
    valid: ValidationConfig = field(default_factory=ValidationConfig)
    adapter: AdapterConfig = field(default_factory=AdapterConfig)
    ros: RosTopicsConfig = field(default_factory=RosTopicsConfig)
