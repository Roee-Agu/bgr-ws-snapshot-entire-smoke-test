from __future__ import annotations

import math
import numpy as np

from ekf_project.config.filter_config import EKFConfig
from ekf_project.data.packet_types import GpsPacket, VehiclePacket, ImuPacket


def finite(*values: float) -> bool:
    return all(np.isfinite(v) for v in values)


def check_gps_position_valid(cfg: EKFConfig, gps: GpsPacket) -> tuple[bool, str]:
    if not gps.is_new:
        return False, 'not a fresh GPS sample'
    if not finite(gps.px, gps.py):
        return False, 'invalid px/py'
    if cfg.enable.use_sats_if_available and gps.sats is not None and gps.sats < cfg.valid.gps_min_sats:
        return False, 'sats below minimum'
    if cfg.enable.use_pos_accuracy_if_available and gps.pos_accuracy is not None and gps.pos_accuracy > cfg.valid.gps_max_pos_accuracy_hard:
        return False, 'pos_accuracy above hard maximum'
    return True, 'ok'


def check_gps_speed_valid(cfg: EKFConfig, gps: GpsPacket) -> tuple[bool, str]:
    if not gps.is_new:
        return False, 'not a fresh GPS sample'
    if gps.v_gps is None or not np.isfinite(gps.v_gps):
        return False, 'missing v_gps'
    if cfg.enable.use_spd_accuracy_if_available and gps.spd_accuracy is not None and gps.spd_accuracy > cfg.valid.gps_max_spd_accuracy_hard:
        return False, 'spd_accuracy above hard maximum'
    return True, 'ok'


def check_vehicle_packet_valid(cfg: EKFConfig, veh: VehiclePacket) -> tuple[bool, str]:
    vals = [veh.delta, veh.rpm_fl, veh.rpm_fr, veh.rpm_rl, veh.rpm_rr]
    if not finite(*vals):
        return False, 'non-finite vehicle packet'
    if abs(veh.delta) > cfg.valid.max_abs_delta:
        return False, 'steering above physical sanity'
    rpms = [veh.rpm_fl, veh.rpm_fr, veh.rpm_rl, veh.rpm_rr]
    if any(abs(rpm) > cfg.valid.max_abs_rpm for rpm in rpms):
        return False, 'rpm above physical sanity'
    return True, 'ok'


def speed_from_state(x: np.ndarray, idx: dict[str, int]) -> float:
    vx = float(x[idx['vx'], 0])
    vy = float(x[idx['vy'], 0])
    return math.hypot(vx, vy)


def check_nhc_regime(cfg: EKFConfig, x: np.ndarray, idx: dict[str, int]) -> tuple[bool, str]:
    if speed_from_state(x, idx) < cfg.valid.nhc_min_speed:
        return False, 'speed below nhc minimum'
    return True, 'ok'


def check_steering_regime(cfg: EKFConfig, x: np.ndarray, idx: dict[str, int], veh: VehiclePacket, last_imu: ImuPacket | None) -> tuple[bool, str]:
    if last_imu is None:
        return False, 'missing latest IMU'
    if speed_from_state(x, idx) < cfg.valid.steering_min_speed:
        return False, 'speed below steering minimum'
    if abs(veh.delta) > cfg.valid.steering_max_abs_delta:
        return False, 'steering outside trusted regime'
    if abs(last_imu.wz) > cfg.valid.max_abs_wz:
        return False, 'yaw rate above physical sanity'
    return True, 'ok'
