from __future__ import annotations

import math
import numpy as np

from ekf_project.config.filter_config import EKFConfig
from ekf_project.data.packet_types import VehiclePacket, ImuPacket
from ekf_project.models.measurement_base import MeasurementBundle


class SteeringConstraintModel:
    def __init__(self, cfg: EKFConfig):
        self.cfg = cfg
        self.idx = {name: cfg.state_spec.idx(name) for name in cfg.state_spec.state_order}

    def build(self, x: np.ndarray, veh: VehiclePacket, last_imu: ImuPacket) -> MeasurementBundle:
        psi = float(x[self.idx['psi'], 0])
        vx = float(x[self.idx['vx'], 0])
        vy = float(x[self.idx['vy'], 0])
        bgz = float(x[self.idx['bgz'], 0])
        bdelta = float(x[self.idx['bdelta'], 0])

        c = math.cos(psi)
        s = math.sin(psi)
        v_long = c * vx + s * vy
        v_lat = -s * vx + c * vy
        delta_eff = veh.delta - bdelta
        L = max(self.cfg.vehicle.wheelbase_L, 1e-6)
        yaw_sign = self.cfg.conventions.yaw_rate_sign_to_heading

        r_imu = yaw_sign * (last_imu.wz - bgz)
        tan_delta = math.tan(delta_eff)
        r_model = (v_long / L) * tan_delta

        z = np.array([[0.0]], dtype=float)
        h = np.array([[r_imu - r_model]], dtype=float)
        H = np.zeros((1, self.cfg.state_spec.size), dtype=float)
        H[0, self.idx['psi']] = -(v_lat / L) * tan_delta
        H[0, self.idx['vx']] = -(c / L) * tan_delta
        H[0, self.idx['vy']] = -(s / L) * tan_delta
        H[0, self.idx['bgz']] = -yaw_sign
        H[0, self.idx['bdelta']] = (v_long / L) * (1.0 / math.cos(delta_eff) ** 2)

        R_scalar = self.cfg.R.R_steering_constraint
        if abs(float(h[0, 0])) > self.cfg.valid.moderate_consistency_residual_threshold:
            R_scalar *= self.cfg.valid.moderate_consistency_inflation_factor
        R = np.array([[R_scalar]], dtype=float)
        return MeasurementBundle('steering_constraint', z, h, H, R, self.cfg.valid.innovation_gate_steering)
