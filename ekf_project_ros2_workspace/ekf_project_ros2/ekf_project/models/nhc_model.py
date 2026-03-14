from __future__ import annotations

import math
import numpy as np

from ekf_project.config.filter_config import EKFConfig
from ekf_project.data.packet_types import VehiclePacket, ImuPacket
from ekf_project.models.measurement_base import MeasurementBundle


class NHCModel:
    def __init__(self, cfg: EKFConfig):
        self.cfg = cfg
        self.idx = {name: cfg.state_spec.idx(name) for name in cfg.state_spec.state_order}

    def build(self, x: np.ndarray, veh: VehiclePacket, last_imu: ImuPacket | None) -> MeasurementBundle:
        psi = float(x[self.idx['psi'], 0])
        vx = float(x[self.idx['vx'], 0])
        vy = float(x[self.idx['vy'], 0])
        bgz = float(x[self.idx['bgz'], 0])
        bdelta = float(x[self.idx['bdelta'], 0])

        c = math.cos(psi)
        s = math.sin(psi)
        v_long = c * vx + s * vy
        v_lat = -s * vx + c * vy

        z = np.array([[0.0]], dtype=float)
        h = np.array([[v_lat]], dtype=float)
        H = np.zeros((1, self.cfg.state_spec.size), dtype=float)
        H[0, self.idx['psi']] = -v_long
        H[0, self.idx['vx']] = -s
        H[0, self.idx['vy']] = c

        R_scalar = self.cfg.R.R_nhc
        if last_imu is not None:
            delta_eff = veh.delta - bdelta
            r_imu = self.cfg.conventions.yaw_rate_sign_to_heading * (last_imu.wz - bgz)
            r_model = (v_long / max(self.cfg.vehicle.wheelbase_L, 1e-6)) * math.tan(delta_eff)
            e_r = r_imu - r_model
            abs_e = abs(e_r)
            if abs_e > self.cfg.valid.moderate_consistency_residual_threshold:
                R_scalar *= self.cfg.valid.moderate_consistency_inflation_factor
        R = np.array([[R_scalar]], dtype=float)
        return MeasurementBundle('nhc', z, h, H, R, self.cfg.valid.innovation_gate_nhc)
