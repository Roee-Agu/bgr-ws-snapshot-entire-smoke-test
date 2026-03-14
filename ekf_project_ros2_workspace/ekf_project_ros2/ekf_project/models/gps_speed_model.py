from __future__ import annotations

import math
import numpy as np

from ekf_project.config.filter_config import EKFConfig
from ekf_project.data.packet_types import GpsPacket
from ekf_project.models.measurement_base import MeasurementBundle


class GPSSpeedModel:
    def __init__(self, cfg: EKFConfig):
        self.cfg = cfg
        self.idx = {name: cfg.state_spec.idx(name) for name in cfg.state_spec.state_order}

    def build(self, x: np.ndarray, gps: GpsPacket) -> MeasurementBundle:
        vx = float(x[self.idx['vx'], 0])
        vy = float(x[self.idx['vy'], 0])
        vnorm = math.hypot(vx, vy)
        z = np.array([[float(gps.v_gps)]], dtype=float)
        h = np.array([[vnorm]], dtype=float)
        H = np.zeros((1, self.cfg.state_spec.size), dtype=float)
        if vnorm > 1e-9:
            H[0, self.idx['vx']] = vx / vnorm
            H[0, self.idx['vy']] = vy / vnorm
        R_scalar = self.cfg.R.R_gps_speed_nominal
        if self.cfg.enable.use_spd_accuracy_if_available and gps.spd_accuracy is not None:
            nominal_sigma = math.sqrt(max(self.cfg.R.R_gps_speed_nominal, 1e-12))
            scale = max(gps.spd_accuracy / nominal_sigma, 1.0)
            R_scalar = (scale**2) * R_scalar
        R = np.array([[R_scalar]], dtype=float)
        return MeasurementBundle('gps_speed', z, h, H, R, self.cfg.valid.innovation_gate_gps_speed)
