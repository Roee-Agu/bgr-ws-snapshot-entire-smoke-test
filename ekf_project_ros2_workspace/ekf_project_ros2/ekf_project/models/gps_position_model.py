from __future__ import annotations

import numpy as np

from ekf_project.config.filter_config import EKFConfig
from ekf_project.data.packet_types import GpsPacket
from ekf_project.models.measurement_base import MeasurementBundle


class GPSPositionModel:
    def __init__(self, cfg: EKFConfig):
        self.cfg = cfg
        self.idx = {name: cfg.state_spec.idx(name) for name in cfg.state_spec.state_order}

    def build(self, x: np.ndarray, gps: GpsPacket) -> MeasurementBundle:
        z = np.array([[gps.px], [gps.py]], dtype=float)
        h = np.array([[x[self.idx['px'], 0]], [x[self.idx['py'], 0]]], dtype=float)
        H = np.zeros((2, self.cfg.state_spec.size), dtype=float)
        H[0, self.idx['px']] = 1.0
        H[1, self.idx['py']] = 1.0
        R = np.array(self.cfg.R.R_gps_pos_nominal, dtype=float)
        if self.cfg.enable.use_pos_accuracy_if_available and gps.pos_accuracy is not None:
            nominal_sigma = float(np.sqrt(max(self.cfg.R.R_gps_pos_nominal[0, 0], 1e-12)))
            scale = max(gps.pos_accuracy / nominal_sigma, 1.0)
            R = (scale**2) * R
        return MeasurementBundle('gps_pos', z, h, H, R, self.cfg.valid.innovation_gate_gps_pos)
