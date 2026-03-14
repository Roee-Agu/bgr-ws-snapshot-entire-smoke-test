from __future__ import annotations

import math
import numpy as np

from ekf_project.config.filter_config import EKFConfig
from ekf_project.data.packet_types import VehiclePacket
from ekf_project.models.measurement_base import MeasurementBundle


class WheelSpeedModel:
    def __init__(self, cfg: EKFConfig):
        self.cfg = cfg
        self.idx = {name: cfg.state_spec.idx(name) for name in cfg.state_spec.state_order}

    def build(self, x: np.ndarray, veh: VehiclePacket) -> MeasurementBundle:
        psi = float(x[self.idx['psi'], 0])
        vx = float(x[self.idx['vx'], 0])
        vy = float(x[self.idx['vy'], 0])
        sws = float(x[self.idx['sws'], 0])
        c = math.cos(psi)
        s = math.sin(psi)
        v_long = c * vx + s * vy

        z = np.array([[0.0]], dtype=float)
        h = np.array([[v_long - sws * float(veh.v_rpm_nominal)]], dtype=float)
        H = np.zeros((1, self.cfg.state_spec.size), dtype=float)
        H[0, self.idx['psi']] = -s * vx + c * vy
        H[0, self.idx['vx']] = c
        H[0, self.idx['vy']] = s
        H[0, self.idx['sws']] = -float(veh.v_rpm_nominal)

        R_scalar = self.cfg.R.R_wheel_speed
        if abs(float(veh.v_rpm_nominal)) < self.cfg.valid.low_speed_wheel_inflation_threshold:
            R_scalar *= self.cfg.valid.low_speed_wheel_inflation_factor
        R = np.array([[R_scalar]], dtype=float)
        return MeasurementBundle('wheel_speed', z, h, H, R, self.cfg.valid.innovation_gate_wheel_speed)
