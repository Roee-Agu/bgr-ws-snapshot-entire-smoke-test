from __future__ import annotations

from typing import List, Dict
import numpy as np

from ekf_project.config.filter_config import EKFConfig
from ekf_project.data.packet_types import ImuPacket, GpsPacket


class StartupInitializer:
    """Collects a stationary IMU window and estimates startup biases.

    These are STARTUP_ESTIMATE parameters and should be recomputed each run.
    """
    def __init__(self, cfg: EKFConfig):
        self.cfg = cfg
        self.imu_window: List[ImuPacket] = []
        self.t0: float | None = None

    def push_imu(self, pkt: ImuPacket) -> None:
        if self.t0 is None:
            self.t0 = pkt.t
        self.imu_window.append(pkt)

    def ready(self) -> bool:
        if not self.imu_window or self.t0 is None:
            return False
        duration = self.imu_window[-1].t - self.t0
        return duration >= self.cfg.init.startup_stationary_duration_s and len(self.imu_window) >= self.cfg.init.min_samples_for_bias_init

    def estimate_biases(self) -> Dict[str, float]:
        if not self.ready():
            raise RuntimeError('StartupInitializer is not ready.')
        ax = np.array([p.ax for p in self.imu_window], dtype=float)
        ay = np.array([p.ay for p in self.imu_window], dtype=float)
        wz = np.array([p.wz for p in self.imu_window], dtype=float)
        return {
            'bax0': float(np.mean(ax)),
            'bay0': float(np.mean(ay)),
            'bgz0': float(np.mean(wz)),
            'sigma_ax_static': float(np.std(ax, ddof=1)),
            'sigma_ay_static': float(np.std(ay, ddof=1)),
            'sigma_wz_static': float(np.std(wz, ddof=1)),
        }


def build_initial_state(cfg: EKFConfig, gps: GpsPacket | None) -> np.ndarray:
    n = cfg.state_spec.size
    x0 = np.zeros((n, 1), dtype=float)
    idx = {name: cfg.state_spec.idx(name) for name in cfg.state_spec.state_order}
    if gps is not None:
        x0[idx['px'], 0] = gps.px
        x0[idx['py'], 0] = gps.py
    x0[idx['psi'], 0] = cfg.init.psi0_default
    x0[idx['vx'], 0] = cfg.init.vx0_default
    x0[idx['vy'], 0] = cfg.init.vy0_default
    x0[idx['sws'], 0] = cfg.init.sws0
    x0[idx['bdelta'], 0] = cfg.init.bdelta0
    return x0


def build_initial_covariance(cfg: EKFConfig, gps: GpsPacket | None) -> np.ndarray:
    P = np.diag([
        cfg.P0.sigma_px0**2,
        cfg.P0.sigma_py0**2,
        cfg.P0.sigma_psi0**2,
        cfg.P0.sigma_vx0**2,
        cfg.P0.sigma_vy0**2,
        cfg.P0.sigma_bax0**2,
        cfg.P0.sigma_bay0**2,
        cfg.P0.sigma_bgz0**2,
        cfg.P0.sigma_sws0**2,
        cfg.P0.sigma_bdelta0**2,
    ])
    if gps is not None and gps.pos_accuracy is not None:
        idx = {name: cfg.state_spec.idx(name) for name in cfg.state_spec.state_order}
        P[idx['px'], idx['px']] = gps.pos_accuracy**2
        P[idx['py'], idx['py']] = gps.pos_accuracy**2
    return P
