from __future__ import annotations

import math
import numpy as np

from ekf_project.config.filter_config import EKFConfig


def wrap_angle_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def ensure_symmetry(P: np.ndarray) -> np.ndarray:
    return 0.5 * (P + P.T)


class ProcessModel2D:
    def __init__(self, cfg: EKFConfig):
        self.cfg = cfg
        self.idx = {name: cfg.state_spec.idx(name) for name in cfg.state_spec.state_order}

    def propagate(self, x: np.ndarray, P: np.ndarray, ax: float, ay: float, wz: float, dt: float) -> tuple[np.ndarray, np.ndarray]:
        i = self.idx
        x_next = np.array(x, dtype=float, copy=True)

        px, py, psi, vx, vy, bax, bay, bgz, sws, bdelta = x[:, 0]
        yaw_sign = self.cfg.conventions.yaw_rate_sign_to_heading

        fx = ax - bax
        fy = ay - bay
        c = math.cos(psi)
        s = math.sin(psi)
        anx = c * fx - s * fy
        any_ = s * fx + c * fy
        wz_eff = yaw_sign * (wz - bgz)

        x_next[i['px'], 0] = px + vx * dt + 0.5 * anx * dt * dt
        x_next[i['py'], 0] = py + vy * dt + 0.5 * any_ * dt * dt
        x_next[i['psi'], 0] = wrap_angle_pi(psi + wz_eff * dt)
        x_next[i['vx'], 0] = vx + anx * dt
        x_next[i['vy'], 0] = vy + any_ * dt
        x_next[i['sws'], 0] = sws
        x_next[i['bdelta'], 0] = bdelta

        n = self.cfg.state_spec.size
        F = np.eye(n, dtype=float)

        F[i['px'], i['vx']] = dt
        F[i['px'], i['psi']] = -0.5 * any_ * dt * dt
        F[i['px'], i['bax']] = -0.5 * c * dt * dt
        F[i['px'], i['bay']] = 0.5 * s * dt * dt

        F[i['py'], i['vy']] = dt
        F[i['py'], i['psi']] = 0.5 * anx * dt * dt
        F[i['py'], i['bax']] = -0.5 * s * dt * dt
        F[i['py'], i['bay']] = -0.5 * c * dt * dt

        F[i['psi'], i['bgz']] = -yaw_sign * dt

        F[i['vx'], i['psi']] = -any_ * dt
        F[i['vx'], i['bax']] = -c * dt
        F[i['vx'], i['bay']] = s * dt

        F[i['vy'], i['psi']] = anx * dt
        F[i['vy'], i['bax']] = -s * dt
        F[i['vy'], i['bay']] = -c * dt

        Q = self.build_Q(psi, dt)
        P_next = ensure_symmetry(F @ P @ F.T + Q)
        return x_next, P_next

    def build_Q(self, psi: float, dt: float) -> np.ndarray:
        i = self.idx
        n = self.cfg.state_spec.size
        Q = np.zeros((n, n), dtype=float)

        Sa_b = np.diag([self.cfg.Q.sigma_ax**2, self.cfg.Q.sigma_ay**2])
        c = math.cos(psi)
        s = math.sin(psi)
        R = np.array([[c, -s], [s, c]], dtype=float)
        Sa_n = R @ Sa_b @ R.T
        Qpv = np.block([
            [0.25 * dt**4 * Sa_n, 0.5 * dt**3 * Sa_n],
            [0.5 * dt**3 * Sa_n, dt**2 * Sa_n],
        ])
        pv = [i['px'], i['py'], i['vx'], i['vy']]
        for rl, rg in enumerate(pv):
            for cl, cg in enumerate(pv):
                Q[rg, cg] = Qpv[rl, cl]

        Q[i['psi'], i['psi']] = self.cfg.Q.sigma_wz**2 * dt**2
        Q[i['bax'], i['bax']] = self.cfg.Q.q_bax * dt
        Q[i['bay'], i['bay']] = self.cfg.Q.q_bay * dt
        Q[i['bgz'], i['bgz']] = self.cfg.Q.q_bgz * dt
        Q[i['sws'], i['sws']] = self.cfg.Q.q_sws * dt
        Q[i['bdelta'], i['bdelta']] = self.cfg.Q.q_bdelta * dt
        return Q
