from __future__ import annotations

"""Minimal replay runner placeholder.

Intended role:
- read normalized or raw logs
- pass them through adapters if needed
- propagate on IMU packets
- update on GPS / vehicle packets
- collect diagnostics for analysis
"""

from ekf_project.config.filter_config import EKFConfig
from ekf_project.core.ekf_core import VehicleEKF2D
from ekf_project.data.adapters import PacketAdapter


def main() -> None:
    cfg = EKFConfig()
    ekf = VehicleEKF2D(cfg)
    _adapter = PacketAdapter(cfg)
    print('Log replay runner placeholder. Wire your CSV/ROS2 replay here.')


if __name__ == '__main__':
    main()
