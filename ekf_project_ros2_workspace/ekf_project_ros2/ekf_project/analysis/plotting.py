from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np


def plot_xy(traj_est: np.ndarray, traj_ref: np.ndarray | None = None) -> None:
    plt.figure()
    plt.plot(traj_est[:, 0], traj_est[:, 1], label='EKF')
    if traj_ref is not None:
        plt.plot(traj_ref[:, 0], traj_ref[:, 1], label='Reference')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.axis('equal')
    plt.legend()
    plt.title('Trajectory')
    plt.show()
