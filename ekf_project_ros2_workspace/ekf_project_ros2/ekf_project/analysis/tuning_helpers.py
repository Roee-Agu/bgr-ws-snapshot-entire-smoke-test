from __future__ import annotations

import numpy as np


def static_bias_and_noise(signal: np.ndarray) -> dict[str, float]:
    signal = np.asarray(signal, dtype=float)
    return {
        'mean': float(np.mean(signal)),
        'std': float(np.std(signal, ddof=1)),
    }


def random_walk_q_from_window_means(signal: np.ndarray, fs_hz: float, window_s: float = 60.0) -> float:
    """Practical bias random-walk estimate used in the design discussion.

    q_b ~= Var(mean[k+1] - mean[k]) / T_w
    """
    signal = np.asarray(signal, dtype=float)
    m = int(round(window_s * fs_hz))
    if m <= 1 or signal.size < 2 * m:
        raise ValueError('Signal too short for chosen window.')
    K = signal.size // m
    means = np.array([np.mean(signal[k*m:(k+1)*m]) for k in range(K)], dtype=float)
    d = np.diff(means)
    return float(np.var(d, ddof=1) / window_s)
