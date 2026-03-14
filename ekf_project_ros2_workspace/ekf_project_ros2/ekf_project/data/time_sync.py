from __future__ import annotations

import numpy as np


def estimate_time_offset(reference_signal: np.ndarray, delayed_signal: np.ndarray, dt: float) -> float:
    """Estimate lag by maximizing discrete cross-correlation.

    Returns offset in seconds such that delayed_signal shifted by this offset best
    matches reference_signal.
    """
    if reference_signal.ndim != 1 or delayed_signal.ndim != 1:
        raise ValueError('Signals must be 1D arrays.')
    n = min(len(reference_signal), len(delayed_signal))
    if n < 3:
        raise ValueError('Signals must contain at least 3 samples.')
    ref = reference_signal[:n] - np.mean(reference_signal[:n])
    sig = delayed_signal[:n] - np.mean(delayed_signal[:n])
    corr = np.correlate(ref, sig, mode='full')
    lag = int(np.argmax(corr) - (n - 1))
    return lag * dt
