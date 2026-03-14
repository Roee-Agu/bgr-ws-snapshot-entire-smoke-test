from __future__ import annotations

import numpy as np


def compute_basic_nis_stats(nis_values: list[float]) -> dict[str, float]:
    if not nis_values:
        return {'count': 0, 'mean': float('nan'), 'max': float('nan')}
    arr = np.asarray(nis_values, dtype=float)
    return {'count': int(arr.size), 'mean': float(np.mean(arr)), 'max': float(np.max(arr))}
