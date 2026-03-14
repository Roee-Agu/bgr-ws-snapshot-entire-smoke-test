from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass(slots=True)
class MeasurementBundle:
    name: str
    z: np.ndarray
    h: np.ndarray
    H: np.ndarray
    R: np.ndarray
    gate: float
