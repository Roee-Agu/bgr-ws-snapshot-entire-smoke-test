from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, List, Tuple
import numpy as np


@dataclass(slots=True)
class MeasurementResult:
    accepted: bool
    reason: str
    innovation: Optional[np.ndarray] = None
    nis: Optional[float] = None
    R_eff: Optional[np.ndarray] = None


@dataclass(slots=True)
class FilterStepDiagnostics:
    propagated: bool = False
    updates: List[Tuple[str, MeasurementResult]] = field(default_factory=list)
