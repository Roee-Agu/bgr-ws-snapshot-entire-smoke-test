from __future__ import annotations

import math

G0 = 9.80665


def deg_to_rad(value_deg: float) -> float:
    return value_deg * math.pi / 180.0


def accel_g_to_mps2(value_g: float) -> float:
    return value_g * G0
