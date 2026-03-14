from __future__ import annotations

import math
from typing import Tuple


def equirectangular_local_xy(lat_deg: float, lon_deg: float, lat0_deg: float, lon0_deg: float) -> Tuple[float, float]:
    """Small-area lat/lon to local ENU approximation in meters."""
    r_earth = 6378137.0
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    lat0 = math.radians(lat0_deg)
    lon0 = math.radians(lon0_deg)
    x = (lon - lon0) * math.cos(lat0) * r_earth
    y = (lat - lat0) * r_earth
    return x, y
