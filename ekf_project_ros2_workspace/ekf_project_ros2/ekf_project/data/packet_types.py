from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(slots=True)
class ImuPacket:
    t: float
    ax: float
    ay: float
    wz: float
    source: str = 'unknown'
    seq: Optional[int] = None
    az: Optional[float] = None
    wx: Optional[float] = None
    wy: Optional[float] = None


@dataclass(slots=True)
class GpsPacket:
    t: float
    px: float
    py: float
    v_gps: Optional[float] = None
    pos_accuracy: Optional[float] = None
    spd_accuracy: Optional[float] = None
    sats: Optional[int] = None
    is_new: bool = True
    source: str = 'unknown'


@dataclass(slots=True)
class VehiclePacket:
    t: float
    delta: float
    rpm_fl: float
    rpm_fr: float
    rpm_rl: float
    rpm_rr: float
    rpm_used: Optional[float] = None
    v_rpm_nominal: Optional[float] = None
    source: str = 'unknown'
