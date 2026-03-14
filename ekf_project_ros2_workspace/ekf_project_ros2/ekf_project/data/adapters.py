from __future__ import annotations

from typing import List, Tuple, Dict

from ekf_project.config.filter_config import EKFConfig, WheelSelector
from ekf_project.data.coordinate_utils import equirectangular_local_xy
from ekf_project.data.packet_types import ImuPacket, GpsPacket, VehiclePacket
from ekf_project.data.unit_conversions import accel_g_to_mps2, deg_to_rad


def select_rpm(pkt: VehiclePacket, selector: WheelSelector) -> float:
    if selector == WheelSelector.REAR_AVG:
        return 0.5 * (pkt.rpm_rl + pkt.rpm_rr)
    if selector == WheelSelector.FRONT_AVG:
        return 0.5 * (pkt.rpm_fl + pkt.rpm_fr)
    if selector == WheelSelector.ALL_AVG:
        return 0.25 * (pkt.rpm_fl + pkt.rpm_fr + pkt.rpm_rl + pkt.rpm_rr)
    if selector == WheelSelector.RL_ONLY:
        return pkt.rpm_rl
    if selector == WheelSelector.RR_ONLY:
        return pkt.rpm_rr
    if selector == WheelSelector.FL_ONLY:
        return pkt.rpm_fl
    if selector == WheelSelector.FR_ONLY:
        return pkt.rpm_fr
    raise ValueError(f'Unknown selector: {selector}')


class PacketAdapter:
    def __init__(self, cfg: EKFConfig):
        self.cfg = cfg
        self._last_gps_by_stream: Dict[str, Tuple[float, float, float | None]] = {}

    def normalize_real_imu(self, t: float, inline_acc: float, lateral_acc: float, yaw_rate: float) -> ImuPacket:
        ax = self.cfg.adapter.accel_x_sign * inline_acc
        ay = self.cfg.adapter.accel_y_sign * lateral_acc
        wz = self.cfg.adapter.yaw_rate_sign * yaw_rate
        if self.cfg.adapter.real_accel_in_g:
            ax = accel_g_to_mps2(ax)
            ay = accel_g_to_mps2(ay)
        if self.cfg.adapter.real_gyro_in_deg_s:
            wz = deg_to_rad(wz)
        return ImuPacket(t=t, ax=ax, ay=ay, wz=wz, source='real')

    def normalize_real_gps(
        self,
        t: float,
        lat_deg: float,
        lon_deg: float,
        gps_speed: float | None = None,
        pos_accuracy: float | None = None,
        spd_accuracy: float | None = None,
        sats: int | None = None,
    ) -> GpsPacket:
        if self.cfg.adapter.lat0_deg is None or self.cfg.adapter.lon0_deg is None:
            raise ValueError('Adapter lat0/lon0 must be set before converting real GPS.')
        px, py = equirectangular_local_xy(lat_deg, lon_deg, self.cfg.adapter.lat0_deg, self.cfg.adapter.lon0_deg)
        return GpsPacket(
            t=t,
            px=px,
            py=py,
            v_gps=gps_speed,
            pos_accuracy=pos_accuracy,
            spd_accuracy=spd_accuracy,
            sats=sats,
            is_new=True,
            source='real',
        )

    def normalize_real_vehicle(self, t: float, steering: float, rpm: float) -> VehiclePacket:
        delta = self.cfg.adapter.steering_sign * steering
        if self.cfg.adapter.real_steering_in_deg:
            delta = deg_to_rad(delta)
        signed_rpm = self.cfg.adapter.rpm_rl_sign * rpm
        pkt = VehiclePacket(t=t, delta=delta, rpm_fl=signed_rpm, rpm_fr=signed_rpm, rpm_rl=signed_rpm, rpm_rr=signed_rpm, source='real')
        pkt.rpm_used = select_rpm(pkt, self.cfg.vehicle.wheel_selector)
        pkt.v_rpm_nominal = self.cfg.vehicle.rpm_to_speed_gain_Krpm * pkt.rpm_used
        return pkt

    def normalize_sim_datalogger(self, t: float, fields: List[float], stream_key: str = 'sim') -> Tuple[ImuPacket, GpsPacket]:
        if len(fields) < 10:
            raise ValueError('Simulator datalogger packet must contain 10 fields.')
        abs_speed, pos_x, pos_y, _pos_z, acc_x, acc_y, _acc_z, _roll, _pitch, yaw_rate = fields[:10]
        imu = ImuPacket(t=t, ax=acc_x, ay=acc_y, wz=yaw_rate, source=stream_key)
        current = (float(pos_x), float(pos_y), float(abs_speed))
        is_new = self._last_gps_by_stream.get(stream_key) != current
        self._last_gps_by_stream[stream_key] = current
        gps = GpsPacket(t=t, px=float(pos_x), py=float(pos_y), v_gps=float(abs_speed), is_new=is_new, source=stream_key)
        return imu, gps

    def normalize_sim_mcu(self, t: float, fields: List[float], stream_key: str = 'sim') -> VehiclePacket:
        if len(fields) < 5:
            raise ValueError('Simulator MCU packet must contain 5 fields.')
        delta, rpm_fl, rpm_fr, rpm_rl, rpm_rr = fields[:5]
        pkt = VehiclePacket(
            t=t,
            delta=float(delta),
            rpm_fl=float(rpm_fl),
            rpm_fr=float(rpm_fr),
            rpm_rl=float(rpm_rl),
            rpm_rr=float(rpm_rr),
            source=stream_key,
        )
        pkt.rpm_used = select_rpm(pkt, self.cfg.vehicle.wheel_selector)
        pkt.v_rpm_nominal = self.cfg.vehicle.rpm_to_speed_gain_Krpm * pkt.rpm_used
        return pkt
