from __future__ import annotations

import numpy as np

from ekf_project.config.filter_config import EKFConfig
from ekf_project.core.diagnostics import MeasurementResult, FilterStepDiagnostics
from ekf_project.core.initialization import build_initial_covariance, build_initial_state
from ekf_project.core import validation
from ekf_project.data.packet_types import ImuPacket, GpsPacket, VehiclePacket
from ekf_project.models.process_model import ProcessModel2D, ensure_symmetry, wrap_angle_pi
from ekf_project.models.gps_position_model import GPSPositionModel
from ekf_project.models.gps_speed_model import GPSSpeedModel
from ekf_project.models.wheel_speed_model import WheelSpeedModel
from ekf_project.models.nhc_model import NHCModel
from ekf_project.models.steering_constraint_model import SteeringConstraintModel
from ekf_project.models.measurement_base import MeasurementBundle


class VehicleEKF2D:
    def __init__(self, cfg: EKFConfig):
        self.cfg = cfg
        self.idx = {name: cfg.state_spec.idx(name) for name in cfg.state_spec.state_order}
        self.x = np.zeros((cfg.state_spec.size, 1), dtype=float)
        self.P = np.eye(cfg.state_spec.size, dtype=float)
        self.t: float | None = None
        self.initialized = False

        self.process = ProcessModel2D(cfg)
        self.gps_pos_model = GPSPositionModel(cfg)
        self.gps_speed_model = GPSSpeedModel(cfg)
        self.wheel_speed_model = WheelSpeedModel(cfg)
        self.nhc_model = NHCModel(cfg)
        self.steering_model = SteeringConstraintModel(cfg)

    def initialize(self, gps: GpsPacket | None = None) -> None:
        self.x = build_initial_state(self.cfg, gps)
        self.P = build_initial_covariance(self.cfg, gps)
        self.t = gps.t if gps is not None else None
        self.initialized = True

    def set_startup_biases(self, bax0: float, bay0: float, bgz0: float) -> None:
        self.x[self.idx['bax'], 0] = bax0
        self.x[self.idx['bay'], 0] = bay0
        self.x[self.idx['bgz'], 0] = bgz0

    def propagate(self, imu: ImuPacket) -> FilterStepDiagnostics:
        diag = FilterStepDiagnostics(propagated=False)
        if not self.initialized:
            raise RuntimeError('EKF must be initialized before propagation.')
        if self.t is None:
            self.t = imu.t
            return diag
        dt = imu.t - self.t
        if dt <= 0.0:
            return diag
        self.x, self.P = self.process.propagate(self.x, self.P, imu.ax, imu.ay, imu.wz, dt)
        self.t = imu.t
        diag.propagated = True
        return diag

    def handle_gps(self, gps: GpsPacket) -> FilterStepDiagnostics:
        diag = FilterStepDiagnostics()
        if not self.initialized:
            self.initialize(gps)
            return diag

        if self.cfg.enable.use_gps_pos:
            ok, reason = validation.check_gps_position_valid(self.cfg, gps)
            if ok:
                res = self._apply_update(self.gps_pos_model.build(self.x, gps))
            else:
                res = MeasurementResult(False, f'gps_pos: {reason}')
            diag.updates.append(('gps_pos', res))

        if self.cfg.enable.use_gps_speed:
            ok, reason = validation.check_gps_speed_valid(self.cfg, gps)
            if ok:
                res = self._apply_update(self.gps_speed_model.build(self.x, gps))
            else:
                res = MeasurementResult(False, f'gps_speed: {reason}')
            diag.updates.append(('gps_speed', res))
        return diag

    def handle_vehicle(self, veh: VehiclePacket, last_imu: ImuPacket | None = None) -> FilterStepDiagnostics:
        diag = FilterStepDiagnostics()
        if not self.initialized:
            return diag

        ok, reason = validation.check_vehicle_packet_valid(self.cfg, veh)
        if not ok:
            diag.updates.append(('vehicle_packet', MeasurementResult(False, f'vehicle_packet: {reason}')))
            return diag

        if self.cfg.enable.use_wheel_speed:
            if veh.v_rpm_nominal is None:
                res = MeasurementResult(False, 'wheel_speed: missing v_rpm_nominal')
            else:
                res = self._apply_update(self.wheel_speed_model.build(self.x, veh))
            diag.updates.append(('wheel_speed', res))

        if self.cfg.enable.use_nhc:
            ok, reason = validation.check_nhc_regime(self.cfg, self.x, self.idx)
            if ok:
                res = self._apply_update(self.nhc_model.build(self.x, veh, last_imu))
            else:
                res = MeasurementResult(False, f'nhc: {reason}')
            diag.updates.append(('nhc', res))

        if self.cfg.enable.use_steering_constraint:
            ok, reason = validation.check_steering_regime(self.cfg, self.x, self.idx, veh, last_imu)
            if ok and last_imu is not None:
                res = self._apply_update(self.steering_model.build(self.x, veh, last_imu))
            else:
                res = MeasurementResult(False, f'steering_constraint: {reason}')
            diag.updates.append(('steering_constraint', res))
        return diag

    def _apply_update(self, bundle: MeasurementBundle) -> MeasurementResult:
        z, h, H, R = bundle.z, bundle.h, bundle.H, bundle.R
        r = z - h
        S = H @ self.P @ H.T + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return MeasurementResult(False, f'{bundle.name}: singular innovation covariance', innovation=r, R_eff=R)
        nis = float(r.T @ S_inv @ r)
        if nis > bundle.gate:
            return MeasurementResult(False, f'{bundle.name}: innovation gate fail', innovation=r, nis=nis, R_eff=R)
        K = self.P @ H.T @ S_inv
        self.x = self.x + K @ r
        self.x[self.idx['psi'], 0] = wrap_angle_pi(float(self.x[self.idx['psi'], 0]))
        I = np.eye(self.cfg.state_spec.size)
        self.P = ensure_symmetry((I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T)
        return MeasurementResult(True, f'{bundle.name}: accepted', innovation=r, nis=nis, R_eff=R)
