from __future__ import annotations

"""ROS2 simulation runner for the EKF workspace.

Subscribes to the four simulator topics published by ekf_publisher.py:
    - /robot/datalogger_gt
    - /robot/datalogger_noisy
    - /robot/mcu_gt
    - /robot/mcu_noisy

Uses noisy topics for estimation and GT topics only for diagnostics.
Default validation phase is IMU + GPS position only.
"""

import math
from typing import Optional

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray

from ekf_project.config.filter_config import EKFConfig
from ekf_project.core.ekf_core import VehicleEKF2D
from ekf_project.core.initialization import StartupInitializer
from ekf_project.data.adapters import PacketAdapter
from ekf_project.data.packet_types import GpsPacket, ImuPacket, VehiclePacket


class SimulationEkfNode(Node):
    def __init__(self) -> None:
        super().__init__('ekf_simulation_runner')

        self.cfg = EKFConfig()

        try:
            if not self.has_parameter('use_sim_time'):
                self.declare_parameter('use_sim_time', True)
        except Exception:
            pass

        # Phase-1 defaults: IMU + GPS position only.
        self.cfg.enable.use_gps_pos = True
        self.cfg.enable.use_gps_speed = False
        self.cfg.enable.use_wheel_speed = False
        self.cfg.enable.use_nhc = False
        self.cfg.enable.use_steering_constraint = False

        self.adapter = PacketAdapter(self.cfg)
        self.ekf = VehicleEKF2D(self.cfg)
        self.startup = StartupInitializer(self.cfg)

        self.last_noisy_imu: Optional[ImuPacket] = None
        self.last_gt_gps: Optional[GpsPacket] = None
        self.last_gt_vehicle: Optional[VehiclePacket] = None
        self.startup_biases_applied = False
        self._gps_accept_count = 0
        self._gps_reject_count = 0
        self._last_gps_reason = 'none'
        self._last_gps_nis = None

        # Lightweight rate diagnostics.
        self._last_rate_wall_t = self._wall_time_s()
        self._count_datalogger_noisy = 0
        self._count_mcu_noisy = 0
        self._count_odom_pub = 0

        qos = self.cfg.ros.qos_depth

        self.create_subscription(
            Float64MultiArray,
            self.cfg.ros.datalogger_gt_topic,
            self._on_datalogger_gt,
            qos,
        )
        self.create_subscription(
            Float64MultiArray,
            self.cfg.ros.datalogger_noisy_topic,
            self._on_datalogger_noisy,
            qos,
        )
        self.create_subscription(
            Float64MultiArray,
            self.cfg.ros.mcu_gt_topic,
            self._on_mcu_gt,
            qos,
        )
        self.create_subscription(
            Float64MultiArray,
            self.cfg.ros.mcu_noisy_topic,
            self._on_mcu_noisy,
            qos,
        )

        self.ekf_state_pub = self.create_publisher(
            Float64MultiArray, self.cfg.ros.ekf_state_topic, qos
        )
        self.ekf_diag_pub = self.create_publisher(
            Float64MultiArray, self.cfg.ros.ekf_diag_topic, qos
        )

        # Estimated odom is only for diagnostics/plotting in this phase, so use a
        # sensor-style best-effort QoS to reduce overhead on a loaded simulator VM.
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.estimated_odom_pub = self.create_publisher(
            Odometry, self.cfg.ros.estimated_odom_topic, odom_qos
        )

        self.create_timer(self.cfg.ros.status_period_s, self._publish_status)
        self.create_timer(2.0, self._publish_rate_status)

        self.get_logger().info(
            'Simulation EKF node started. Default phase: IMU + GPS position only.'
        )

    def _wall_time_s(self) -> float:
        import time
        return time.perf_counter()

    def _stamp_now(self) -> float:
        # Simulator payload currently has no source timestamps, so receipt time is used.
        return self.get_clock().now().nanoseconds * 1e-9

    def _odom_stamp_now(self) -> Time:
        return self.get_clock().now().to_msg()

    def _on_datalogger_gt(self, msg: Float64MultiArray) -> None:
        t = self._stamp_now()
        _imu, gps = self.adapter.normalize_sim_datalogger(
            t, list(msg.data), stream_key='sim_gt'
        )
        self.last_gt_gps = gps

    def _on_datalogger_noisy(self, msg: Float64MultiArray) -> None:
        self._count_datalogger_noisy += 1

        t = self._stamp_now()
        imu, gps = self.adapter.normalize_sim_datalogger(
            t, list(msg.data), stream_key='sim_noisy'
        )

        self.last_noisy_imu = imu
        self.startup.push_imu(imu)

        if not self.ekf.initialized and gps.is_new:
            self.ekf.initialize(gps)
            self.get_logger().info(
                'EKF initialized from first fresh noisy GPS sample.'
            )

        if self.ekf.initialized:
            self.ekf.propagate(imu)

            if gps.is_new:
                gps_diag = self.ekf.handle_gps(gps)

                for name, res in gps_diag.updates:
                    if name == 'gps_pos':
                        self._last_gps_reason = res.reason
                        self._last_gps_nis = res.nis

                        if res.accepted:
                            self._gps_accept_count += 1
                            self.get_logger().info(
                                f"GPS_POS accepted: "
                                f"z=({gps.px:.2f}, {gps.py:.2f}), "
                                f"nis={res.nis if res.nis is not None else 'None'}"
                            )
                        else:
                            self._gps_reject_count += 1
                            self.get_logger().warn(
                                f"GPS_POS rejected: "
                                f"z=({gps.px:.2f}, {gps.py:.2f}), "
                                f"reason='{res.reason}', "
                                f"nis={res.nis if res.nis is not None else 'None'}"
                            )

            self._maybe_apply_startup_biases()
            self._publish_state()

    def _on_mcu_gt(self, msg: Float64MultiArray) -> None:
        t = self._stamp_now()
        self.last_gt_vehicle = self.adapter.normalize_sim_mcu(
            t, list(msg.data), stream_key='sim_gt'
        )

    def _on_mcu_noisy(self, msg: Float64MultiArray) -> None:
        self._count_mcu_noisy += 1

        if not self.ekf.initialized:
            return

        # For the phase-1 smoke test, vehicle aiding is disabled.
        # In that case, avoid duplicate odom republishes on every MCU packet.
        if not (
            self.cfg.enable.use_wheel_speed
            or self.cfg.enable.use_nhc
            or self.cfg.enable.use_steering_constraint
        ):
            return

        t = self._stamp_now()
        veh = self.adapter.normalize_sim_mcu(
            t, list(msg.data), stream_key='sim_noisy'
        )
        self.ekf.handle_vehicle(veh, self.last_noisy_imu)
        self._publish_state()

    def _maybe_apply_startup_biases(self) -> None:
        if self.startup_biases_applied or not self.startup.ready():
            return

        est = self.startup.estimate_biases()
        self.ekf.set_startup_biases(
            est['bax0'],
            est['bay0'],
            est['bgz0'],
        )
        self.startup_biases_applied = True

        self.get_logger().info(
            "\n"
            "================ EKF STARTUP BIASES APPLIED ================\n\n\n"
            f"bax = {est['bax0']:.4f} m/s^2\n"
            f"bay = {est['bay0']:.4f} m/s^2\n"
            f"bgz = {math.degrees(est['bgz0']):.4f} deg/s\n"
            "=============================================================\n\n\n"
        )

    def _yaw_to_quaternion(self, psi: float) -> Quaternion:
        half = 0.5 * psi
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(half)
        q.w = math.cos(half)
        return q

    def _publish_state(self) -> None:
        if not self.ekf.initialized:
            return

        state_msg = Float64MultiArray()
        state_msg.data = [float(v) for v in self.ekf.x[:, 0]]
        self.ekf_state_pub.publish(state_msg)

        x = self.ekf.x[:, 0]

        odom = Odometry()
        odom.header.stamp = self._odom_stamp_now()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = float(x[0])
        odom.pose.pose.position.y = float(x[1])
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self._yaw_to_quaternion(float(x[2]))

        odom.twist.twist.linear.x = float(x[3])
        odom.twist.twist.linear.y = float(x[4])

        if self.last_noisy_imu is not None:
            odom.twist.twist.angular.z = float(self.last_noisy_imu.wz - x[7])
        else:
            odom.twist.twist.angular.z = 0.0

        self.estimated_odom_pub.publish(odom)
        self._count_odom_pub += 1

    def _publish_status(self) -> None:
        if not self.ekf.initialized:
            self.get_logger().info('EKF waiting for first fresh GPS sample...')
            return

        x = self.ekf.x[:, 0]
        px, py, psi, vx, vy = x[0], x[1], x[2], x[3], x[4]
        speed = math.hypot(vx, vy)

        diag_values = [px, py, psi, speed]

        gps_total = self._gps_accept_count + self._gps_reject_count
        gps_ratio = (
            self._gps_accept_count / gps_total if gps_total > 0 else 0.0
        )

        if self.last_gt_gps is not None:
            ex = px - self.last_gt_gps.px
            ey = py - self.last_gt_gps.py
            epos = math.hypot(ex, ey)
            diag_values.extend([ex, ey, epos])

            self.get_logger().info(
                f'[EKF STATUS] '
                f'px={px:.2f}, py={py:.2f}, psi={math.degrees(psi):.2f} deg, '
                f'v={speed:.2f} m/s, pos_err={epos:.2f} m, '
                f'gps_acc={self._gps_accept_count}, gps_rej={self._gps_reject_count}, '
                f'gps_acc_ratio={gps_ratio:.2f}, '
                f'last_gps_reason={self._last_gps_reason}, '
                f'last_gps_nis={self._last_gps_nis}'
            )
        else:
            self.get_logger().info(
                f'[EKF STATUS] '
                f'px={px:.2f}, py={py:.2f}, psi={math.degrees(psi):.2f} deg, '
                f'v={speed:.2f} m/s, '
                f'gps_acc={self._gps_accept_count}, gps_rej={self._gps_reject_count}, '
                f'gps_acc_ratio={gps_ratio:.2f}, '
                f'last_gps_reason={self._last_gps_reason}, '
                f'last_gps_nis={self._last_gps_nis}'
            )

        diag_msg = Float64MultiArray()
        diag_msg.data = diag_values
        self.ekf_diag_pub.publish(diag_msg)

    def _publish_rate_status(self) -> None:
        now = self._wall_time_s()
        dt = now - self._last_rate_wall_t
        if dt <= 0.0:
            return

        datalogger_hz = self._count_datalogger_noisy / dt
        mcu_hz = self._count_mcu_noisy / dt
        odom_hz = self._count_odom_pub / dt

        self.get_logger().info(
            f'wall rates: datalogger_noisy={datalogger_hz:.1f} Hz, '
            f'mcu_noisy={mcu_hz:.1f} Hz, odom_pub={odom_hz:.1f} Hz'
        )

        self._last_rate_wall_t = now
        self._count_datalogger_noisy = 0
        self._count_mcu_noisy = 0
        self._count_odom_pub = 0


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimulationEkfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
