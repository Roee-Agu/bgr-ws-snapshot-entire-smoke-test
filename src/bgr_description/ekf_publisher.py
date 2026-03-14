#!/usr/bin/env python3

"""
EKF publisher signal conventions
================================

Published topics
----------------
/robot/datalogger_gt
/robot/datalogger_noisy
  Layout:
    [0] abs_speed    [m/s]
    [1] pos_x        [m]
    [2] pos_y        [m]
    [3] pos_z        [m]
    [4] acc_x        [m/s^2]
    [5] acc_y        [m/s^2]
    [6] acc_z        [m/s^2]
    [7] roll_rate    [rad/s]
    [8] pitch_rate   [rad/s]
    [9] yaw_rate     [rad/s]

/robot/mcu_gt
/robot/mcu_noisy
  Layout:
    [0] steering_angle [rad]
    [1] rpm_fl         [rpm]
    [2] rpm_fr         [rpm]
    [3] rpm_rl         [rpm]
    [4] rpm_rr         [rpm]

Sign conventions
----------------
pos_x:
  Positive when driving forward from the nominal spawn direction.

pos_y:
  Positive according to the simulator map Y direction.
  At the default spawn used in current tests, forward motion primarily changes pos_x.

pos_z:
  Positive upward.
  Note: when the car is standing on the ground, pos_z may be around 0.48 m
  because the published reference point is above the ground plane.

abs_speed:
  Always nonnegative. This is speed magnitude, not signed longitudinal velocity.

acc_x:
  Positive for forward acceleration.

acc_y:
  Positive for lateral acceleration to the left.

acc_z:
  Positive upward.

roll_rate:
  Positive by simulator right-hand-rule convention.

pitch_rate:
  Positive by simulator right-hand-rule convention.

yaw_rate:
  Positive for left turn, negative for right turn.

steering_angle:
  Positive for left steer, negative for right steer.

rpm_fl / rpm_fr / rpm_rl / rpm_rr:
  Positive for forward wheel rotation, negative for reverse.

Timing model
------------
The publisher runs at 50 Hz.
The datalogger topic is published at 50 Hz.
GPS-like fields [abs_speed, pos_x, pos_y, pos_z] are refreshed every 5 ticks
(10 Hz effective) and held constant between updates.
IMU-like fields are refreshed every tick (50 Hz).
MCU fields are refreshed every tick (50 Hz).

Noise / bias model
------------------
GPS:
  abs_speed, pos_x, pos_y, pos_z use additive white measurement noise.

IMU:
  acc_x_meas   = acc_x_true   + b_ax + white_noise
  acc_y_meas   = acc_y_true   + b_ay + white_noise
  yaw_rate_meas= yaw_rate_true+ b_gz + white_noise

  Biases evolve as random walks:
    b_ax(k+1) = b_ax(k) + w_bax
    b_ay(k+1) = b_ay(k) + w_bay
    b_gz(k+1) = b_gz(k) + w_bgz

MCU:
  steering_meas = steering_true + b_delta + white_noise
  rpm_meas      = (1 + s_ws) * rpm_true + white_noise
"""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class EkfPublisher(Node):
    RAD_S_TO_RPM = 9.549296596425384

    def __init__(self):
        super().__init__("ekf_publisher")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # =========================
        # Timing
        # =========================
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("gps_update_every_n_ticks", 5)   # 50 / 5 = 10 Hz
        self.declare_parameter("enable_noise", True)

        # =========================
        # Sign conventions
        # Use these if the simulator sign does not match your EKF convention
        # =========================
        self.declare_parameter("steering_sign", 1.0)
        self.declare_parameter("yaw_rate_sign", 1.0)
        self.declare_parameter("rpm_fl_sign", 1.0)
        self.declare_parameter("rpm_fr_sign", 1.0)
        self.declare_parameter("rpm_rl_sign", 1.0)
        self.declare_parameter("rpm_rr_sign", 1.0)

        # =========================
        # GPS measurement noise
        # =========================
        self.declare_parameter("gps_speed_noise_std", 0.05)   # m/s
        self.declare_parameter("gps_pos_x_noise_std", 0.20)   # m
        self.declare_parameter("gps_pos_y_noise_std", 0.20)   # m
        self.declare_parameter("gps_pos_z_noise_std", 0.10)   # m

        # =========================
        # IMU initial biases
        # =========================
        self.declare_parameter("acc_x_bias_initial", 0.0)     # m/s^2
        self.declare_parameter("acc_y_bias_initial", 0.0)     # m/s^2
        self.declare_parameter("gyro_z_bias_initial", 0.0)    # rad/s

        # =========================
        # IMU white noise (per sample)
        # =========================
        self.declare_parameter("acc_x_noise_std", 0.15)       # m/s^2
        self.declare_parameter("acc_y_noise_std", 0.15)       # m/s^2
        self.declare_parameter("acc_z_noise_std", 0.15)       # m/s^2
        self.declare_parameter("roll_rate_noise_std", 0.01)   # rad/s
        self.declare_parameter("pitch_rate_noise_std", 0.01)  # rad/s
        self.declare_parameter("gyro_z_noise_std", 0.01)      # rad/s

        # =========================
        # IMU bias random walk std
        # Interpreted as sigma * sqrt(dt)
        # =========================
        self.declare_parameter("acc_x_bias_rw_std", 0.001)
        self.declare_parameter("acc_y_bias_rw_std", 0.001)
        self.declare_parameter("gyro_z_bias_rw_std", 0.0005)

        # =========================
        # MCU model
        # measured_steering = true_steering + steering_bias + white_noise
        # measured_rpm = (1 + wheel_speed_scale) * true_rpm + white_noise
        # =========================
        self.declare_parameter("steering_bias", 0.0)          # rad
        self.declare_parameter("steering_noise_std", 0.005)   # rad
        self.declare_parameter("wheel_speed_scale", 0.0)      # dimensionless
        self.declare_parameter("rpm_fl_noise_std", 3.0)       # rpm
        self.declare_parameter("rpm_fr_noise_std", 3.0)       # rpm
        self.declare_parameter("rpm_rl_noise_std", 3.0)       # rpm
        self.declare_parameter("rpm_rr_noise_std", 3.0)       # rpm

        self.odom_sub = self.create_subscription(
            Odometry, "/model/bgr/odometry", self.odom_callback, qos
        )
        self.joint_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_callback, qos
        )

        self.pub_datalogger_gt = self.create_publisher(
            Float64MultiArray, "/robot/datalogger_gt", 10
        )
        self.pub_datalogger_noisy = self.create_publisher(
            Float64MultiArray, "/robot/datalogger_noisy", 10
        )
        self.pub_mcu_gt = self.create_publisher(
            Float64MultiArray, "/robot/mcu_gt", 10
        )
        self.pub_mcu_noisy = self.create_publisher(
            Float64MultiArray, "/robot/mcu_noisy", 10
        )

        self.steering_joint = "Steering_fl_joint"
        self.wheel_joints = [
            "Wheel_fl_joint",
            "Wheel_fr_joint",
            "Wheel_rl_joint",
            "Wheel_rr_joint",
        ]

        self.have_odom = False
        self.have_joint = False

        # Latest ground truth
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0

        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_z = 0.0

        self.acc_x = 0.0
        self.acc_y = 0.0
        self.acc_z = 0.0

        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate = 0.0

        self.steering_angle = 0.0
        self.rpms = [0.0, 0.0, 0.0, 0.0]

        # For acceleration estimation
        self.last_odom_time = None
        self.last_vel_x = 0.0
        self.last_vel_y = 0.0
        self.last_vel_z = 0.0

        # GPS sample-and-hold caches
        self.gps_gt_cache = [0.0, 0.0, 0.0, 0.0]
        self.gps_noisy_cache = [0.0, 0.0, 0.0, 0.0]

        # Persistent sensor biases
        self.bias_acc_x = self.get_parameter("acc_x_bias_initial").value
        self.bias_acc_y = self.get_parameter("acc_y_bias_initial").value
        self.bias_gyro_z = self.get_parameter("gyro_z_bias_initial").value

        self.tick = 0

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.dt_nominal = 1.0 / rate
        self.timer = self.create_timer(self.dt_nominal, self.publish_callback)

        self.get_logger().info(
            "ekf_publisher started: /robot/datalogger_gt, /robot/datalogger_noisy, "
            "/robot/mcu_gt, /robot/mcu_noisy"
        )

    def odom_callback(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.pos_z = msg.pose.pose.position.z

        self.vel_x = msg.twist.twist.linear.x
        self.vel_y = msg.twist.twist.linear.y
        self.vel_z = msg.twist.twist.linear.z

        self.roll_rate = msg.twist.twist.angular.x
        self.pitch_rate = msg.twist.twist.angular.y
        self.yaw_rate = msg.twist.twist.angular.z

        self.acc_x = 0.0
        self.acc_y = 0.0
        self.acc_z = 0.0

        if self.last_odom_time is not None:
            dt = t - self.last_odom_time
            if dt > 0.0:
                self.acc_x = (self.vel_x - self.last_vel_x) / dt
                self.acc_y = (self.vel_y - self.last_vel_y) / dt
                self.acc_z = (self.vel_z - self.last_vel_z) / dt

        self.last_odom_time = t
        self.last_vel_x = self.vel_x
        self.last_vel_y = self.vel_y
        self.last_vel_z = self.vel_z

        self.have_odom = True

    def joint_callback(self, msg: JointState):
        try:
            if self.steering_joint in msg.name:
                idx = msg.name.index(self.steering_joint)
                self.steering_angle = msg.position[idx]

            rpms = [0.0, 0.0, 0.0, 0.0]
            for i, joint_name in enumerate(self.wheel_joints):
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    vel_rad_s = msg.velocity[idx]
                    rpms[i] = vel_rad_s * self.RAD_S_TO_RPM

            self.rpms = rpms
            self.have_joint = True

        except Exception as e:
            self.get_logger().warn(f"Joint parsing error: {e}")

    def gaussian(self, std_dev: float) -> float:
        return random.gauss(0.0, float(std_dev))

    def update_bias_random_walk(self, dt: float):
        self.bias_acc_x += self.gaussian(
            self.get_parameter("acc_x_bias_rw_std").value * math.sqrt(dt)
        )
        self.bias_acc_y += self.gaussian(
            self.get_parameter("acc_y_bias_rw_std").value * math.sqrt(dt)
        )
        self.bias_gyro_z += self.gaussian(
            self.get_parameter("gyro_z_bias_rw_std").value * math.sqrt(dt)
        )

    def publish_callback(self):
        if not (self.have_odom and self.have_joint):
            return

        self.tick += 1

        gps_update_every = int(self.get_parameter("gps_update_every_n_ticks").value)
        enable_noise = bool(self.get_parameter("enable_noise").value)

        steering_sign = float(self.get_parameter("steering_sign").value)
        yaw_rate_sign = float(self.get_parameter("yaw_rate_sign").value)
        rpm_signs = [
            float(self.get_parameter("rpm_fl_sign").value),
            float(self.get_parameter("rpm_fr_sign").value),
            float(self.get_parameter("rpm_rl_sign").value),
            float(self.get_parameter("rpm_rr_sign").value),
        ]

        # ===== Ground truth after sign convention mapping =====
        abs_speed = math.sqrt(self.vel_x ** 2 + self.vel_y ** 2)

        gps_gt = [
            abs_speed,
            self.pos_x,
            self.pos_y,
            self.pos_z,
        ]

        imu_gt = [
            self.acc_x,
            self.acc_y,
            self.acc_z,
            self.roll_rate,
            self.pitch_rate,
            yaw_rate_sign * self.yaw_rate,
        ]

        mcu_gt = [
            steering_sign * self.steering_angle,
            rpm_signs[0] * self.rpms[0],
            rpm_signs[1] * self.rpms[1],
            rpm_signs[2] * self.rpms[2],
            rpm_signs[3] * self.rpms[3],
        ]

        # ===== GPS sample-and-hold at 10 Hz =====
        if self.tick % gps_update_every == 1:
            self.gps_gt_cache = list(gps_gt)

            if enable_noise:
                self.gps_noisy_cache = [
                    gps_gt[0] + self.gaussian(self.get_parameter("gps_speed_noise_std").value),
                    gps_gt[1] + self.gaussian(self.get_parameter("gps_pos_x_noise_std").value),
                    gps_gt[2] + self.gaussian(self.get_parameter("gps_pos_y_noise_std").value),
                    gps_gt[3] + self.gaussian(self.get_parameter("gps_pos_z_noise_std").value),
                ]
            else:
                self.gps_noisy_cache = list(gps_gt)

        # ===== Bias evolution =====
        if enable_noise:
            self.update_bias_random_walk(self.dt_nominal)

        # ===== IMU noisy measurement =====
        if enable_noise:
            imu_noisy = [
                imu_gt[0] + self.bias_acc_x + self.gaussian(self.get_parameter("acc_x_noise_std").value),
                imu_gt[1] + self.bias_acc_y + self.gaussian(self.get_parameter("acc_y_noise_std").value),
                imu_gt[2] + self.gaussian(self.get_parameter("acc_z_noise_std").value),
                imu_gt[3] + self.gaussian(self.get_parameter("roll_rate_noise_std").value),
                imu_gt[4] + self.gaussian(self.get_parameter("pitch_rate_noise_std").value),
                imu_gt[5] + self.bias_gyro_z + self.gaussian(self.get_parameter("gyro_z_noise_std").value),
            ]
        else:
            imu_noisy = list(imu_gt)

        # ===== MCU noisy measurement =====
        wheel_speed_scale = float(self.get_parameter("wheel_speed_scale").value)
        steering_bias = float(self.get_parameter("steering_bias").value)

        if enable_noise:
            mcu_noisy = [
                mcu_gt[0] + steering_bias + self.gaussian(self.get_parameter("steering_noise_std").value),
                (1.0 + wheel_speed_scale) * mcu_gt[1] + self.gaussian(self.get_parameter("rpm_fl_noise_std").value),
                (1.0 + wheel_speed_scale) * mcu_gt[2] + self.gaussian(self.get_parameter("rpm_fr_noise_std").value),
                (1.0 + wheel_speed_scale) * mcu_gt[3] + self.gaussian(self.get_parameter("rpm_rl_noise_std").value),
                (1.0 + wheel_speed_scale) * mcu_gt[4] + self.gaussian(self.get_parameter("rpm_rr_noise_std").value),
            ]
        else:
            mcu_noisy = list(mcu_gt)

        datalogger_gt = self.gps_gt_cache + imu_gt
        datalogger_noisy = self.gps_noisy_cache + imu_noisy

        msg = Float64MultiArray()
        msg.data = datalogger_gt
        self.pub_datalogger_gt.publish(msg)

        msg = Float64MultiArray()
        msg.data = datalogger_noisy
        self.pub_datalogger_noisy.publish(msg)

        msg = Float64MultiArray()
        msg.data = mcu_gt
        self.pub_mcu_gt.publish(msg)

        msg = Float64MultiArray()
        msg.data = mcu_noisy
        self.pub_mcu_noisy.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EkfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()