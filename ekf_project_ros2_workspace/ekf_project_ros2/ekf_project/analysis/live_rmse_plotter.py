#!/usr/bin/env python3
import math
import os
from collections import deque

import matplotlib.pyplot as plt
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


class RealTimePlotterSynced(Node):
    def __init__(self):
        super().__init__('ekf_plotter_synced')

        try:
            if not self.has_parameter('use_sim_time'):
                self.declare_parameter('use_sim_time', True)
        except Exception:
            pass

        self.get_logger().info(
            "Plotter started: /robot/estimated_odom vs /model/bgr/odometry"
        )
        self.get_logger().info(
            f"Matplotlib backend={plt.get_backend()}, DISPLAY={os.environ.get('DISPLAY', '<unset>')}"
        )

        # --- data history ---
        self.est_x, self.est_y = [], []
        self.gt_x, self.gt_y = [], []

        # --- error accumulators ---
        self.sum_sq_x = 0.0
        self.sum_sq_y = 0.0
        self.sum_sq_2d = 0.0
        self.count = 0

        # --- instantaneous errors ---
        self.inst_dx = 0.0
        self.inst_dy = 0.0
        self.inst_e2d = 0.0
        self.inst_dt = 0.0

        # --- plotting ---
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 7))
        try:
            self.fig.canvas.manager.set_window_title(
                'BGR Localization Performance Monitor'
            )
        except Exception:
            pass

        # Explicitly show the window. This helps on VMs/X11 setups.
        plt.show(block=False)
        plt.pause(0.001)

        # refresh plot at 10 Hz
        self.create_timer(0.1, self.update_plot)

        # --- manual sync only ---
        self.slop = 0.15  # seconds allowed between stamps
        self.est_q = deque(maxlen=200)
        self.gt_q = deque(maxlen=200)

        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        self.create_subscription(
            Odometry,
            '/robot/estimated_odom',
            self.est_cb_manual,
            odom_qos,
        )
        self.create_subscription(
            Odometry,
            '/model/bgr/odometry',
            self.gt_cb_manual,
            odom_qos,
        )

        self.get_logger().info(
            "Using manual sync with BEST_EFFORT QoS on both odom topics."
        )

    def est_cb_manual(self, msg: Odometry):
        self.est_q.append(msg)
        self.try_match_manual()

    def gt_cb_manual(self, msg: Odometry):
        self.gt_q.append(msg)
        self.try_match_manual()

    def try_match_manual(self):
        while self.est_q and self.gt_q:
            best_ie = None
            best_ig = None
            best_abs_dt = float('inf')

            for ie, est_msg in enumerate(self.est_q):
                t_est = stamp_to_sec(est_msg.header.stamp)
                for ig, gt_msg in enumerate(self.gt_q):
                    t_gt = stamp_to_sec(gt_msg.header.stamp)
                    abs_dt = abs(t_est - t_gt)
                    if abs_dt < best_abs_dt:
                        best_abs_dt = abs_dt
                        best_ie = ie
                        best_ig = ig

            if best_ie is not None and best_ig is not None and best_abs_dt <= self.slop:
                est_msg = self.est_q[best_ie]
                gt_msg = self.gt_q[best_ig]
                del self.est_q[best_ie]
                del self.gt_q[best_ig]
                self.process_pair(est_msg, gt_msg)
                continue

            # prune stale messages if queues drift apart
            t_est0 = stamp_to_sec(self.est_q[0].header.stamp)
            t_gt0 = stamp_to_sec(self.gt_q[0].header.stamp)

            if t_est0 < t_gt0 - self.slop:
                self.est_q.popleft()
            elif t_gt0 < t_est0 - self.slop:
                self.gt_q.popleft()
            else:
                break

    def process_pair(self, est_msg: Odometry, gt_msg: Odometry):
        ex = float(est_msg.pose.pose.position.x)
        ey = float(est_msg.pose.pose.position.y)
        gx = float(gt_msg.pose.pose.position.x)
        gy = float(gt_msg.pose.pose.position.y)

        dx = ex - gx
        dy = ey - gy
        e2d = math.sqrt(dx * dx + dy * dy)

        self.inst_dx = dx
        self.inst_dy = dy
        self.inst_e2d = e2d
        self.inst_dt = (
            stamp_to_sec(est_msg.header.stamp) -
            stamp_to_sec(gt_msg.header.stamp)
        )

        self.est_x.append(ex)
        self.est_y.append(ey)
        self.gt_x.append(gx)
        self.gt_y.append(gy)

        self.sum_sq_x += dx * dx
        self.sum_sq_y += dy * dy
        self.sum_sq_2d += (dx * dx + dy * dy)
        self.count += 1

    def update_plot(self):
        if self.count < 2:
            return

        self.ax.clear()

        self.ax.set_xlabel("Global X [m]", fontsize=10, fontweight='bold')
        self.ax.set_ylabel("Global Y [m]", fontsize=10, fontweight='bold')
        self.ax.set_title(
            "BGR Autonomous Car - Localization Tracking",
            fontsize=12
        )

        self.ax.plot(
            self.gt_x, self.gt_y,
            'g-', label='Ground Truth (Simulator)', alpha=0.5
        )
        self.ax.plot(
            self.est_x, self.est_y,
            'r--', label='EKF Estimate', linewidth=1.5
        )

        self.ax.scatter(
            self.est_x[-1], self.est_y[-1],
            color='red', s=60, zorder=5, edgecolors='black'
        )
        self.ax.scatter(
            self.gt_x[-1], self.gt_y[-1],
            color='green', s=60, zorder=5, edgecolors='black'
        )

        rmse_x = math.sqrt(self.sum_sq_x / self.count)
        rmse_y = math.sqrt(self.sum_sq_y / self.count)
        rmse_2d = math.sqrt(self.sum_sq_2d / self.count)

        inst_rx = abs(self.inst_dx)
        inst_ry = abs(self.inst_dy)
        inst_r2d = self.inst_e2d

        stats_text = (
            f'--- PERFORMANCE ---\n'
            f'Instant |ex|: {inst_rx:.3f} m\n'
            f'Instant |ey|: {inst_ry:.3f} m\n'
            f'Instant e2D:  {inst_r2d:.3f} m\n'
            f'Δt(est-gt):   {self.inst_dt * 1000.0:.1f} ms\n'
            f'\n'
            f'Total RMSE_x: {rmse_x:.3f} m\n'
            f'Total RMSE_y: {rmse_y:.3f} m\n'
            f'Total RMSE_2D:{rmse_2d:.3f} m\n'
            f'Samples:      {self.count}'
        )

        self.ax.text(
            0.05, 0.95, stats_text,
            transform=self.ax.transAxes,
            fontsize=11,
            verticalalignment='top',
            family='monospace',
            bbox=dict(
                boxstyle='round',
                facecolor='white',
                alpha=0.88,
                edgecolor='gray'
            )
        )

        self.ax.legend(loc='lower right')
        self.ax.grid(True, linestyle=':', alpha=0.6)
        self.ax.axis('equal')
        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    node = RealTimePlotterSynced()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close('all')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()