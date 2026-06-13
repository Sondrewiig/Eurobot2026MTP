#!/usr/bin/env python3
"""
pose_logger.py — Log pose sources to CSV files for offline analysis.

Records:
  1. Ground truth            -> /bot_pose_ground_truth
  2. Overhead estimate       -> /vision/robot_pose
  3. Raw odom + IMU          -> /odom x/y transformed to arena, /imu yaw
  4. Raw ArUco estimate      -> /bot_pose_estimate
  5. ArUco + Odom fused      -> ArUco anchors + odom takeover between detections

Important:
  - The odom_imu CSV is raw. It is NOT corrected by ArUco.
  - The fused CSV is the corrected/anchored estimate.
  - When ArUco disappears, fused continues from the last accepted ArUco pose
    using odom delta since that ArUco pose. It never jumps back to the raw odom
    coordinate frame.
"""

import csv
import math
import os
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class PoseState:
    x: float
    y: float
    yaw: float

    @classmethod
    def from_pose2d(cls, msg: Pose2D) -> "PoseState":
        return cls(float(msg.x), float(msg.y), wrap_angle(float(msg.theta)))


class PoseLogger(Node):
    def __init__(self):
        super().__init__("pose_logger")

        self.declare_parameter("log_dir", os.path.expanduser("~/pose_logs"))
        self.declare_parameter("ground_truth_topic", "/bot_pose_ground_truth")
        self.declare_parameter("overhead_topic", "/vision/robot_pose")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("aruco_topic", "/bot_pose_estimate")
        self.declare_parameter("aruco_timeout", 0.35)

        self.declare_parameter("spawn_x_m", 0.3)
        self.declare_parameter("spawn_y_m", 1.8)
        self.declare_parameter("spawn_yaw_rad", -math.pi / 2.0)

        self.declare_parameter("imu_yaw_relative_to_spawn", True)
        self.declare_parameter("imu_yaw_offset_deg", 0.0)
        self.declare_parameter("odom_pose_relative_to_spawn", True)
        self.declare_parameter("odom_xy_yaw_offset_deg", 0.0)

        self.declare_parameter("aruco_gate_distance_m", 0.45)
        self.declare_parameter("aruco_gate_yaw_deg", 45.0)
        self.declare_parameter("aruco_reacquire_s", 1.0)

        log_dir = self.get_parameter("log_dir").value
        ground_truth_topic = self.get_parameter("ground_truth_topic").value
        overhead_topic = self.get_parameter("overhead_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        imu_topic = self.get_parameter("imu_topic").value
        aruco_topic = self.get_parameter("aruco_topic").value
        self._aruco_timeout = float(self.get_parameter("aruco_timeout").value)

        self._spawn_x = float(self.get_parameter("spawn_x_m").value)
        self._spawn_y = float(self.get_parameter("spawn_y_m").value)
        self._spawn_yaw = float(self.get_parameter("spawn_yaw_rad").value)

        self._imu_yaw_relative_to_spawn = bool(self.get_parameter("imu_yaw_relative_to_spawn").value)
        self._imu_yaw_offset = math.radians(float(self.get_parameter("imu_yaw_offset_deg").value))

        self._odom_pose_relative_to_spawn = bool(self.get_parameter("odom_pose_relative_to_spawn").value)
        self._odom_xy_yaw_offset = math.radians(float(self.get_parameter("odom_xy_yaw_offset_deg").value))

        self._aruco_gate_distance_m = float(self.get_parameter("aruco_gate_distance_m").value)
        self._aruco_gate_yaw_rad = math.radians(float(self.get_parameter("aruco_gate_yaw_deg").value))
        self._aruco_reacquire_s = float(self.get_parameter("aruco_reacquire_s").value)

        odom_rot = self._odom_xy_yaw_offset
        if self._odom_pose_relative_to_spawn:
            odom_rot += self._spawn_yaw
        self._cos_odom_rot = math.cos(odom_rot)
        self._sin_odom_rot = math.sin(odom_rot)

        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        ground_truth_path = os.path.join(log_dir, f"ground_truth_{timestamp}.csv")
        overhead_path = os.path.join(log_dir, f"overhead_{timestamp}.csv")
        odom_imu_path = os.path.join(log_dir, f"odom_imu_{timestamp}.csv")
        aruco_path = os.path.join(log_dir, f"aruco_{timestamp}.csv")
        fused_path = os.path.join(log_dir, f"fused_aruco_odom_{timestamp}.csv")

        self._ground_truth_file = open(ground_truth_path, "w", newline="")
        self._overhead_file = open(overhead_path, "w", newline="")
        self._odom_imu_file = open(odom_imu_path, "w", newline="")
        self._aruco_file = open(aruco_path, "w", newline="")
        self._fused_file = open(fused_path, "w", newline="")

        self._ground_truth_csv = csv.writer(self._ground_truth_file)
        self._overhead_csv = csv.writer(self._overhead_file)
        self._odom_imu_csv = csv.writer(self._odom_imu_file)
        self._aruco_csv = csv.writer(self._aruco_file)
        self._fused_csv = csv.writer(self._fused_file)

        pose2d_header = [
            "ros_time_s", "wall_time_s",
            "x_m", "y_m", "yaw_rad", "yaw_deg",
        ]
        self._ground_truth_csv.writerow(pose2d_header)
        self._overhead_csv.writerow(pose2d_header)
        self._aruco_csv.writerow(pose2d_header)

        self._odom_imu_csv.writerow([
            "ros_time_s", "wall_time_s",
            # Main raw odom+IMU estimate.
            "odom_x_m", "odom_y_m", "imu_yaw_rad", "imu_yaw_deg",
            # Raw /odom diagnostics.
            "raw_odom_x_m", "raw_odom_y_m", "raw_odom_yaw_rad", "raw_odom_yaw_deg",
            "odom_vx_m_s", "odom_vy_m_s", "odom_wz_rad_s",
            # IMU diagnostics.
            "raw_imu_yaw_rad", "raw_imu_yaw_deg",
            "imu_ax_m_s2", "imu_ay_m_s2", "imu_az_m_s2",
            "imu_wx_rad_s", "imu_wy_rad_s", "imu_wz_rad_s",
        ])

        self._fused_csv.writerow([
            "ros_time_s", "wall_time_s",
            "x_m", "y_m", "yaw_rad", "yaw_deg",
            "source",
            "anchor_age_s",
            "odom_dx_since_anchor_m", "odom_dy_since_anchor_m", "odom_dyaw_since_anchor_deg",
        ])

        self._raw_imu_yaw = 0.0
        self._imu_world_yaw = self._spawn_yaw
        self._have_imu = False
        self._imu_ax = 0.0
        self._imu_ay = 0.0
        self._imu_az = 0.0
        self._imu_wx = 0.0
        self._imu_wy = 0.0
        self._imu_wz = 0.0

        # Raw odom+IMU state. No ArUco correction here.
        self._odom_pose = PoseState(self._spawn_x, self._spawn_y, self._spawn_yaw)
        self._have_odom = False

        # Fused anchor state.
        self._last_aruco_pose: Optional[PoseState] = None
        self._odom_at_last_aruco: Optional[PoseState] = None
        self._latest_accepted_aruco_time: Optional[float] = None

        self._ground_truth_count = 0
        self._overhead_count = 0
        self._odom_imu_count = 0
        self._aruco_count = 0
        self._aruco_rejected_count = 0
        self._fused_count = 0

        self.create_subscription(Pose2D, ground_truth_topic, self._ground_truth_cb, 10)
        self.create_subscription(Pose2D, overhead_topic, self._overhead_cb, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_subscription(Imu, imu_topic, self._imu_cb, 10)
        self.create_subscription(Pose2D, aruco_topic, self._aruco_cb, 10)

        self.create_timer(5.0, self._status_timer)

        self.get_logger().info(
            f"pose_logger started — writing to {log_dir}/\n"
            f"  ground_truth : {ground_truth_path}\n"
            f"  overhead     : {overhead_path}\n"
            f"  odom+imu     : {odom_imu_path}\n"
            f"  aruco        : {aruco_path}\n"
            f"  fused        : {fused_path}\n"
            f"  raw odom+IMU: x/y=/odom pose transformed, yaw=/imu\n"
            f"  fused: last ArUco + odom delta since last ArUco"
        )

    # ── Time helpers ────────────────────────────────────────────

    def _ros_time_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _wall_time_s() -> float:
        return time.time()

    # ── Shared helpers ─────────────────────────────────────────

    def _write_pose2d_row(self, writer, msg: Pose2D):
        yaw_rad = wrap_angle(float(msg.theta))
        writer.writerow([
            f"{self._ros_time_s():.6f}",
            f"{self._wall_time_s():.6f}",
            f"{float(msg.x):.6f}",
            f"{float(msg.y):.6f}",
            f"{yaw_rad:.6f}",
            f"{math.degrees(yaw_rad):.4f}",
        ])

    def _current_fused_from_anchor(self) -> tuple[Optional[PoseState], str, float, float, float, float]:
        if self._last_aruco_pose is None or self._odom_at_last_aruco is None:
            if self._have_odom:
                return self._odom_pose, "odom_raw_unanchored", 0.0, 0.0, 0.0, 0.0
            return None, "no_data", 0.0, 0.0, 0.0, 0.0

        if not self._have_odom:
            return self._last_aruco_pose, "aruco_no_odom", 0.0, 0.0, 0.0, 0.0

        current = self._odom_pose
        anchor_odom = self._odom_at_last_aruco
        anchor_aruco = self._last_aruco_pose

        dx_world = current.x - anchor_odom.x
        dy_world = current.y - anchor_odom.y

        c0 = math.cos(-anchor_odom.yaw)
        s0 = math.sin(-anchor_odom.yaw)
        dx_local = c0 * dx_world - s0 * dy_world
        dy_local = s0 * dx_world + c0 * dy_world

        ca = math.cos(anchor_aruco.yaw)
        sa = math.sin(anchor_aruco.yaw)
        dx_fused = ca * dx_local - sa * dy_local
        dy_fused = sa * dx_local + ca * dy_local

        dyaw = wrap_angle(current.yaw - anchor_odom.yaw)

        fused = PoseState(
            anchor_aruco.x + dx_fused,
            anchor_aruco.y + dy_fused,
            wrap_angle(anchor_aruco.yaw + dyaw),
        )

        anchor_age = 0.0
        if self._latest_accepted_aruco_time is not None:
            anchor_age = self._ros_time_s() - self._latest_accepted_aruco_time

        return fused, "odom_from_last_aruco", anchor_age, dx_world, dy_world, math.degrees(dyaw)

    def _aruco_measurement_is_valid(self, msg: Pose2D, now: float) -> bool:
        if self._latest_accepted_aruco_time is None:
            return True
        if (now - self._latest_accepted_aruco_time) > self._aruco_reacquire_s:
            return True

        predicted, _, _, _, _, _ = self._current_fused_from_anchor()
        if predicted is None:
            return True

        dist = math.hypot(float(msg.x) - predicted.x, float(msg.y) - predicted.y)
        yaw_err = abs(wrap_angle(float(msg.theta) - predicted.yaw))

        if dist > self._aruco_gate_distance_m or yaw_err > self._aruco_gate_yaw_rad:
            self._aruco_rejected_count += 1
            self.get_logger().warn(
                f"Rejected ArUco outlier: dist={dist:.3f} m, yaw={math.degrees(yaw_err):.1f} deg"
            )
            return False

        return True

    def _write_fused_row(self, source_override: Optional[str] = None):
        fused, source, anchor_age, dx, dy, dyaw_deg = self._current_fused_from_anchor()
        if fused is None:
            return

        if source_override is not None:
            source = source_override

        self._fused_csv.writerow([
            f"{self._ros_time_s():.6f}",
            f"{self._wall_time_s():.6f}",
            f"{fused.x:.6f}",
            f"{fused.y:.6f}",
            f"{wrap_angle(fused.yaw):.6f}",
            f"{math.degrees(wrap_angle(fused.yaw)):.4f}",
            source,
            f"{anchor_age:.6f}",
            f"{dx:.6f}",
            f"{dy:.6f}",
            f"{dyaw_deg:.4f}",
        ])
        self._fused_count += 1

    # ── Callbacks ──────────────────────────────────────────────

    def _ground_truth_cb(self, msg: Pose2D):
        self._write_pose2d_row(self._ground_truth_csv, msg)
        self._ground_truth_count += 1

    def _overhead_cb(self, msg: Pose2D):
        self._write_pose2d_row(self._overhead_csv, msg)
        self._overhead_count += 1

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        self._raw_imu_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        if self._imu_yaw_relative_to_spawn:
            self._imu_world_yaw = wrap_angle(self._spawn_yaw + self._raw_imu_yaw + self._imu_yaw_offset)
        else:
            self._imu_world_yaw = wrap_angle(self._raw_imu_yaw + self._imu_yaw_offset)

        self._have_imu = True
        self._imu_ax = float(msg.linear_acceleration.x)
        self._imu_ay = float(msg.linear_acceleration.y)
        self._imu_az = float(msg.linear_acceleration.z)
        self._imu_wx = float(msg.angular_velocity.x)
        self._imu_wy = float(msg.angular_velocity.y)
        self._imu_wz = float(msg.angular_velocity.z)

    def _odom_cb(self, msg: Odometry):
        raw_x = float(msg.pose.pose.position.x)
        raw_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        raw_odom_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular

        if self._odom_pose_relative_to_spawn:
            x = self._spawn_x + self._cos_odom_rot * raw_x - self._sin_odom_rot * raw_y
            y = self._spawn_y + self._sin_odom_rot * raw_x + self._cos_odom_rot * raw_y
        else:
            x = raw_x
            y = raw_y

        if self._have_imu:
            yaw = self._imu_world_yaw
        elif self._odom_pose_relative_to_spawn:
            yaw = wrap_angle(self._spawn_yaw + raw_odom_yaw)
        else:
            yaw = raw_odom_yaw

        self._odom_pose = PoseState(x, y, yaw)
        self._have_odom = True

        self._odom_imu_csv.writerow([
            f"{self._ros_time_s():.6f}",
            f"{self._wall_time_s():.6f}",
            f"{self._odom_pose.x:.6f}",
            f"{self._odom_pose.y:.6f}",
            f"{self._odom_pose.yaw:.6f}",
            f"{math.degrees(self._odom_pose.yaw):.4f}",
            f"{raw_x:.6f}",
            f"{raw_y:.6f}",
            f"{raw_odom_yaw:.6f}",
            f"{math.degrees(raw_odom_yaw):.4f}",
            f"{float(v.x):.6f}",
            f"{float(v.y):.6f}",
            f"{float(w.z):.6f}",
            f"{self._raw_imu_yaw:.6f}",
            f"{math.degrees(self._raw_imu_yaw):.4f}",
            f"{self._imu_ax:.6f}",
            f"{self._imu_ay:.6f}",
            f"{self._imu_az:.6f}",
            f"{self._imu_wx:.6f}",
            f"{self._imu_wy:.6f}",
            f"{self._imu_wz:.6f}",
        ])
        self._odom_imu_count += 1

        # This is the odom takeover row. It keeps moving after ArUco disappears.
        self._write_fused_row()

    def _aruco_cb(self, msg: Pose2D):
        # Raw ArUco is logged exactly as received. Your plotter can break this
        # line at gaps so missing detections are not drawn as continuous lines.
        self._write_pose2d_row(self._aruco_csv, msg)
        self._aruco_count += 1

        now = self._ros_time_s()
        if not self._aruco_measurement_is_valid(msg, now):
            self._write_fused_row(source_override="rejected_aruco_keep_odom")
            return

        self._last_aruco_pose = PoseState.from_pose2d(msg)
        self._latest_accepted_aruco_time = now

        if self._have_odom:
            self._odom_at_last_aruco = PoseState(
                self._odom_pose.x,
                self._odom_pose.y,
                self._odom_pose.yaw,
            )

        # Log the exact ArUco anchor row immediately. Later odom rows will
        # continue from this point instead of jumping back to raw odom.
        self._fused_csv.writerow([
            f"{self._ros_time_s():.6f}",
            f"{self._wall_time_s():.6f}",
            f"{self._last_aruco_pose.x:.6f}",
            f"{self._last_aruco_pose.y:.6f}",
            f"{self._last_aruco_pose.yaw:.6f}",
            f"{math.degrees(self._last_aruco_pose.yaw):.4f}",
            "aruco_anchor",
            "0.000000",
            "0.000000",
            "0.000000",
            "0.0000",
        ])
        self._fused_count += 1

    def _status_timer(self):
        self.get_logger().info(
            f"pose_logger rows: ground_truth={self._ground_truth_count}  "
            f"overhead={self._overhead_count}  odom_imu={self._odom_imu_count}  "
            f"aruco={self._aruco_count} rejected_aruco={self._aruco_rejected_count}  "
            f"fused={self._fused_count}"
        )
        self._ground_truth_file.flush()
        self._overhead_file.flush()
        self._odom_imu_file.flush()
        self._aruco_file.flush()
        self._fused_file.flush()

    def destroy_node(self):
        self.get_logger().info(
            f"pose_logger shutting down — ground_truth={self._ground_truth_count} "
            f"overhead={self._overhead_count} odom_imu={self._odom_imu_count} "
            f"aruco={self._aruco_count} rejected_aruco={self._aruco_rejected_count} "
            f"fused={self._fused_count} rows written"
        )
        self._ground_truth_file.close()
        self._overhead_file.close()
        self._odom_imu_file.close()
        self._aruco_file.close()
        self._fused_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PoseLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
