#!/usr/bin/env python3
"""
pose_fuser.py — Priority pose fusion with ArUco-anchored odom takeover.

The important behaviour:

  * Raw odom+IMU is kept raw:
      - x/y comes from /odom pose transformed into the arena frame
      - yaw comes from /imu
      - no ArUco correction is applied to this raw odom state

  * Fused ArUco+Odom does NOT switch back to the raw odom frame.
    Instead, every accepted ArUco pose creates an anchor:

        anchor_aruco_pose = current ArUco arena pose
        anchor_odom_pose  = current raw odom+IMU arena pose

    Then odom takeover is:

        fused = anchor_aruco_pose + (current_odom - anchor_odom_pose)

    The odom delta is computed in the odom-anchor local frame and then rotated
    into the ArUco-anchor frame. This keeps the fused output continuous when
    ArUco disappears.
"""

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def pose2d_copy(msg: Pose2D) -> Pose2D:
    p = Pose2D()
    p.x = float(msg.x)
    p.y = float(msg.y)
    p.theta = wrap_angle(float(msg.theta))
    return p


@dataclass
class PoseState:
    x: float
    y: float
    yaw: float

    @classmethod
    def from_pose2d(cls, msg: Pose2D) -> "PoseState":
        return cls(float(msg.x), float(msg.y), wrap_angle(float(msg.theta)))

    def to_pose2d(self) -> Pose2D:
        p = Pose2D()
        p.x = self.x
        p.y = self.y
        p.theta = wrap_angle(self.yaw)
        return p


class PoseFuser(Node):
    def __init__(self):
        super().__init__("pose_fuser")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("aruco_topic", "/bot_pose_estimate")
        self.declare_parameter("overhead_topic", "/vision/robot_pose")
        self.declare_parameter("fused_topic", "/bot_pose_fused")
        self.declare_parameter("status_topic", "/localization_status")

        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("aruco_timeout", 0.35)
        self.declare_parameter("overhead_timeout", 0.5)

        # Overridden by sim.launch.py from control_tuning.yaml.
        self.declare_parameter("spawn_x_m", 0.3)
        self.declare_parameter("spawn_y_m", 1.8)
        self.declare_parameter("spawn_yaw_rad", -math.pi / 2.0)

        # Your Gazebo IMU currently starts at 0 deg even when the robot spawned
        # at -90 deg. Therefore the default is relative-to-spawn.
        self.declare_parameter("imu_yaw_relative_to_spawn", True)
        self.declare_parameter("imu_yaw_offset_deg", 0.0)

        # If the Gazebo /odom position is already in world coordinates, set this
        # false. For gz DiffDrive with frame_id=odom, true is normally correct.
        self.declare_parameter("odom_pose_relative_to_spawn", True)
        self.declare_parameter("odom_xy_yaw_offset_deg", 0.0)

        # ArUco outlier gate. It is checked against the predicted fused pose.
        self.declare_parameter("aruco_gate_distance_m", 0.45)
        self.declare_parameter("aruco_gate_yaw_deg", 45.0)
        self.declare_parameter("aruco_reacquire_s", 1.0)

        odom_topic = self.get_parameter("odom_topic").value
        imu_topic = self.get_parameter("imu_topic").value
        aruco_topic = self.get_parameter("aruco_topic").value
        overhead_topic = self.get_parameter("overhead_topic").value
        fused_topic = self.get_parameter("fused_topic").value
        status_topic = self.get_parameter("status_topic").value

        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.aruco_timeout = float(self.get_parameter("aruco_timeout").value)
        self.overhead_timeout = float(self.get_parameter("overhead_timeout").value)

        self.spawn_x = float(self.get_parameter("spawn_x_m").value)
        self.spawn_y = float(self.get_parameter("spawn_y_m").value)
        self.spawn_yaw = float(self.get_parameter("spawn_yaw_rad").value)

        self.imu_yaw_relative_to_spawn = bool(self.get_parameter("imu_yaw_relative_to_spawn").value)
        self.imu_yaw_offset = math.radians(float(self.get_parameter("imu_yaw_offset_deg").value))

        self.odom_pose_relative_to_spawn = bool(self.get_parameter("odom_pose_relative_to_spawn").value)
        self.odom_xy_yaw_offset = math.radians(float(self.get_parameter("odom_xy_yaw_offset_deg").value))

        self.aruco_gate_distance_m = float(self.get_parameter("aruco_gate_distance_m").value)
        self.aruco_gate_yaw_rad = math.radians(float(self.get_parameter("aruco_gate_yaw_deg").value))
        self.aruco_reacquire_s = float(self.get_parameter("aruco_reacquire_s").value)

        odom_rot = self.odom_xy_yaw_offset
        if self.odom_pose_relative_to_spawn:
            odom_rot += self.spawn_yaw
        self._cos_odom_rot = math.cos(odom_rot)
        self._sin_odom_rot = math.sin(odom_rot)

        self._imu_world_yaw = self.spawn_yaw
        self._have_imu = False

        # Raw odom+IMU pose. This is never ArUco-corrected.
        self._odom_pose = PoseState(self.spawn_x, self.spawn_y, self.spawn_yaw)
        self._have_odom = False

        # Last accepted ArUco anchor and matching odom anchor.
        self._last_aruco_pose: Optional[PoseState] = None
        self._odom_at_last_aruco: Optional[PoseState] = None
        self._last_aruco_time: Optional[float] = None

        # Overhead source.
        self._overhead_pose: Optional[Pose2D] = None
        self._last_overhead_time: Optional[float] = None

        self.pose_pub = self.create_publisher(Pose2D, fused_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)

        self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)
        self.create_subscription(Imu, imu_topic, self.imu_cb, 10)
        self.create_subscription(Pose2D, aruco_topic, self.aruco_cb, 10)
        self.create_subscription(Pose2D, overhead_topic, self.overhead_cb, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_fused)

        self.get_logger().info(
            f"pose_fuser | overhead={overhead_topic} | aruco={aruco_topic} | "
            f"odom={odom_topic} | imu={imu_topic} | out={fused_topic}\n"
            f"  raw odom+IMU: x/y=/odom pose transformed, yaw=/imu\n"
            f"  fused fallback: last ArUco + odom delta since that ArUco\n"
            f"  spawn: x={self.spawn_x:.3f} m, y={self.spawn_y:.3f} m, "
            f"yaw={math.degrees(self.spawn_yaw):.1f} deg\n"
            f"  imu_relative_to_spawn={self.imu_yaw_relative_to_spawn}, "
            f"odom_relative_to_spawn={self.odom_pose_relative_to_spawn}"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def aruco_fresh(self) -> bool:
        return (
            self._last_aruco_time is not None
            and (self.now_sec() - self._last_aruco_time) <= self.aruco_timeout
        )

    def overhead_fresh(self) -> bool:
        return (
            self._last_overhead_time is not None
            and (self.now_sec() - self._last_overhead_time) <= self.overhead_timeout
        )

    def _current_fused_from_anchor(self) -> Optional[PoseState]:
        """
        Return last_aruco_pose + odom_delta_since_last_aruco.

        The odom delta is first converted into the odom anchor's local frame,
        then rotated into the ArUco anchor frame. This makes the takeover
        continuous even if raw odom and ArUco do not share exactly the same
        absolute coordinate offset/yaw.
        """
        if self._last_aruco_pose is None or self._odom_at_last_aruco is None:
            return self._odom_pose if self._have_odom else None
        if not self._have_odom:
            return self._last_aruco_pose

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

        return PoseState(
            anchor_aruco.x + dx_fused,
            anchor_aruco.y + dy_fused,
            wrap_angle(anchor_aruco.yaw + dyaw),
        )

    def _aruco_measurement_is_valid(self, msg: Pose2D, now: float) -> bool:
        if self._last_aruco_time is None:
            return True
        if (now - self._last_aruco_time) > self.aruco_reacquire_s:
            return True

        predicted = self._current_fused_from_anchor()
        if predicted is None:
            return True

        dist = math.hypot(float(msg.x) - predicted.x, float(msg.y) - predicted.y)
        yaw_err = abs(wrap_angle(float(msg.theta) - predicted.yaw))

        if dist > self.aruco_gate_distance_m or yaw_err > self.aruco_gate_yaw_rad:
            self.get_logger().warn(
                f"Rejected ArUco outlier: dist={dist:.3f} m, "
                f"yaw={math.degrees(yaw_err):.1f} deg"
            )
            return False

        return True

    # ── Callbacks ─────────────────────────────────────────────────

    def imu_cb(self, msg: Imu):
        q = msg.orientation
        raw_imu_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        if self.imu_yaw_relative_to_spawn:
            self._imu_world_yaw = wrap_angle(self.spawn_yaw + raw_imu_yaw + self.imu_yaw_offset)
        else:
            self._imu_world_yaw = wrap_angle(raw_imu_yaw + self.imu_yaw_offset)

        self._have_imu = True

    def odom_cb(self, msg: Odometry):
        raw_x = float(msg.pose.pose.position.x)
        raw_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        raw_odom_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        if self.odom_pose_relative_to_spawn:
            x = self.spawn_x + self._cos_odom_rot * raw_x - self._sin_odom_rot * raw_y
            y = self.spawn_y + self._sin_odom_rot * raw_x + self._cos_odom_rot * raw_y
        else:
            x = raw_x
            y = raw_y

        if self._have_imu:
            yaw = self._imu_world_yaw
        elif self.odom_pose_relative_to_spawn:
            yaw = wrap_angle(self.spawn_yaw + raw_odom_yaw)
        else:
            yaw = raw_odom_yaw

        self._odom_pose = PoseState(x, y, yaw)
        self._have_odom = True

    def aruco_cb(self, msg: Pose2D):
        now = self.now_sec()
        if not self._aruco_measurement_is_valid(msg, now):
            return

        self._last_aruco_pose = PoseState.from_pose2d(msg)
        self._last_aruco_time = now

        # This is the anchor that makes odom take over from the ArUco pose.
        if self._have_odom:
            self._odom_at_last_aruco = PoseState(
                self._odom_pose.x,
                self._odom_pose.y,
                self._odom_pose.yaw,
            )

        # Publish the ArUco pose immediately so the fused stream snaps exactly
        # to the accepted measurement at the anchor instant.
        self.pose_pub.publish(self._last_aruco_pose.to_pose2d())
        self.publish_status("ARUCO_ANCHOR")

    def overhead_cb(self, msg: Pose2D):
        self._overhead_pose = pose2d_copy(msg)
        self._last_overhead_time = self.now_sec()

    # ── Publish ───────────────────────────────────────────────────

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def publish_fused(self):
        if self.overhead_fresh() and self._overhead_pose is not None:
            self.pose_pub.publish(self._overhead_pose)
            self.publish_status("OVERHEAD")
            return

        fused = self._current_fused_from_anchor()
        if fused is not None:
            self.pose_pub.publish(fused.to_pose2d())
            if self._last_aruco_pose is not None:
                self.publish_status("ARUCO_ODOM_FUSED")
            else:
                self.publish_status("ODOM_IMU_RAW")
            return

        self.publish_status("NO_DATA")


def main(args=None):
    rclpy.init(args=args)
    node = PoseFuser()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
