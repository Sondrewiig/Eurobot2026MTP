#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from std_msgs.msg import String


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class TimedPose:
    pose: Pose2D
    stamp_s: float


class PoseFuser(Node):
    """
    Priority pose selector:

      1) Overhead camera pose, if fresh
      2) ArUco pose, if fresh
      3) OpenCR/ESP odometry, if fresh
      4) Publish nothing, status WAITING_FOR_POSE

    This intentionally does NOT blend odometry into camera estimates.
    If a real camera estimate exists, it wins.
    """

    def __init__(self):
        super().__init__("pose_fuser")

        self.declare_parameter("overhead_topic", "/vision/robot_pose")
        self.declare_parameter("aruco_topic", "/bot_pose_estimate")
        self.declare_parameter("odom_topic", "/opencr/odom_pose")

        self.declare_parameter("fused_topic", "/bot_pose_fused")
        self.declare_parameter("status_topic", "/localization_status")

        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("overhead_timeout_s", 0.7)
        self.declare_parameter("aruco_timeout_s", 0.7)
        self.declare_parameter("odom_timeout_s", 1.5)

        self.overhead_topic = self.get_parameter("overhead_topic").value
        self.aruco_topic = self.get_parameter("aruco_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value

        self.fused_topic = self.get_parameter("fused_topic").value
        self.status_topic = self.get_parameter("status_topic").value

        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.overhead_timeout_s = float(self.get_parameter("overhead_timeout_s").value)
        self.aruco_timeout_s = float(self.get_parameter("aruco_timeout_s").value)
        self.odom_timeout_s = float(self.get_parameter("odom_timeout_s").value)

        self.overhead: Optional[TimedPose] = None
        self.aruco: Optional[TimedPose] = None
        self.odom: Optional[TimedPose] = None

        self.pose_pub = self.create_publisher(Pose2D, self.fused_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.create_subscription(Pose2D, self.overhead_topic, self.overhead_cb, 10)
        self.create_subscription(Pose2D, self.aruco_topic, self.aruco_cb, 10)
        self.create_subscription(Pose2D, self.odom_topic, self.odom_cb, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_best_pose)

        self.get_logger().info(
            "pose_fuser priority mode started | "
            f"overhead={self.overhead_topic} | "
            f"aruco={self.aruco_topic} | "
            f"odom={self.odom_topic} | "
            f"out={self.fused_topic}"
        )

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def copy_pose(self, msg: Pose2D) -> Pose2D:
        p = Pose2D()
        p.x = float(msg.x)
        p.y = float(msg.y)
        p.theta = wrap_angle(float(msg.theta))
        return p

    def overhead_cb(self, msg: Pose2D):
        self.overhead = TimedPose(self.copy_pose(msg), self.now_s())

    def aruco_cb(self, msg: Pose2D):
        self.aruco = TimedPose(self.copy_pose(msg), self.now_s())

    def odom_cb(self, msg: Pose2D):
        self.odom = TimedPose(self.copy_pose(msg), self.now_s())

    def is_fresh(self, tp: Optional[TimedPose], timeout_s: float) -> bool:
        if tp is None:
            return False
        return (self.now_s() - tp.stamp_s) <= timeout_s

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def publish_pose(self, source: str, pose: Pose2D):
        self.pose_pub.publish(pose)
        self.publish_status(source)

    def publish_best_pose(self):
        if self.is_fresh(self.overhead, self.overhead_timeout_s):
            self.publish_pose("OVERHEAD", self.overhead.pose)
            return

        if self.is_fresh(self.aruco, self.aruco_timeout_s):
            self.publish_pose("ARUCO", self.aruco.pose)
            return

        if self.is_fresh(self.odom, self.odom_timeout_s):
            self.publish_pose("ODOM_FALLBACK", self.odom.pose)
            return

        self.publish_status("WAITING_FOR_POSE")


def main(args=None):
    rclpy.init(args=args)
    node = PoseFuser()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
