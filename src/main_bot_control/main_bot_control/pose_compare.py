import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class PoseCompare(Node):
    def __init__(self):
        super().__init__("pose_compare")

        self.gt_pose = None
        self.est_pose = None

        self.create_subscription(
            Pose2D,
            "/bot_pose_ground_truth",
            self.gt_callback,
            10
        )

        self.create_subscription(
            Pose2D,
            "/bot_pose_estimate",
            self.est_callback,
            10
        )

        self.timer = self.create_timer(0.3, self.timer_callback)

        self.get_logger().info("Comparing /bot_pose_ground_truth and /bot_pose_estimate")

    def gt_callback(self, msg: Pose2D):
        self.gt_pose = msg

    def est_callback(self, msg: Pose2D):
        self.est_pose = msg

    def timer_callback(self):
        if self.gt_pose is None or self.est_pose is None:
            return

        dx = self.est_pose.x - self.gt_pose.x
        dy = self.est_pose.y - self.gt_pose.y
        dpos = math.sqrt(dx * dx + dy * dy)

        dyaw = wrap_angle(self.est_pose.theta - self.gt_pose.theta)
        dyaw_deg = math.degrees(dyaw)

        self.get_logger().info(
            f"GT:  x={self.gt_pose.x:.3f}, y={self.gt_pose.y:.3f}, yaw={math.degrees(self.gt_pose.theta):.1f} deg | "
            f"EST: x={self.est_pose.x:.3f}, y={self.est_pose.y:.3f}, yaw={math.degrees(self.est_pose.theta):.1f} deg | "
            f"ERR: dx={dx:.3f}, dy={dy:.3f}, pos={dpos:.3f} m, yaw={dyaw_deg:.1f} deg"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PoseCompare()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()