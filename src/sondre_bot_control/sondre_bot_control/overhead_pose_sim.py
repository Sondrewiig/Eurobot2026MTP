import math
import random

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped


def yaw_to_quat(yaw: float):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


class OverheadPoseSim(Node):
    def __init__(self):
        super().__init__("overhead_pose_sim")

        self.declare_parameter("input_topic", "/bot_pose_ground_truth")
        self.declare_parameter("output_topic", "/bot_pose_overhead")
        self.declare_parameter("frame_id", "field")
        self.declare_parameter("pos_noise_std", 0.0)
        self.declare_parameter("yaw_noise_std_deg", 0.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.pos_noise_std = float(self.get_parameter("pos_noise_std").value)
        self.yaw_noise_std = math.radians(
            float(self.get_parameter("yaw_noise_std_deg").value)
        )

        self.pub = self.create_publisher(PoseWithCovarianceStamped, output_topic, 10)
        self.sub = self.create_subscription(Pose2D, input_topic, self.gt_cb, 10)

        self.get_logger().info(
            f"overhead_pose_sim started | in={input_topic} | out={output_topic} | frame={self.frame_id}"
        )

    def gt_cb(self, msg: Pose2D):
        x = msg.x + random.gauss(0.0, self.pos_noise_std)
        y = msg.y + random.gauss(0.0, self.pos_noise_std)
        yaw = msg.theta + random.gauss(0.0, self.yaw_noise_std)

        qx, qy, qz, qw = yaw_to_quat(yaw)

        out = PoseWithCovarianceStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id

        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.position.z = 0.0

        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw

        cov = [0.0] * 36
        cov[0] = self.pos_noise_std ** 2
        cov[7] = self.pos_noise_std ** 2
        cov[35] = self.yaw_noise_std ** 2
        out.pose.covariance = cov

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OverheadPoseSim()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()