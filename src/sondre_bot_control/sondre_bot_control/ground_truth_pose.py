import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from tf2_msgs.msg import TFMessage


class GroundTruthPose(Node):
    def __init__(self):
        super().__init__("ground_truth_pose")

        self.declare_parameter("tf_topic", "/world/Eurobot_world/dynamic_pose/info")
        self.declare_parameter("pose_index", 0)
        self.declare_parameter("publish_topic", "/bot_pose_ground_truth")

        tf_topic = self.get_parameter("tf_topic").value
        self.pose_index = int(self.get_parameter("pose_index").value)
        publish_topic = self.get_parameter("publish_topic").value

        self.pub = self.create_publisher(Pose2D, publish_topic, 10)

        self.sub = self.create_subscription(
            TFMessage,
            tf_topic,
            self.tf_callback,
            10
        )

        self.last_log_time_ns = 0

        self.get_logger().info(
            f"Listening on {tf_topic} using transform index {self.pose_index}"
        )

    def tf_callback(self, msg: TFMessage):
        if len(msg.transforms) <= self.pose_index:
            return

        tf = msg.transforms[self.pose_index]

        x = tf.transform.translation.x
        y = tf.transform.translation.y

        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        pose = Pose2D()
        pose.x = x
        pose.y = y
        pose.theta = yaw
        self.pub.publish(pose)

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_log_time_ns > 300_000_000:
            self.get_logger().info(
                f"ground truth: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f} deg"
            )
            self.last_log_time_ns = now_ns


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthPose()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()