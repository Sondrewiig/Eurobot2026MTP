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
        self.declare_parameter("model_name", "")
        self.declare_parameter("publish_topic", "/bot_pose_ground_truth")

        tf_topic = self.get_parameter("tf_topic").value
        self.pose_index = int(self.get_parameter("pose_index").value)
        self.model_name = str(self.get_parameter("model_name").value).strip()
        publish_topic = self.get_parameter("publish_topic").value

        self.pub = self.create_publisher(Pose2D, publish_topic, 10)

        self.sub = self.create_subscription(
            TFMessage,
            tf_topic,
            self.tf_callback,
            10
        )

        self.last_log_time_ns = 0
        self.last_missing_log_time_ns = 0

        target = f'model "{self.model_name}"' if self.model_name else f"transform index {self.pose_index}"
        self.get_logger().info(
            f"Listening on {tf_topic} using {target}; publishing {publish_topic}"
        )

    def _name_matches(self, frame_id: str) -> bool:
        if not self.model_name:
            return False
        if not frame_id:
            return False
        name = str(frame_id)
        model = self.model_name
        return (
            name == model
            or name.endswith("/" + model)
            or name.startswith(model + "/")
            or ("::" in name and (name.split("::")[0] == model or model in name.split("::")))
        )

    def _select_transform(self, msg: TFMessage):
        if self.model_name:
            # Prefer an exact top-level model frame.
            for tf in msg.transforms:
                if tf.child_frame_id == self.model_name:
                    return tf

            # Then accept scoped names such as model/link or world/model.
            for tf in msg.transforms:
                if self._name_matches(tf.child_frame_id) or self._name_matches(tf.header.frame_id):
                    return tf

            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_missing_log_time_ns > 1_000_000_000:
                sample = [t.child_frame_id for t in msg.transforms[:8]]
                self.get_logger().warn(
                    f"No transform found for model '{self.model_name}'. "
                    f"Sample child_frame_id values: {sample}"
                )
                self.last_missing_log_time_ns = now_ns
            return None

        if len(msg.transforms) <= self.pose_index:
            return None
        return msg.transforms[self.pose_index]

    def tf_callback(self, msg: TFMessage):
        tf = self._select_transform(msg)
        if tf is None:
            return

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
            label = self.model_name if self.model_name else f"index {self.pose_index}"
            self.get_logger().info(
                f"ground truth [{label}]: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f} deg"
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
