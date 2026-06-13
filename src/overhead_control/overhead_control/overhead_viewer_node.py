import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class OverheadViewerNode(Node):
    def __init__(self):
        super().__init__("overhead_viewer_node")

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            "/overhead/debug_image",
            self.image_callback,
            10,
        )

        self.get_logger().info("Overhead viewer started")
        self.get_logger().info("Subscribing to /overhead/debug_image")
        self.get_logger().info("Press q in the image window to close viewer")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        cv2.imshow("Overhead Debug Feed", frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            self.get_logger().info("q pressed, closing viewer")
            rclpy.shutdown()

    def destroy_node(self):
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = OverheadViewerNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()