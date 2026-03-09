import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import SetBool


class DriveModeMux(Node):
    def __init__(self):
        super().__init__('drive_mode_mux')

        self.manual_mode = False  # False = AUTO, True = MANUAL

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/drive_mode', 10)

        self.auto_sub = self.create_subscription(
            Twist, '/cmd_vel_auto', self.auto_callback, 10
        )
        self.manual_sub = self.create_subscription(
            Twist, '/cmd_vel_manual', self.manual_callback, 10
        )

        self.mode_srv = self.create_service(
            SetBool, '/set_manual_mode', self.set_manual_mode_callback
        )

        self.timer = self.create_timer(0.5, self.publish_mode)

        self.get_logger().info('DriveModeMux started in AUTO mode')

    def publish_mode(self):
        msg = String()
        msg.data = 'MANUAL' if self.manual_mode else 'AUTO'
        self.mode_pub.publish(msg)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def set_manual_mode_callback(self, request, response):
        self.manual_mode = request.data
        self.stop_robot()

        mode = 'MANUAL' if self.manual_mode else 'AUTO'
        response.success = True
        response.message = f'Switched to {mode} mode'

        self.get_logger().info(response.message)
        self.publish_mode()
        return response

    def auto_callback(self, msg):
        if not self.manual_mode:
            self.cmd_pub.publish(msg)

    def manual_callback(self, msg):
        if self.manual_mode:
            self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DriveModeMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()