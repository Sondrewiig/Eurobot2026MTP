import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import numpy as np
import cv2


class VisionDrive(Node):

    def __init__(self):
        super().__init__('vision_drive')

        self.ball_reached = False
        self.prev_error = 0.0

        # memory variables
        self.last_seen_error = 0.0
        self.frames_since_seen = 0

        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        self.get_logger().info("vision_drive node started")


    def image_callback(self, msg):

        img = np.frombuffer(msg.data, dtype=np.uint8)
        img = img.reshape((msg.height, msg.width, 3))
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower = np.array([18, 80, 80])
        upper = np.array([40, 255, 255])

        mask = cv2.inRange(hsv, lower, upper)

        pixels = np.sum(mask > 0)

        twist = Twist()

        if pixels > 200:

            M = cv2.moments(mask)

            if M["m00"] > 0:

                self.frames_since_seen = 0

                cx = int(M["m10"] / M["m00"])

                raw_error = (cx - msg.width/2) / (msg.width/2)

                # smooth error
                error = 0.7 * self.prev_error + 0.3 * raw_error
                self.prev_error = error

                # store memory direction
                self.last_seen_error = error

                ratio = pixels / (msg.width * msg.height)

                stop_ratio = 0.012
                leave_ratio = 0.008

                if ratio > stop_ratio:
                    self.ball_reached = True

                if ratio < leave_ratio:
                    self.ball_reached = False

                if self.ball_reached:

                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.get_logger().info("BALL REACHED")

                else:

                    align_threshold = 0.15

                    if abs(error) > align_threshold:

                        twist.linear.x = 0.0
                        twist.angular.z = np.clip(-1.2 * error, -1.0, 1.0)

                    else:

                        if ratio < 0.003:
                            twist.linear.x = 0.35
                        elif ratio < 0.008:
                            twist.linear.x = 0.18
                        else:
                            twist.linear.x = 0.08

                        if ratio < 0.003:
                            twist.angular.z = np.clip(-0.4 * error, -0.4, 0.4)
                        else:
                            twist.angular.z = np.clip(-0.8 * error, -0.6, 0.6)

                    self.get_logger().info(
                        f"Tracking ball | error={error:.2f} | size={ratio:.4f}"
                    )

        else:

            self.frames_since_seen += 1

            self.ball_reached = False

            twist.linear.x = 0.0
            
            seconds_searching = 5
            frame_rate = 30

            # short-term memory: turn toward last seen direction
            if self.frames_since_seen < seconds_searching * frame_rate:

                twist.angular.z = np.clip(-1.0 * self.last_seen_error, -0.6, 0.6)

                self.get_logger().info("Ball lost — turning toward memory")

            else:

                twist.angular.z = 0.3

                self.get_logger().info("Searching for ball")

        self.pub.publish(twist)


def main(args=None):

    rclpy.init(args=args)

    node = VisionDrive()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()