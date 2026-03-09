import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import numpy as np
import cv2


class VisionDrive(Node):

    SEARCHING = "SEARCHING"
    TURNING_TO_MEMORY = "TURNING_TO_MEMORY"
    TRACKING = "TRACKING"
    APPROACHING = "APPROACHING"
    REACHED_BALL = "REACHED_BALL"

    def __init__(self):
        super().__init__('vision_drive')

        self.ball_reached = False
        self.prev_error = 0.0

        # memory variables
        self.last_seen_error = 0.0
        self.frames_since_seen = 0

        # explicit state
        self.state = None

        # debug memory
        self.last_error = 0.0
        self.last_ratio = 0.0
        self.debug_counter = 0

        self.sub = self.create_subscription(
            Image,
            '/camera/left/image_raw',
            self.image_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.state_pub = self.create_publisher(
            String,
            '/bot_state',
            10
        )

        # publish current state periodically so topic echo always shows something
        self.state_timer = self.create_timer(0.5, self.publish_state)

        self.set_state(self.SEARCHING)
        self.get_logger().info("vision_drive node started")

    def set_state(self, new_state):
        if self.state != new_state:
            old_state = self.state
            self.state = new_state
            if old_state is None:
                self.get_logger().info(f"State -> {new_state}")
            else:
                self.get_logger().info(f"State: {old_state} -> {new_state}")

        self.publish_state()

    def publish_state(self):
        msg = String()
        msg.data = self.state if self.state is not None else "UNKNOWN"
        self.state_pub.publish(msg)

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

        M = cv2.moments(mask)

        if pixels > 200 and M["m00"] > 0:

            self.frames_since_seen = 0

            cx = int(M["m10"] / M["m00"])
            raw_error = (cx - msg.width / 2) / (msg.width / 2)

            # smooth error
            error = 0.7 * self.prev_error + 0.3 * raw_error
            self.prev_error = error

            # store memory direction
            self.last_seen_error = error
            self.last_error = error

            ratio = pixels / (msg.width * msg.height)
            self.last_ratio = ratio

            stop_ratio = 0.012
            leave_ratio = 0.008

            if ratio > stop_ratio:
                self.ball_reached = True

            if ratio < leave_ratio:
                self.ball_reached = False

            if self.ball_reached:
                self.set_state(self.REACHED_BALL)
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            else:
                align_threshold = 0.15

                if abs(error) > align_threshold:
                    self.set_state(self.TRACKING)
                    twist.linear.x = 0.0
                    twist.angular.z = float(np.clip(-1.2 * error, -1.0, 1.0))

                else:
                    self.set_state(self.APPROACHING)

                    if ratio < 0.003:
                        twist.linear.x = 0.35
                    elif ratio < 0.008:
                        twist.linear.x = 0.18
                    else:
                        twist.linear.x = 0.08

                    if ratio < 0.003:
                        twist.angular.z = float(np.clip(-0.4 * error, -0.4, 0.4))
                    else:
                        twist.angular.z = float(np.clip(-0.8 * error, -0.6, 0.6))

        else:
            self.frames_since_seen += 1
            self.ball_reached = False
            self.last_ratio = 0.0

            twist.linear.x = 0.0

            seconds_searching = 5
            frame_rate = 30

            # use memory only if we actually saw the ball somewhere
            if (
                self.frames_since_seen < seconds_searching * frame_rate
                and abs(self.last_seen_error) > 0.02
            ):
                self.set_state(self.TURNING_TO_MEMORY)
                twist.angular.z = float(np.clip(-1.0 * self.last_seen_error, -0.6, 0.6))

            else:
                self.set_state(self.SEARCHING)
                twist.angular.z = 0.3

        self.cmd_pub.publish(twist)

        # small debug print every ~15 frames instead of every frame
        self.debug_counter += 1
        if self.debug_counter % 15 == 0:
            self.get_logger().info(
                f"state={self.state} | error={self.last_error:.2f} | "
                f"size={self.last_ratio:.4f} | vx={twist.linear.x:.2f} | wz={twist.angular.z:.2f}"
            )


def main(args=None):
    rclpy.init(args=args)

    node = VisionDrive()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()