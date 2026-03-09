import math
import sys
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import Pose2D, Twist


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class TelemetryConsole(Node):
    def __init__(self):
        super().__init__("telemetry_console")

        self.state = None
        self.aruco_ids = None
        self.est_pose = None
        self.gt_pose = None
        self.cmd_vel = None

        self.create_subscription(String, "/bot_state", self.state_cb, 10)
        self.create_subscription(Int32MultiArray, "/aruco_ids", self.aruco_cb, 10)
        self.create_subscription(Pose2D, "/bot_pose_estimate", self.est_cb, 10)
        self.create_subscription(Pose2D, "/bot_pose_ground_truth", self.gt_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)

        self.timer = self.create_timer(0.25, self.draw)

        self.get_logger().info("Telemetry console started")

    def state_cb(self, msg: String):
        self.state = msg.data

    def aruco_cb(self, msg: Int32MultiArray):
        self.aruco_ids = list(msg.data)

    def est_cb(self, msg: Pose2D):
        self.est_pose = msg

    def gt_cb(self, msg: Pose2D):
        self.gt_pose = msg

    def cmd_cb(self, msg: Twist):
        self.cmd_vel = msg

    def fmt_pose(self, pose):
        if pose is None:
            return "x=---   y=---   yaw=---"
        return f"x={pose.x: .3f}   y={pose.y: .3f}   yaw={math.degrees(pose.theta): .1f} deg"

    def draw(self):
        sys.stdout.write("\033[2J\033[H")  # clear terminal
        sys.stdout.write("=== SONDRE BOT TELEMETRY ===\n\n")

        sys.stdout.write(f"State:      {self.state if self.state is not None else '---'}\n")
        sys.stdout.write(f"ArUco IDs:  {self.aruco_ids if self.aruco_ids is not None else '---'}\n\n")

        sys.stdout.write(f"Estimate:   {self.fmt_pose(self.est_pose)}\n")
        sys.stdout.write(f"GroundTruth:{self.fmt_pose(self.gt_pose)}\n")

        if self.est_pose is not None and self.gt_pose is not None:
            dx = self.est_pose.x - self.gt_pose.x
            dy = self.est_pose.y - self.gt_pose.y
            dpos = math.hypot(dx, dy)
            dyaw = wrap_angle(self.est_pose.theta - self.gt_pose.theta)

            sys.stdout.write(
                f"Error:      dx={dx: .3f}   dy={dy: .3f}   pos={dpos: .3f} m   "
                f"dyaw={math.degrees(dyaw): .1f} deg\n"
            )
        else:
            sys.stdout.write("Error:      ---\n")

        if self.cmd_vel is not None:
            sys.stdout.write(
                f"Cmd Vel:    vx={self.cmd_vel.linear.x: .3f}   wz={self.cmd_vel.angular.z: .3f}\n"
            )
        else:
            sys.stdout.write("Cmd Vel:    ---\n")

        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryConsole()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()