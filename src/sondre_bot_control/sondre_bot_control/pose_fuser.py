import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def compose_pose(a: Pose2D, b: Pose2D) -> Pose2D:
    out = Pose2D()
    ca = math.cos(a.theta)
    sa = math.sin(a.theta)
    out.x = a.x + ca * b.x - sa * b.y
    out.y = a.y + sa * b.x + ca * b.y
    out.theta = wrap_angle(a.theta + b.theta)
    return out


def inverse_pose(p: Pose2D) -> Pose2D:
    out = Pose2D()
    c = math.cos(p.theta)
    s = math.sin(p.theta)
    out.x = -(c * p.x + s * p.y)
    out.y = -(-s * p.x + c * p.y)
    out.theta = wrap_angle(-p.theta)
    return out


def blend_scalar(a: float, b: float, alpha: float) -> float:
    return (1.0 - alpha) * a + alpha * b


def blend_angle(a: float, b: float, alpha: float) -> float:
    return wrap_angle(a + alpha * wrap_angle(b - a))


def blend_pose(a: Pose2D, b: Pose2D, alpha: float) -> Pose2D:
    out = Pose2D()
    out.x = blend_scalar(a.x, b.x, alpha)
    out.y = blend_scalar(a.y, b.y, alpha)
    out.theta = blend_angle(a.theta, b.theta, alpha)
    return out


class PoseFuser(Node):
    def __init__(self):
        super().__init__("pose_fuser")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("aruco_topic", "/bot_pose_estimate")
        self.declare_parameter("fused_topic", "/bot_pose_fused")
        self.declare_parameter("status_topic", "/localization_status")

        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("aruco_timeout", 0.35)
        self.declare_parameter("offset_alpha", 0.20)
        self.declare_parameter("imu_yaw_weight", 0.25)

        odom_topic = self.get_parameter("odom_topic").value
        imu_topic = self.get_parameter("imu_topic").value
        aruco_topic = self.get_parameter("aruco_topic").value
        fused_topic = self.get_parameter("fused_topic").value
        status_topic = self.get_parameter("status_topic").value

        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.aruco_timeout = float(self.get_parameter("aruco_timeout").value)
        self.offset_alpha = float(self.get_parameter("offset_alpha").value)
        self.imu_yaw_weight = float(self.get_parameter("imu_yaw_weight").value)

        self.odom_pose = None              # pose in odom frame
        self.aruco_pose = None             # pose in field frame
        self.last_aruco_time = None

        self.field_from_odom = None        # transform: field <- odom

        self.imu_integrated_yaw = None
        self.last_imu_time = None

        self.pose_pub = self.create_publisher(Pose2D, fused_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)

        self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)
        self.create_subscription(Imu, imu_topic, self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(Pose2D, aruco_topic, self.aruco_cb, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_fused_pose)

        self.get_logger().info(
            f"pose_fuser started | odom={odom_topic} | imu={imu_topic} | "
            f"aruco={aruco_topic} | out={fused_topic}"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def aruco_is_fresh(self) -> bool:
        if self.last_aruco_time is None:
            return False
        return (self.now_sec() - self.last_aruco_time) <= self.aruco_timeout

    def get_local_pose(self) -> Pose2D | None:
        if self.odom_pose is None:
            return None

        local = Pose2D()
        local.x = self.odom_pose.x
        local.y = self.odom_pose.y
        local.theta = self.odom_pose.theta

        if self.imu_integrated_yaw is not None:
            local.theta = blend_angle(
                self.odom_pose.theta,
                self.imu_integrated_yaw,
                self.imu_yaw_weight
            )

        return local

    def odom_cb(self, msg: Odometry):
        p = Pose2D()
        p.x = float(msg.pose.pose.position.x)
        p.y = float(msg.pose.pose.position.y)
        p.theta = quat_to_yaw(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        self.odom_pose = p

        if self.imu_integrated_yaw is None:
            self.imu_integrated_yaw = p.theta

    def imu_cb(self, msg: Imu):
        now = self.now_sec()

        if self.last_imu_time is not None and self.imu_integrated_yaw is not None:
            dt = now - self.last_imu_time
            if 0.0 < dt < 0.2:
                self.imu_integrated_yaw = wrap_angle(
                    self.imu_integrated_yaw + float(msg.angular_velocity.z) * dt
                )

        self.last_imu_time = now

    def aruco_cb(self, msg: Pose2D):
        self.aruco_pose = msg
        self.last_aruco_time = self.now_sec()

        local_pose = self.get_local_pose()
        if local_pose is None:
            return

        target_field_from_odom = compose_pose(msg, inverse_pose(local_pose))

        if self.field_from_odom is None:
            self.field_from_odom = target_field_from_odom
            self.get_logger().info("pose_fuser initialized field<-odom transform from ArUco")
        else:
            self.field_from_odom = blend_pose(
                self.field_from_odom,
                target_field_from_odom,
                self.offset_alpha
            )

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def publish_fused_pose(self):
        local_pose = self.get_local_pose()

        if local_pose is None:
            if self.aruco_pose is not None and self.aruco_is_fresh():
                self.pose_pub.publish(self.aruco_pose)
                self.publish_status("ARUCO_ONLY")
            else:
                self.publish_status("WAITING_FOR_ODOM_OR_TAG")
            return

        if self.field_from_odom is None:
            if self.aruco_pose is not None and self.aruco_is_fresh():
                self.pose_pub.publish(self.aruco_pose)
                self.publish_status("ARUCO_ONLY")
            else:
                self.publish_status("WAITING_FOR_INITIAL_TAG")
            return

        fused = compose_pose(self.field_from_odom, local_pose)
        self.pose_pub.publish(fused)

        if self.aruco_is_fresh():
            self.publish_status("ODOM_IMU_PLUS_TAG")
        else:
            self.publish_status("ODOM_IMU_ONLY")


def main(args=None):
    rclpy.init(args=args)
    node = PoseFuser()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()