#!/usr/bin/env python3

import math
import queue
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose2D, Quaternion
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Empty, Float32, Int32, Int32MultiArray, String

import serial
from serial import SerialException


def yaw_deg_to_quaternion_msg(yaw_deg: float) -> Quaternion:
    yaw_rad = math.radians(yaw_deg)
    half = 0.5 * yaw_rad
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


class OpenCRBridge(Node):
    def __init__(self) -> None:
        super().__init__("opencr_bridge")

        # ---------------- Parameters ----------------
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("telemetry_hz", 10)
        self.declare_parameter("reconnect_period_s", 1.0)
        self.declare_parameter("heartbeat_period_s", 2.0)
        self.declare_parameter("imu_frame_id", "imu_link")
        self.declare_parameter("publish_imu", True)

        self.declare_parameter("camera_pose_topic", "/bot_pose_estimate")
        self.declare_parameter("sync_camera_pose_to_opencr", True)
        self.declare_parameter("pose_sync_period_s", 0.5)
        self.declare_parameter("pose_sync_min_translation_m", 0.02)
        self.declare_parameter("pose_sync_min_yaw_deg", 3.0)
        self.declare_parameter("pose_sync_max_staleness_s", 1.5)

        self.camera_pose_topic: str = self.get_parameter("camera_pose_topic").value
        self.sync_camera_pose_to_opencr: bool = bool(self.get_parameter("sync_camera_pose_to_opencr").value)
        self.pose_sync_period_s: float = float(self.get_parameter("pose_sync_period_s").value)
        self.pose_sync_min_translation_m: float = float(self.get_parameter("pose_sync_min_translation_m").value)
        self.pose_sync_min_yaw_deg: float = float(self.get_parameter("pose_sync_min_yaw_deg").value)
        self.pose_sync_max_staleness_s: float = float(self.get_parameter("pose_sync_max_staleness_s").value)

        self.latest_camera_pose: Optional[Pose2D] = None
        self.latest_camera_pose_time: float = 0.0
        self.last_synced_camera_pose: Optional[tuple[float, float, float]] = None

        self.port: str = self.get_parameter("port").value
        self.baud: int = int(self.get_parameter("baud").value)
        self.telemetry_hz: int = int(self.get_parameter("telemetry_hz").value)
        self.reconnect_period_s: float = float(self.get_parameter("reconnect_period_s").value)
        self.heartbeat_period_s: float = float(self.get_parameter("heartbeat_period_s").value)
        self.imu_frame_id: str = self.get_parameter("imu_frame_id").value
        self.publish_imu: bool = bool(self.get_parameter("publish_imu").value)

        # ---------------- Serial state ----------------
        self._serial: Optional[serial.Serial] = None
        self._serial_lock = threading.Lock()
        self._reader_thread: Optional[threading.Thread] = None
        self._reader_running = False
        self._line_queue: "queue.Queue[str]" = queue.Queue(maxsize=1000)
        self._last_heartbeat_time = 0.0
        self._connected = False

        # ---------------- Publishers ----------------
        self.connected_pub = self.create_publisher(Bool, "/opencr/connected", 10)
        self.raw_line_pub = self.create_publisher(String, "/opencr/raw_line", 50)

        self.ack_pub = self.create_publisher(String, "/opencr/ack", 20)
        self.done_pub = self.create_publisher(String, "/opencr/done", 20)
        self.error_pub = self.create_publisher(String, "/opencr/error", 20)
        self.event_pub = self.create_publisher(String, "/opencr/event", 20)
        self.status_pub = self.create_publisher(String, "/opencr/status", 20)
        self.brick_state_pub = self.create_publisher(String, "/opencr/brick_state", 20)

        self.odom_pose_pub = self.create_publisher(Pose2D, "/opencr/odom_pose", 20)
        self.goal_pose_pub = self.create_publisher(Pose2D, "/opencr/goal_pose", 20)
        self.imu_yaw_deg_pub = self.create_publisher(Float32, "/opencr/imu_yaw_deg", 20)
        self.gyro_z_pub = self.create_publisher(Float32, "/opencr/gyro_z", 20)

        self.imu_pub = self.create_publisher(Imu, "/imu", 20)

        # ---------------- Subscribers ----------------
        self.create_subscription(Pose2D, "/opencr/cmd/go", self.go_cmd_cb, 10)
        self.create_subscription(Empty, "/opencr/cmd/go_home", self.go_home_cmd_cb, 10)
        self.create_subscription(Empty, "/opencr/cmd/stop", self.stop_cmd_cb, 10)
        self.create_subscription(Empty, "/opencr/cmd/estop", self.estop_cmd_cb, 10)
        self.create_subscription(Int32, "/opencr/cmd/flip", self.flip_cmd_cb, 10)
        self.create_subscription(Int32MultiArray, "/opencr/cmd/flip_seq", self.flip_seq_cmd_cb, 10)
        self.create_subscription(String, "/opencr/cmd/set_pattern", self.set_pattern_cmd_cb, 10)
        self.create_subscription(String, "/opencr/cmd/set_bricks", self.set_bricks_cmd_cb, 10)
        self.create_subscription(Pose2D, "/opencr/cmd/reset_odom", self.reset_odom_cmd_cb, 10)
        self.create_subscription(Pose2D, "/opencr/cmd/set_home", self.set_home_cmd_cb, 10)
        self.create_subscription(Empty, "/opencr/cmd/get_state", self.get_state_cmd_cb, 10)
        self.create_subscription(Int32, "/opencr/cmd/telemetry_hz", self.telemetry_hz_cmd_cb, 10)
        self.create_subscription(Pose2D, self.camera_pose_topic, self.camera_pose_cb, 10)
        self.pose_sync_timer = self.create_timer(self.pose_sync_period_s, self.sync_camera_pose_timer_cb)

        # ---------------- Timers ----------------
        self.process_timer = self.create_timer(0.02, self.process_serial_lines)
        self.reconnect_timer = self.create_timer(self.reconnect_period_s, self.ensure_connection)
        self.heartbeat_timer = self.create_timer(self.heartbeat_period_s, self.send_heartbeat)

        self.get_logger().info(
            f"opencr_bridge starting on {self.port} @ {self.baud} baud"
        )

        self.publish_connected(False)
        self.ensure_connection()

    # =========================================================
    # Serial connection management
    # =========================================================
    def publish_connected(self, state: bool) -> None:
        if self._connected == state:
            return
        self._connected = state
        msg = Bool()
        msg.data = state
        self.connected_pub.publish(msg)

    def ensure_connection(self) -> None:
        if self._serial is not None and self._serial.is_open:
            return

        try:
            ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=0.1,
                write_timeout=0.2,
            )
            time.sleep(0.2)

            with self._serial_lock:
                self._serial = ser

            self._reader_running = True
            self._reader_thread = threading.Thread(target=self.reader_loop, daemon=True)
            self._reader_thread.start()

            self.publish_connected(True)
            self.get_logger().info(f"Connected to OpenCR on {self.port}")

            self.send_line("PING")
            self.send_line(f"TELEM {self.telemetry_hz}")
            self.send_line("GET_STATE")
            self._last_heartbeat_time = time.time()

        except Exception as e:
            self.publish_connected(False)
            self.get_logger().warning(f"OpenCR not connected on {self.port}: {e}")

    def close_serial(self) -> None:
        self._reader_running = False

        with self._serial_lock:
            ser = self._serial
            self._serial = None

        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

        self.publish_connected(False)

    def reader_loop(self) -> None:
        while self._reader_running and rclpy.ok():
            try:
                with self._serial_lock:
                    ser = self._serial

                if ser is None:
                    break

                raw = ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                try:
                    self._line_queue.put_nowait(line)
                except queue.Full:
                    self.get_logger().warning("OpenCR line queue full, dropping line")

            except SerialException as e:
                self.get_logger().error(f"Serial read error: {e}")
                self.close_serial()
                break
            except Exception as e:
                self.get_logger().error(f"Reader loop error: {e}")
                self.close_serial()
                break

    def send_line(self, line: str) -> bool:
        try:
            with self._serial_lock:
                ser = self._serial
                if ser is None or not ser.is_open:
                    self.get_logger().warning(f"Cannot send, OpenCR not connected: {line}")
                    return False

                ser.write((line.strip() + "\n").encode("utf-8"))
                ser.flush()
            return True

        except Exception as e:
            self.get_logger().error(f"Serial write failed for '{line}': {e}")
            self.close_serial()
            return False

    def send_heartbeat(self) -> None:
        if self._serial is None:
            return
        now = time.time()
        if now - self._last_heartbeat_time >= self.heartbeat_period_s:
            self.send_line("PING")
            self._last_heartbeat_time = now

    # =========================================================
    # Incoming serial parsing
    # =========================================================
    def process_serial_lines(self) -> None:
        processed = 0
        while processed < 100:
            try:
                line = self._line_queue.get_nowait()
            except queue.Empty:
                break

            self.handle_line(line)
            processed += 1

    def handle_line(self, line: str) -> None:
        self.publish_string(self.raw_line_pub, line)

        if line.startswith("ACK "):
            self.publish_string(self.ack_pub, line[4:])
            return

        if line.startswith("DONE "):
            self.publish_string(self.done_pub, line[5:])
            return

        if line.startswith("ERR "):
            self.publish_string(self.error_pub, line[4:])
            return

        if line.startswith("EVENT "):
            self.publish_string(self.event_pub, line[6:])
            return

        if line.startswith("TEL "):
            self.handle_tel_line(line[4:])
            return

    def handle_tel_line(self, payload: str) -> None:
        parts = payload.split()
        if not parts:
            return

        kind = parts[0]

        if kind == "ODOM" and len(parts) >= 4:
            try:
                x_mm = float(parts[1])
                y_mm = float(parts[2])
                yaw_deg = float(parts[3])

                msg = Pose2D()
                msg.x = x_mm / 1000.0
                msg.y = y_mm / 1000.0
                msg.theta = math.radians(yaw_deg)
                self.odom_pose_pub.publish(msg)
            except Exception as e:
                self.get_logger().warning(f"Failed parsing ODOM: {payload} ({e})")
            return

        if kind == "IMU" and len(parts) >= 3:
            try:
                yaw_deg = float(parts[1])
                gz = float(parts[2])

                yaw_msg = Float32()
                yaw_msg.data = yaw_deg
                self.imu_yaw_deg_pub.publish(yaw_msg)

                gz_msg = Float32()
                gz_msg.data = gz
                self.gyro_z_pub.publish(gz_msg)

                if self.publish_imu:
                    imu = Imu()
                    imu.header.stamp = self.get_clock().now().to_msg()
                    imu.header.frame_id = self.imu_frame_id
                    imu.orientation = yaw_deg_to_quaternion_msg(yaw_deg)
                    imu.angular_velocity.z = gz
                    imu.linear_acceleration.x = 0.0
                    imu.linear_acceleration.y = 0.0
                    imu.linear_acceleration.z = 0.0
                    self.imu_pub.publish(imu)
            except Exception as e:
                self.get_logger().warning(f"Failed parsing IMU: {payload} ({e})")
            return

        if kind == "STATUS" and len(parts) >= 2:
            self.publish_string(self.status_pub, " ".join(parts[1:]))
            return

        if kind == "GOAL" and len(parts) >= 4:
            try:
                x_mm = float(parts[1])
                y_mm = float(parts[2])
                yaw_deg = float(parts[3])

                msg = Pose2D()
                msg.x = x_mm / 1000.0
                msg.y = y_mm / 1000.0
                msg.theta = math.radians(yaw_deg)
                self.goal_pose_pub.publish(msg)
            except Exception as e:
                self.get_logger().warning(f"Failed parsing GOAL: {payload} ({e})")
            return

        if kind == "BRICKS" and len(parts) >= 2:
            self.publish_string(self.brick_state_pub, " ".join(parts[1:]))
            return

    # =========================================================
    # ROS -> serial command callbacks
    # =========================================================
    def go_cmd_cb(self, msg: Pose2D) -> None:
        x_mm = msg.x * 1000.0
        y_mm = msg.y * 1000.0
        yaw_deg = math.degrees(msg.theta)
        self.send_line(f"GO {x_mm:.1f} {y_mm:.1f} {yaw_deg:.1f}")

    def go_home_cmd_cb(self, _msg: Empty) -> None:
        self.send_line("GO_HOME")

    def stop_cmd_cb(self, _msg: Empty) -> None:
        self.send_line("STOP")

    def estop_cmd_cb(self, _msg: Empty) -> None:
        self.send_line("ESTOP")

    def flip_cmd_cb(self, msg: Int32) -> None:
        self.send_line(f"FLIP {int(msg.data)}")

    def flip_seq_cmd_cb(self, msg: Int32MultiArray) -> None:
        if len(msg.data) == 0:
            self.get_logger().warning("Ignoring empty flip sequence")
            return
        csv = ",".join(str(int(v)) for v in msg.data)
        self.send_line(f"FLIP_SEQ {csv}")

    def set_pattern_cmd_cb(self, msg: String) -> None:
        payload = msg.data.strip()
        if not payload:
            self.get_logger().warning("Ignoring empty SET_PATTERN")
            return
        self.send_line(f"SET_PATTERN {payload}")

    def set_bricks_cmd_cb(self, msg: String) -> None:
        payload = msg.data.strip()
        if not payload:
            self.get_logger().warning("Ignoring empty SET_BRICKS")
            return
        self.send_line(f"SET_BRICKS {payload}")

    def reset_odom_cmd_cb(self, msg: Pose2D) -> None:
        x_mm = msg.x * 1000.0
        y_mm = msg.y * 1000.0
        yaw_deg = math.degrees(msg.theta)
        self.send_line(f"RESET_ODOM {x_mm:.1f} {y_mm:.1f} {yaw_deg:.1f}")

    def set_home_cmd_cb(self, msg: Pose2D) -> None:
        x_mm = msg.x * 1000.0
        y_mm = msg.y * 1000.0
        yaw_deg = math.degrees(msg.theta)
        self.send_line(f"SET_HOME {x_mm:.1f} {y_mm:.1f} {yaw_deg:.1f}")

    def get_state_cmd_cb(self, _msg: Empty) -> None:
        self.send_line("GET_STATE")

    def telemetry_hz_cmd_cb(self, msg: Int32) -> None:
        hz = int(msg.data)
        if hz < 0:
            hz = 0
        self.send_line(f"TELEM {hz}")

    def camera_pose_cb(self, msg: Pose2D) -> None:
        self.latest_camera_pose = msg
        self.latest_camera_pose_time = time.time()

    def angle_diff_deg(self, a: float, b: float) -> float:
        d = a - b
        while d > 180.0:
            d -= 360.0
        while d < -180.0:
            d += 360.0
        return d

    def sync_camera_pose_timer_cb(self) -> None:
        if not self.sync_camera_pose_to_opencr:
            return
        if not self._connected:
            return
        if self.latest_camera_pose is None:
            return
        if time.time() - self.latest_camera_pose_time > self.pose_sync_max_staleness_s:
            return

        x_m = float(self.latest_camera_pose.x)
        y_m = float(self.latest_camera_pose.y)
        yaw_deg = math.degrees(float(self.latest_camera_pose.theta))

        if self.last_synced_camera_pose is not None:
            last_x_m, last_y_m, last_yaw_deg = self.last_synced_camera_pose
            dpos_m = math.hypot(x_m - last_x_m, y_m - last_y_m)
            dyaw_deg = abs(self.angle_diff_deg(yaw_deg, last_yaw_deg))

            if dpos_m < self.pose_sync_min_translation_m and dyaw_deg < self.pose_sync_min_yaw_deg:
                return

        x_mm = x_m * 1000.0
        y_mm = y_m * 1000.0

        if self.send_line(f"RESET_ODOM {x_mm:.1f} {y_mm:.1f} {yaw_deg:.1f}"):
            self.last_synced_camera_pose = (x_m, y_m, yaw_deg)

    # =========================================================
    # Helpers
    # =========================================================
    def publish_string(self, pub, text: str) -> None:
        msg = String()
        msg.data = text
        pub.publish(msg)

    def destroy_node(self) -> bool:
        self.close_serial()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OpenCRBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()