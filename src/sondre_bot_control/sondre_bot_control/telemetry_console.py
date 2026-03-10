import math
import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String, Int32MultiArray
from std_srvs.srv import SetBool
from geometry_msgs.msg import Pose2D, Twist
from sensor_msgs.msg import Imu


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))

def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class TelemetryConsole(Node):
    def __init__(self):
        super().__init__("telemetry_console")

        # ---------------- ROS state ----------------
        self.state = None
        self.aruco_ids = None
        self.est_pose = None
        self.gt_pose = None
        self.cmd_vel = None
        self.drive_mode = "UNKNOWN"
        self.imu_msg = None
        
        self.overhead_pose = None
        
        self.create_subscription(
            Pose2D,
            "/vision/robot_pose",
            self.overhead_pose_callback,
            10,
        )
                
        self.selected_tag = None

        self.marker_names = {
            20: "Yellow_close",
            21: "Blue_close",
            22: "Yellow_far",
            23: "Blue_far",
        }
                
        self.fused_pose = None
        self.localization_status = None

        # Manual driving parameters
        self.manual_linear = 0.35
        self.manual_angular = 1.2
        self.current_manual_cmd = Twist()
        self.active_keys = set()

        # ---------------- ROS interfaces ----------------
        self.create_subscription(String, "/bot_state", self.state_cb, 10)
        self.create_subscription(Int32MultiArray, "/aruco_ids", self.aruco_cb, 10)
        self.create_subscription(Pose2D, "/bot_pose_estimate", self.est_cb, 10)
        self.create_subscription(Pose2D, "/bot_pose_ground_truth", self.gt_cb, 10)
        self.create_subscription(Pose2D, "/bot_pose_fused", self.fused_cb, 10)
        self.create_subscription(String, "/localization_status", self.loc_status_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)
        self.create_subscription(String, "/drive_mode", self.drive_mode_cb, 10)
        self.create_subscription(Imu, "/imu", self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(String, "/aruco_selected_tag", self.selected_tag_cb, 10)

        self.manual_pub = self.create_publisher(Twist, "/cmd_vel_manual", 10)
        self.mode_client = self.create_client(SetBool, "/set_manual_mode")

        # Publish manual command repeatedly while a button/key is held
        self.manual_timer = self.create_timer(0.10, self.publish_manual_cmd)

        # ---------------- GUI ----------------
        self.root = tk.Tk()
        self.root.title("Sondre Telemetry")
        self.root.geometry("900x620")
        self.root.configure(padx=12, pady=12)

        # Top status row
        top_frame = tk.Frame(self.root)
        top_frame.pack(fill="x", pady=(0, 10))
        

        self.mode_label = tk.Label(
            top_frame, text="Mode: UNKNOWN", font=("Arial", 16, "bold")
        )
        self.mode_label.pack(side="left")

        button_frame = tk.Frame(top_frame)
        button_frame.pack(side="right")

        self.auto_button = tk.Button(
            button_frame, text="AUTO", width=10, command=self.set_auto
        )
        self.auto_button.pack(side="left", padx=4)

        self.manual_button = tk.Button(
            button_frame, text="MANUAL", width=10, command=self.set_manual
        )
        self.manual_button.pack(side="left", padx=4)

        # Main split
        main_frame = tk.Frame(self.root)
        main_frame.pack(fill="both", expand=True)

        # Telemetry text
        telemetry_frame = tk.LabelFrame(main_frame, text="Telemetry", padx=8, pady=8)
        telemetry_frame.pack(side="left", fill="both", expand=True, padx=(0, 10))

        self.text = tk.Text(
            telemetry_frame,
            width=72,
            height=28,
            font=("Courier New", 12),
            state="disabled",
        )
        self.text.pack(fill="both", expand=True)

        # Controls panel
        control_frame = tk.LabelFrame(main_frame, text="Manual Drive", padx=10, pady=10)
        control_frame.pack(side="right", fill="y")

        speed_frame = tk.Frame(control_frame)
        speed_frame.pack(pady=(0, 10))

        tk.Label(speed_frame, text="Linear speed").grid(row=0, column=0, sticky="w")
        self.linear_scale = tk.Scale(
            speed_frame,
            from_=0.1,
            to=1.0,
            resolution=0.05,
            orient="horizontal",
            length=180,
            command=self.on_linear_scale,
        )
        self.linear_scale.set(self.manual_linear)
        self.linear_scale.grid(row=1, column=0, padx=4, pady=(0, 8))

        tk.Label(speed_frame, text="Angular speed").grid(row=2, column=0, sticky="w")
        self.angular_scale = tk.Scale(
            speed_frame,
            from_=0.2,
            to=3.0,
            resolution=0.1,
            orient="horizontal",
            length=180,
            command=self.on_angular_scale,
        )
        self.angular_scale.set(self.manual_angular)
        self.angular_scale.grid(row=3, column=0, padx=4)

        drive_frame = tk.Frame(control_frame)
        drive_frame.pack(pady=8)

        self.btn_forward = tk.Button(drive_frame, text="W / Forward", width=14, height=2)
        self.btn_left = tk.Button(drive_frame, text="A / Left", width=14, height=2)
        self.btn_stop = tk.Button(drive_frame, text="STOP", width=14, height=2)
        self.btn_right = tk.Button(drive_frame, text="D / Right", width=14, height=2)
        self.btn_back = tk.Button(drive_frame, text="S / Back", width=14, height=2)

        self.btn_forward.grid(row=0, column=1, padx=4, pady=4)
        self.btn_left.grid(row=1, column=0, padx=4, pady=4)
        self.btn_stop.grid(row=1, column=1, padx=4, pady=4)
        self.btn_right.grid(row=1, column=2, padx=4, pady=4)
        self.btn_back.grid(row=2, column=1, padx=4, pady=4)

        help_label = tk.Label(
            control_frame,
            text="Click this window, then use WASD.\nRelease key/button to stop.",
            justify="left",
        )
        help_label.pack(pady=(10, 0))

        # Button press/release bindings
        self.bind_drive_button(self.btn_forward, "w")
        self.bind_drive_button(self.btn_left, "a")
        self.bind_drive_button(self.btn_right, "d")
        self.bind_drive_button(self.btn_back, "s")

        self.btn_stop.bind("<ButtonPress-1>", lambda event: self.stop_manual())

        # Keyboard bindings
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)
        self.root.bind("<FocusOut>", lambda event: self.stop_manual())

        # Refresh GUI telemetry
        self.root.after(200, self.refresh_gui)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.get_logger().info("Telemetry GUI started")

    # ---------------- ROS callbacks ----------------
    def state_cb(self, msg: String):
        self.state = msg.data

    def aruco_cb(self, msg: Int32MultiArray):
        self.aruco_ids = list(msg.data)

    def est_cb(self, msg: Pose2D):
        self.est_pose = msg
    
    def fused_cb(self, msg: Pose2D):
        self.fused_pose = msg
        
    def overhead_pose_callback(self, msg: Pose2D):
        self.overhead_pose = msg

    def loc_status_cb(self, msg: String):
        self.localization_status = msg.data
    
    def gt_cb(self, msg: Pose2D):
        self.gt_pose = msg

    def cmd_cb(self, msg: Twist):
        self.cmd_vel = msg

    def drive_mode_cb(self, msg: String):
        self.drive_mode = msg.data
        self.root.after(0, self.update_mode_label)

    def imu_cb(self, msg: Imu):
        self.imu_msg = msg
        
    def selected_tag_cb(self, msg: String):
        self.selected_tag = msg.data

    # ---------------- Formatting ----------------
    def fmt_pose(self, pose):
        if pose is None:
            return "x=---   y=---   yaw=---"
        return f"x={pose.x: .3f}   y={pose.y: .3f}   yaw={math.degrees(pose.theta): .1f} deg"

    def fmt5(self, value):
        return f"{value:.5f}"

    def imu_status(self):
        if self.imu_msg is None:
            return "---"

        gz = abs(self.imu_msg.angular_velocity.z)
        ax = abs(self.imu_msg.linear_acceleration.x)
        ay = abs(self.imu_msg.linear_acceleration.y)

        if gz > 0.15:
            return "TURNING"
        if ax > 0.20 or ay > 0.20:
            return "ACCELERATING"
        return "STEADY"
    
    def fmt_aruco_ids(self):
        if self.aruco_ids is None:
            return "---"
        if len(self.aruco_ids) == 0:
            return "none"

        parts = []
        for marker_id in self.aruco_ids:
            name = self.marker_names.get(marker_id, f"id_{marker_id}")
            parts.append(f"{marker_id} ({name})")
        return ", ".join(parts)
    
    


    def build_telemetry_text(self):
        lines = []
        lines.append("=== SONDRE BOT TELEMETRY ===")
        lines.append("")
        lines.append(f"Mode:       {self.drive_mode}")
        lines.append(f"State:      {self.state if self.state is not None else '---'}")
        lines.append(f"ArUco IDs:  {self.fmt_aruco_ids()}")
        lines.append(f"ArUco Use:  {self.selected_tag if self.selected_tag is not None else '---'}")
        lines.append("")

        lines.append(f"Estimate:   {self.fmt_pose(self.fused_pose)}")
        lines.append(f"ArUco Pose: {self.fmt_pose(self.est_pose)}")
        lines.append(f"Overhead:   {self.fmt_pose(self.overhead_pose)}")
        lines.append(
            f"Loc Status: {self.localization_status if self.localization_status is not None else '---'}"
        )
        lines.append(f"GroundTruth:{self.fmt_pose(self.gt_pose)}")

        if self.fused_pose is not None and self.gt_pose is not None:
            dx = self.fused_pose.x - self.gt_pose.x
            dy = self.fused_pose.y - self.gt_pose.y
            dpos = math.hypot(dx, dy)
            dyaw = wrap_angle(self.fused_pose.theta - self.gt_pose.theta)
            lines.append(
                f"Error:      dx={dx: .3f}   dy={dy: .3f}   pos={dpos: .3f} m   "
                f"dyaw={math.degrees(dyaw): .1f} deg"
            )
        else:
            lines.append("Error:      ---")

        if self.cmd_vel is not None:
            lines.append(
                f"Cmd Vel:    vx={self.cmd_vel.linear.x: .3f}   wz={self.cmd_vel.angular.z: .3f}"
            )
        else:
            lines.append("Cmd Vel:    ---")
        
                
        

        
        lines.append("")
        lines.append("IMU:")
        if self.imu_msg is not None:
            lines.append(
                f"  Accel:    ax={self.fmt5(self.imu_msg.linear_acceleration.x)}   "
                f"ay={self.fmt5(self.imu_msg.linear_acceleration.y)}   "
                f"az={self.fmt5(self.imu_msg.linear_acceleration.z)}"
            )
            lines.append(
                f"  Gyro:     gx={self.fmt5(self.imu_msg.angular_velocity.x)}   "
                f"gy={self.fmt5(self.imu_msg.angular_velocity.y)}   "
                f"gz={self.fmt5(self.imu_msg.angular_velocity.z)}"
            )
            lines.append(f"  Status:   {self.imu_status()}")
        else:
            lines.append("  ---")

        return "\n".join(lines)

    # ---------------- GUI updates ----------------
    def update_mode_label(self):
        self.mode_label.config(text=f"Mode: {self.drive_mode}")

    def refresh_gui(self):
        self.update_mode_label()

        text = self.build_telemetry_text()
        self.text.config(state="normal")
        self.text.delete("1.0", tk.END)
        self.text.insert(tk.END, text)
        self.text.config(state="disabled")

        self.root.after(200, self.refresh_gui)

    # ---------------- Mode switching ----------------
    def set_auto(self):
        threading.Thread(target=self.call_mode_service, args=(False,), daemon=True).start()

    def set_manual(self):
        threading.Thread(target=self.call_mode_service, args=(True,), daemon=True).start()

    def call_mode_service(self, manual: bool):
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("/set_manual_mode service not available")
            return

        req = SetBool.Request()
        req.data = manual
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() is not None:
            self.get_logger().info(future.result().message)
        else:
            self.get_logger().warning("Mode service call failed")

    # ---------------- Manual driving ----------------
    def on_linear_scale(self, value):
        self.manual_linear = float(value)
        self.recompute_manual_cmd()

    def on_angular_scale(self, value):
        self.manual_angular = float(value)
        self.recompute_manual_cmd()

    def bind_drive_button(self, button, key_name):
        button.bind("<ButtonPress-1>", lambda event: self.press_drive(key_name))
        button.bind("<ButtonRelease-1>", lambda event: self.release_drive(key_name))

    def on_key_press(self, event):
        key = event.keysym.lower()
        if key in ("w", "a", "s", "d"):
            self.press_drive(key)

    def on_key_release(self, event):
        key = event.keysym.lower()
        if key in ("w", "a", "s", "d"):
            self.release_drive(key)

    def press_drive(self, key):
        self.active_keys.add(key)
        self.recompute_manual_cmd()

    def release_drive(self, key):
        if key in self.active_keys:
            self.active_keys.remove(key)
        self.recompute_manual_cmd()
        

    def recompute_manual_cmd(self):
        linear = 0.0
        angular = 0.0

        if "w" in self.active_keys and "s" not in self.active_keys:
            linear += self.manual_linear
        if "s" in self.active_keys and "w" not in self.active_keys:
            linear -= self.manual_linear
        if "a" in self.active_keys and "d" not in self.active_keys:
            angular += self.manual_angular
        if "d" in self.active_keys and "a" not in self.active_keys:
            angular -= self.manual_angular

        self.current_manual_cmd.linear.x = linear
        self.current_manual_cmd.angular.z = angular

        if linear == 0.0 and angular == 0.0:
            self.publish_zero_once()

    def publish_zero_once(self):
        msg = Twist()
        self.manual_pub.publish(msg)

    def stop_manual(self):
        self.active_keys.clear()
        self.recompute_manual_cmd()

    def publish_manual_cmd(self):
        if self.drive_mode == "MANUAL":
            self.manual_pub.publish(self.current_manual_cmd)

    # ---------------- Run / shutdown ----------------
    def on_close(self):
        self.stop_manual()
        self.root.quit()
        self.root.destroy()

    def run(self):
        ros_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        ros_thread.start()
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryConsole()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()