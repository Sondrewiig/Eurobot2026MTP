import threading
import tkinter as tk

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import SetBool


class ModeSwitchGui(Node):
    def __init__(self):
        super().__init__('mode_switch_gui')

        self.client = self.create_client(SetBool, '/set_manual_mode')
        self.mode_sub = self.create_subscription(
            String, '/drive_mode', self.mode_callback, 10
        )

        self.current_mode = 'UNKNOWN'

        self.root = tk.Tk()
        self.root.title('Drive Mode')
        self.root.geometry('260x140')

        self.mode_label = tk.Label(
            self.root,
            text='Mode: UNKNOWN',
            font=('Arial', 16)
        )
        self.mode_label.pack(pady=10)

        self.auto_button = tk.Button(
            self.root,
            text='AUTO',
            width=12,
            height=2,
            command=self.set_auto
        )
        self.auto_button.pack(pady=5)

        self.manual_button = tk.Button(
            self.root,
            text='MANUAL',
            width=12,
            height=2,
            command=self.set_manual
        )
        self.manual_button.pack(pady=5)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def mode_callback(self, msg):
        self.current_mode = msg.data
        self.root.after(0, lambda: self.mode_label.config(text=f'Mode: {self.current_mode}'))

    def call_mode_service(self, manual: bool):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('/set_manual_mode service not available')
            return

        req = SetBool.Request()
        req.data = manual

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() is not None:
            self.get_logger().info(future.result().message)
        else:
            self.get_logger().warning('Service call failed')

    def set_auto(self):
        threading.Thread(target=self.call_mode_service, args=(False,), daemon=True).start()

    def set_manual(self):
        threading.Thread(target=self.call_mode_service, args=(True,), daemon=True).start()

    def on_close(self):
        self.root.quit()
        self.root.destroy()

    def run(self):
        def ros_spin():
            rclpy.spin(self)

        threading.Thread(target=ros_spin, daemon=True).start()
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = ModeSwitchGui()
    node.run()
    node.destroy_node()
    rclpy.shutdown()