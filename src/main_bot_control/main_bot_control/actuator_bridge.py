import queue
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node

import serial

from std_msgs.msg import Bool, Int32, Int32MultiArray, String


class ActuatorBridge(Node):
    def __init__(self):
        super().__init__("actuator_bridge")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)

        self.ser: Optional[serial.Serial] = None
        self.lock = threading.Lock()
        self.reader_running = False
        self.reader_thread: Optional[threading.Thread] = None

        self.connected_pub = self.create_publisher(Bool, "/actuator/connected", 10)
        self.raw_line_pub = self.create_publisher(String, "/actuator/raw_line", 50)
        self.ack_pub = self.create_publisher(String, "/actuator/ack", 20)
        self.error_pub = self.create_publisher(String, "/actuator/error", 20)

        self.create_subscription(String, "/actuator/cmd/raw", self.raw_cb, 10)
        self.create_subscription(Int32, "/actuator/cmd/flip", self.flip_cb, 10)
        self.create_subscription(Int32MultiArray, "/actuator/cmd/flip_seq", self.flip_seq_cb, 10)

        self.get_logger().info(f"actuator_bridge starting on {self.port} @ {self.baud}")
        self.create_timer(1.0, self.ensure_connected)

    def publish_connected(self, value: bool):
        msg = Bool()
        msg.data = value
        try:
            if rclpy.ok():
                self.connected_pub.publish(msg)
        except Exception:
            pass

    def ensure_connected(self):
        with self.lock:
            if self.ser is not None and self.ser.is_open:
                self.publish_connected(True)
                return

            try:
                self.ser = serial.Serial(
                    port=self.port,
                    baudrate=self.baud,
                    timeout=0.1,
                    write_timeout=0.2,
                )

                # Avoid auto-reset glitches on ESP32 USB serial.
                self.ser.dtr = False
                self.ser.rts = False

                time.sleep(0.2)
                self.publish_connected(True)
                self.get_logger().info(f"Connected to actuator ESP on {self.port}")

                if not self.reader_running:
                    self.reader_running = True
                    self.reader_thread = threading.Thread(target=self.reader_loop, daemon=True)
                    self.reader_thread.start()

            except Exception as e:
                self.publish_connected(False)
                self.get_logger().warning(f"Actuator ESP not connected on {self.port}: {e}")

    def reader_loop(self):
        while self.reader_running and rclpy.ok():
            try:
                with self.lock:
                    ser = self.ser

                if ser is None or not ser.is_open:
                    time.sleep(0.1)
                    continue

                raw = ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                msg = String()
                msg.data = line
                self.raw_line_pub.publish(msg)

                if line.startswith("ACK"):
                    self.ack_pub.publish(msg)
                elif line.startswith("ERR") or "BAD" in line.upper():
                    self.error_pub.publish(msg)

            except Exception as e:
                self.get_logger().error(f"Actuator serial read error: {e}")
                with self.lock:
                    try:
                        if self.ser:
                            self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
                self.publish_connected(False)
                time.sleep(0.5)

    def send_line(self, line: str):
        line = line.strip()
        if not line:
            return

        self.ensure_connected()

        with self.lock:
            ser = self.ser

            if ser is None or not ser.is_open:
                self.get_logger().warning(f"Cannot send actuator command, not connected: {line}")
                return

            try:
                ser.write((line + "\n").encode())
                ser.flush()
                self.get_logger().info(f"> actuator: {line}")
            except Exception as e:
                self.get_logger().error(f"Actuator serial write error: {e}")
                try:
                    ser.close()
                except Exception:
                    pass
                self.ser = None
                self.publish_connected(False)

    def raw_cb(self, msg: String):
        cmd = msg.data.strip()

        # Compatibility mappings from old GUI names to actuator ESP commands.
        upper = cmd.upper()

        if upper.startswith("FLIP_SEQ"):
            tail = cmd.split(maxsplit=1)[1] if len(cmd.split(maxsplit=1)) > 1 else ""
            nums = [x for x in tail.replace(",", " ").split() if x.isdigit()]
            if nums:
                self.send_line("FLIP_SEQ " + ",".join(nums))
            return

        if upper.startswith("FLIP "):
            n = int(cmd.split()[1])
            self.send_line(f"F{n}")
            return

        if upper == "CARWASH_SPIN_POSITIVE":
            self.send_line("VF")
            return

        if upper == "CARWASH_SPIN_NEGATIVE":
            self.send_line("VB")
            return

        if upper == "CARWASH_SPIN_STOP":
            self.send_line("VN")
            return

        if upper.startswith("CARWASH_ARM"):
            self.send_line(cmd)
            return

        self.send_line(cmd)

    def flip_cb(self, msg: Int32):
        n = int(msg.data)
        if 1 <= n <= 4:
            self.send_line(f"F{n}")
        else:
            self.get_logger().warning(f"Ignoring invalid flipper index: {n}")

    def flip_seq_cb(self, msg: Int32MultiArray):
        nums = []
        for n in msg.data:
            n = int(n)
            if 1 <= n <= 4:
                nums.append(str(n))

        if nums:
            self.send_line("FLIP_SEQ " + ",".join(nums))

    def destroy_node(self):
        self.reader_running = False
        with self.lock:
            try:
                if self.ser:
                    self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
