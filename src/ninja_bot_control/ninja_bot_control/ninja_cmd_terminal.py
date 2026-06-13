#!/usr/bin/env python3

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class NinjaCmdTerminal(Node):
    def __init__(self):
        super().__init__("ninja_cmd_terminal")

        self.cmd_pub = self.create_publisher(
            String,
            "/ninja/esp32_cmd",
            10,
        )

        self.telemetry_sub = self.create_subscription(
            String,
            "/ninja/telemetry",
            self.telemetry_callback,
            10,
        )

        self.get_logger().info("Ninja command terminal started")
        self.get_logger().info("Make sure esp32_bridge is running in another terminal.")
        self.get_logger().info("Type: ping, settings, stop, m1 60, motors 80 80")
        self.get_logger().info("Type: exit or quit to close")

        self.running = True

    def telemetry_callback(self, msg):
        print(f"\nESP32: {msg.data}")
        print("ninja> ", end="", flush=True)

    def publish_command(self, command: str):
        command = command.strip()

        if not command:
            return

        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)

        print(f"PI -> ESP32: {command}")


def input_loop(node: NinjaCmdTerminal):
    while rclpy.ok() and node.running:
        try:
            command = input("ninja> ").strip()

            if command in ["exit", "quit"]:
                node.running = False
                rclpy.shutdown()
                return

            node.publish_command(command)

        except KeyboardInterrupt:
            node.running = False
            rclpy.shutdown()
            return

        except EOFError:
            node.running = False
            rclpy.shutdown()
            return


def main(args=None):
    rclpy.init(args=args)

    node = NinjaCmdTerminal()

    thread = threading.Thread(
        target=input_loop,
        args=(node,),
        daemon=True,
    )
    thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()