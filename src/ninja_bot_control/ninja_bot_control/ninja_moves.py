#!/usr/bin/env python3

import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class NinjaMoves(Node):
    def __init__(self):
        super().__init__("ninja_moves")

        self.cmd_pub = self.create_publisher(String, "/ninja/esp32_cmd", 10)
        self.team_color = "blue"  # "blue" eller "yellow"
        self.telemetry_sub = self.create_subscription(
            String,
            "/ninja/telemetry",
            self.telemetry_callback,
            10,
        )

        self.avoidance_enabled = True
        self.ignore_front_obstacles = False
        
        # Standard tuning values
        self.default_speed = 180
        self.turn_speed = 180
        self.sweep_speed = 210
        self.slow_speed = 150

        # Timed movement calibration
        self.turn_90_left_time = 0.80
        self.turn_90_right_time = 0.80

        # Sensor state placeholders
        self.front_vlx_mm = None
        self.left_vlx_mm = None
        self.right_vlx_mm = None

        # Overhead camera pose placeholder
        self.robot_x = None
        self.robot_y = None
        self.robot_theta = None

        time.sleep(0.5)
        self.get_logger().info("Ninja moves ready")
        
    # -------------------------
    # Basic command sending
    # -------------------------

    def send(self, command: str):
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"CMD: {command}")
        time.sleep(0.05)

    def stop(self):
        self.send("stop")

    def motors(self, left: int, right: int):
        self.send(f"motors {left} {right}")

    def drive_for(self, left: int, right: int, seconds: float):
        self.motors(left, right)
        time.sleep(seconds)
        self.stop()
        time.sleep(0.2)

    # -------------------------
    # Team color
    # -------------------------

    def set_team_color(self, color: str):
        color = color.lower().strip()

        if color not in ["blue", "yellow"]:
            self.get_logger().warn("Team color must be 'blue' or 'yellow'")
            return

        self.team_color = color
        self.get_logger().info(f"Team color set to {self.team_color}")

    def is_blue(self):
        return self.team_color == "blue"

    def is_yellow(self):
        return self.team_color == "yellow"
    
    # -------------------------
    # Speed control
    # -------------------------

    def set_speed(self, speed: int):
        self.default_speed = max(0, min(255, speed))
        self.get_logger().info(f"Default speed set to {self.default_speed}")

    def speed_up(self, amount: int = 10):
        self.set_speed(self.default_speed + amount)

    def speed_down(self, amount: int = 10):
        self.set_speed(self.default_speed - amount)

    # -------------------------
    # Basic movement
    # -------------------------

    def forward(self, seconds=1.0, speed=None):
        speed = speed or self.default_speed
        self.drive_for(speed, speed, seconds)

    def backward(self, seconds=1.0, speed=None):
        speed = speed or self.default_speed
        self.drive_for(-speed, -speed, seconds)

    def turn_left(self, seconds=0.5, speed=None):
        speed = speed or self.turn_speed
        self.drive_for(-speed, speed, seconds)

    def turn_right(self, seconds=0.5, speed=None):
        speed = speed or self.turn_speed
        self.drive_for(speed, -speed, seconds)

    def turn_left_90(self):
        self.turn_left(self.turn_90_left_time, self.turn_speed)

    def turn_right_90(self):
        self.turn_right(self.turn_90_right_time, self.turn_speed)

    def forward_corrected(self, seconds=1.0, speed=None, kp=0.25):
        speed = speed or self.default_speed
        start = time.time()

        while rclpy.ok() and time.time() - start < seconds:
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.should_stop_for_obstacle():
                self.get_logger().warn("Obstacle detected, stopping")
                break

            if self.left_vlx_mm is None or self.right_vlx_mm is None:
                self.motors(speed, speed)
                time.sleep(0.05)
                continue

            error = self.left_vlx_mm - self.right_vlx_mm
            correction = int(error * kp)
            correction = max(-40, min(40, correction))

            left = self.clamp_pwm(speed + correction)
            right = self.clamp_pwm(speed - correction)

            self.motors(left, right)
            time.sleep(0.05)

        self.stop()

    # -------------------------
    # Servo / mechanism commands
    # -------------------------

    def grip(self, angle: int):
        self.send(f"grip {angle}")

    def tilt(self, angle: int):
        self.send(f"tilt {angle}")

    def release(self):
        self.send("release")

    def start_position(self):
        self.send("startposition")

    def neutral_position(self):
        self.send("neutralposition")

    def eating(self):
        self.ignore_front_obstacles = True
        self.send("eating")

    def stop_eating(self):
        self.send("stopeating")
        self.ignore_front_obstacles = False

    # -------------------------
    # Useful robot actions
    # -------------------------

    def sweep(self):
        self.get_logger().info("Sweeping pieces")

        self.ignore_front_obstacles = True

        self.forward(seconds=0.8, speed=self.sweep_speed)
        self.backward(seconds=0.25, speed=self.default_speed)

        self.ignore_front_obstacles = False
        self.stop()

    def grab_one_crate(self):
        self.tilt(0)
        time.sleep(0.4)
        self.grip(165)
        time.sleep(0.8)
        self.tilt(65)
        time.sleep(0.4)

    def grab_two_crate(self):
        self.tilt(0)
        time.sleep(0.4)
        self.grip(58)
        time.sleep(0.8)
        self.tilt(65)
        time.sleep(0.4)

    def drop_crate(self):
        self.tilt(9)
        time.sleep(0.4)
        self.release()
        time.sleep(0.4)

    def drive_to_endspot(self):
        """
        Temporary timed routine.
        Later replace with coordinate/camera navigation.
        """
        self.forward(1.2, self.default_speed)
        
        if self.is_blue():
            self.turn_left_90()
        else:
            self.turn_right_90()
            
        self.forward(0.8, self.default_speed)
        self.stop()

    # -------------------------
    # Alignment / sensor logic
    # -------------------------
    def telemetry_callback(self, msg):
        text = msg.data.strip()

        if not text.startswith("TEL VLX"):
            return

        parts = text.split()

        # Format: TEL VLX d1 d2 d3 d4 d5 d6
        if len(parts) < 8:
            return

        try:
            values = [int(x) for x in parts[2:8]]
        except ValueError:
            return

        d1, d2, d3, d4, d5, d6 = values

        front_values = [d for d in [d1, d2, d3, d4] if d > 0]

        self.front_vlx_mm = min(front_values) if front_values else None
        self.left_vlx_mm = d5 if d5 > 0 else None
        self.right_vlx_mm = d6 if d6 > 0 else None

    def clamp_pwm(self, value):
        return max(-255, min(255, int(value)))

    def should_stop_for_obstacle(self, threshold_mm=180):
        if not self.avoidance_enabled:
            return False

        if self.ignore_front_obstacles:
            return False

        if self.front_vlx_mm is None:
            return False

        return self.front_vlx_mm < threshold_mm
    def align_to_piece(self):
        """
        Placeholder for camera-based alignment.
        Later this should use camera topic, e.g.:
        - piece_x in image
        - image center
        - rotate until centered
        """
        self.get_logger().warn("align_to_piece is placeholder until camera topic is connected")

        # Temporary manual-style alignment movement:
        self.turn_left(0.10, self.slow_speed)
        self.stop()

    def detect_wall(self, threshold_mm=120):
        """
        Uses VLX values once telemetry parsing is connected.
        """
        if self.front_vlx_mm is None:
            self.get_logger().warn("No front VLX data yet")
            return False

        return self.front_vlx_mm < threshold_mm

    def detect_opponent(self, threshold_mm=250):
        """
        Basic opponent detection using front VLX.
        Later can combine VLX + camera.
        """
        if self.front_vlx_mm is None:
            self.get_logger().warn("No front VLX data yet")
            return False

        return self.front_vlx_mm < threshold_mm

    def drive_until_wall(self, stop_distance_mm=120, speed=None):
        """
        Placeholder. Needs VLX telemetry parsing before it becomes reliable.
        """
        speed = speed or self.slow_speed

        self.get_logger().info("Driving until wall detected")

        while rclpy.ok():
            if self.detect_wall(stop_distance_mm):
                break

            self.motors(speed, speed)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.stop()

    # -------------------------
    # Overhead camera coordinate driving
    # -------------------------

    def drive_to_coordinate(self, target_x, target_y):
        """
        Placeholder for overhead camera navigation.

        Needs robot pose:
        self.robot_x
        self.robot_y
        self.robot_theta

        Later:
        - rotate toward target
        - drive forward
        - correct heading from camera
        """
        if self.robot_x is None or self.robot_y is None or self.robot_theta is None:
            self.get_logger().warn("No overhead camera pose yet")
            return

        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        distance = math.sqrt(dx * dx + dy * dy)

        self.get_logger().info(
            f"Driving to coordinate x={target_x}, y={target_y}, distance={distance:.2f}"
        )

        # Temporary unsafe placeholder:
        # Do not use for real coordinate driving yet.
        self.forward(seconds=1.0, speed=self.default_speed)
        self.stop()

    # -------------------------
    # Arena routines
    # -------------------------

    def go_to_crates(self):
        self.forward(1.2, self.default_speed)
    
        if self.is_blue():
            self.turn_left_90()
        else:
            self.turn_right_90()
    
        self.forward(0.8, self.slow_speed)
        self.stop()

    def go_to_drop_zone(self):
        self.backward(0.5, self.default_speed)
            
        if self.is_blue():
            self.turn_left_90()
        else:
            self.turn_right_90()
    
        self.forward(1.0, self.default_speed)
        self.stop()

    def go_to_fridge(self):
        self.backward(0.5, self.default_speed)
    
        if self.is_blue():
            self.turn_left_90()
        else:
            self.turn_right_90()
    
        self.forward(1.0, self.default_speed)
        self.stop()

    def full_test_routine(self):
        self.start_position()
        time.sleep(0.5)
        self.go_to_crates()
        self.grab_two_crate()
        self.go_to_drop_zone()
        self.drop_crate()
        self.drive_to_endspot()


def main(args=None):
    rclpy.init(args=args)
    node = NinjaMoves()

    try:
        while rclpy.ok():
            command = input("ninja> ").strip()
            parts = command.split()
            if not parts:
                continue

            if command in ["q", "quit", "exit"]:
                break
                
            elif command.startswith("team "):
                color = command.split()[1]
                node.set_team_color(color)
            
            elif command == "team":
                node.get_logger().info(f"Current team color: {node.team_color}")

            elif command == "stop":
                node.stop()

            elif parts[0] == "forward":
                seconds = float(parts[1]) if len(parts) > 1 else 1.0
                speed = int(parts[2]) if len(parts) > 2 else None
                node.forward(seconds, speed)

            elif parts[0] == "backward":
                seconds = float(parts[1]) if len(parts) > 1 else 1.0
                speed = int(parts[2]) if len(parts) > 2 else None
                node.backward(seconds, speed)

            elif parts[0] == "left":
                seconds = float(parts[1]) if len(parts) > 1 else 0.5
                speed = int(parts[2]) if len(parts) > 2 else None
                node.turn_left(seconds, speed)

            elif parts[0] == "right":
                seconds = float(parts[1]) if len(parts) > 1 else 0.5
                speed = int(parts[2]) if len(parts) > 2 else None
                node.turn_right(seconds, speed)

            elif command == "left90":
                node.turn_left_90()

            elif command == "right90":
                node.turn_right_90()

            elif command == "sweep":
                node.sweep()

            elif command == "grabone":
                node.grab_one_crate()

            elif command == "grabtwo":
                node.grab_two_crate()

            elif command == "drop":
                node.drop_crate()

            elif command == "crates":
                node.go_to_crates()

            elif command == "dropzone":
                node.go_to_drop_zone()

            elif command == "endspot":
                node.drive_to_endspot()

            elif command == "eating":
                node.eating()

            elif command == "stopeating":
                node.stop_eating()

            elif command == "align":
                node.align_to_piece()

            elif command == "wall":
                node.drive_until_wall()

            elif command == "routine":
                node.full_test_routine()

            elif command.startswith("speed "):
                speed = int(command.split()[1])
                node.set_speed(speed)

            elif command == "speedup":
                node.speed_up()

            elif command == "speeddown":
                node.speed_down()

            elif parts[0] == "forwardcorrected":
                seconds = float(parts[1]) if len(parts) > 1 else 1.0
                speed = int(parts[2]) if len(parts) > 2 else None
                node.forward_corrected(seconds, speed)

            else:
                # Raw command passthrough
                node.send(command)

    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
