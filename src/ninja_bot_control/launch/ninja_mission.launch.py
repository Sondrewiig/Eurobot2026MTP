"""
ninja_mission.launch.py

Launches the full Ninja drive stack + mission orchestrator.

Nodes started:
  esp32_bridge    — serial bridge to ESP32 motor/gripper controller
  go_to_point     — closed-loop navigation from /ninja/pose to /ninja/goal_pose
  ninja_mission   — mission orchestrator (fridge1 / fridge2 / full / test_*)

Usage:
  ros2 launch ninja_bot_control ninja_mission.launch.py
  ros2 launch ninja_bot_control ninja_mission.launch.py port:=/dev/ttyUSB1

Send a mission:
  ros2 topic pub --once /ninja/mission std_msgs/String "{data: test_nav_fridge1}"
  ros2 topic pub --once /ninja/mission std_msgs/String "{data: fridge1}"
  ros2 topic pub --once /ninja/mission std_msgs/String "{data: abort}"

Monitor status:
  ros2 topic echo /ninja/mission_status
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration("port")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),

        Node(
            package="ninja_bot_control",
            executable="esp32_bridge",
            name="esp32_bridge",
            output="screen",
            parameters=[{
                "port": port,
                "command_rate_hz": 30.0,
                "max_pwm": 220,
                "min_pwm": 150,
                "turn_scale": 0.30,
                "drive_guard_norm": 0.20,
                "max_pwm_step_per_tick": 20,
                "linear_full_scale_mps": 0.20,
                "angular_full_scale_radps": 1.20,
                "prevent_reverse_while_driving": True,
                "send_only_on_change": False,
                "pwm_change_threshold": 2,
                "resend_interval_s": 0.05,
                "cmd_vel_timeout_sec": 1.0,
            }],
        ),

        Node(
            package="ninja_bot_control",
            executable="go_to_point",
            name="ninja_go_to_point",
            output="screen",
            parameters=[{
                "pose_topic":      "/ninja/pose",
                "goal_topic":      "/ninja/goal_pose",
                "cmd_vel_topic":   "/cmd_vel",
                "enable_topic":    "/ninja/enable_drive",
                # Operator must type `enable` before driving.
                # Values tuned to overcome drivetrain static friction.
                "start_enabled":   False,
                "max_linear_mps":          0.08,
                "max_angular_radps":       0.25,
                "min_linear_mps":          0.025,
                "min_angular_radps":       0.08,
                "k_linear":                1.2,
                "k_angular":               1.0,
                "turn_in_place_threshold_deg": 90.0,
                "xy_tolerance_mm":         30.0,
                "heading_tolerance_deg":   10.0,
                "use_goal_heading":        True,
                "slowdown_radius_mm":      150.0,
                "pose_timeout_s":          1.0,
            }],
        ),

        Node(
            package="ninja_bot_control",
            executable="ninja_mission",
            name="ninja_mission_node",
            output="screen",
            parameters=[{
                "nest_pose_x":            2300.0,
                "nest_pose_y":            1940.0,
                "nest_pose_theta_deg":    180.0,
                "fridge1_pre_x":          1900.0,
                "fridge1_pre_y":          1925.0,
                "fridge1_pre_theta_deg":  270.0,
                "fridge2_pre_x":          1650.0,
                "fridge2_pre_y":          1925.0,
                "fridge2_pre_theta_deg":  270.0,
                "fridge_approach_y":      1940.0,
                "fridge_approach_theta_deg": 180.0,
                "fridge2_push_mm":        60.0,
                "nav_timeout_s":          30.0,
            }],
        ),
    ])
