#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("tilt_up_on_start", default_value="true"),
        DeclareLaunchArgument("ready_required_count", default_value="2"),
        DeclareLaunchArgument("align_enable_keepalive_hz", default_value="5.0"),

        # ESP32 command strings. These are passed directly to /ninja/esp32_cmd by
        # pickup_sequence_node.py, so commands with spaces such as "tilt 40" work.
        DeclareLaunchArgument("tilt_up_cmd", default_value="tiltup"),
        DeclareLaunchArgument("tilt_down_cmd", default_value="tiltdown"),
        DeclareLaunchArgument("half_tilt_cmd", default_value="tilt 40"),
        DeclareLaunchArgument("grip_two_cmd", default_value="twocrates"),
        DeclareLaunchArgument("release_cmd", default_value="release"),
        DeclareLaunchArgument("stop_cmd", default_value="stop"),

        # Timing parameters.
        DeclareLaunchArgument("tilt_down_wait_s", default_value="1.2"),
        DeclareLaunchArgument("grip_wait_s", default_value="1.0"),
        DeclareLaunchArgument("tilt_up_wait_s", default_value="1.0"),
        DeclareLaunchArgument("half_tilt_wait_s", default_value="0.8"),
        DeclareLaunchArgument("release_wait_s", default_value="0.5"),

        DeclareLaunchArgument("release_before_align", default_value="false"),
        DeclareLaunchArgument("tilt_up_after_drop", default_value="false"),

        Node(
            package="ninja_bot_control",
            executable="pickup_sequence",
            name="ninja_pickup_sequence",
            output="screen",
            parameters=[{
                "tilt_up_on_start": LaunchConfiguration("tilt_up_on_start"),
                "ready_required_count": LaunchConfiguration("ready_required_count"),
                "align_enable_keepalive_hz": LaunchConfiguration("align_enable_keepalive_hz"),

                "tilt_up_cmd": LaunchConfiguration("tilt_up_cmd"),
                "tilt_down_cmd": LaunchConfiguration("tilt_down_cmd"),
                "half_tilt_cmd": LaunchConfiguration("half_tilt_cmd"),
                "grip_two_cmd": LaunchConfiguration("grip_two_cmd"),
                "release_cmd": LaunchConfiguration("release_cmd"),
                "stop_cmd": LaunchConfiguration("stop_cmd"),

                "tilt_down_wait_s": LaunchConfiguration("tilt_down_wait_s"),
                "grip_wait_s": LaunchConfiguration("grip_wait_s"),
                "tilt_up_wait_s": LaunchConfiguration("tilt_up_wait_s"),
                "half_tilt_wait_s": LaunchConfiguration("half_tilt_wait_s"),
                "release_wait_s": LaunchConfiguration("release_wait_s"),

                "release_before_align": LaunchConfiguration("release_before_align"),
                "tilt_up_after_drop": LaunchConfiguration("tilt_up_after_drop"),
            }],
        ),
    ])
