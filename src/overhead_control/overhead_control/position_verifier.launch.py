from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="overhead_control",
            executable="position_verifier",
            name="ninja_position_verifier",
            output="screen",
            parameters=[{
                "display_hz": 1.0,
                "clear_screen": True,
            }],
        )
    ])
