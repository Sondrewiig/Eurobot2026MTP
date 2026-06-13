"""
ninja_crate_align_only.launch.py

Onboard crate/Jenga alignment launch for the Ninja Pi.

Starts only:
  - esp32_bridge
  - camera_node
  - crate_detector_node
  - crate_align_node

It deliberately does NOT start ninja_go_to_point, so the overhead/global
controller cannot fight crate_align_node on /cmd_vel.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration("port")

    camera = LaunchConfiguration("camera")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    fmt = LaunchConfiguration("format")
    image_topic = LaunchConfiguration("image_topic")
    show_window = LaunchConfiguration("show_window")
    publish_debug_image = LaunchConfiguration("publish_debug_image")
    debug_image_rate_hz = LaunchConfiguration("debug_image_rate_hz")
    debug_image_max_width = LaunchConfiguration("debug_image_max_width")
    process_every_n_frames = LaunchConfiguration("process_every_n_frames")

    enable_motion = LaunchConfiguration("enable_motion")
    align_mode = LaunchConfiguration("align_mode")
    target_id = LaunchConfiguration("target_id")
    orientation = LaunchConfiguration("orientation")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),

        DeclareLaunchArgument(
            "camera",
            default_value="/base/axi/pcie@120000/rp1/i2c@88000/imx219@10",
            description="camera_ros/libcamera camera id for Pi IMX219",
        ),
        DeclareLaunchArgument("width", default_value="1640"),
        DeclareLaunchArgument("height", default_value="1232"),
        DeclareLaunchArgument("format", default_value="RGB888"),
        DeclareLaunchArgument("image_topic", default_value="/ninja/camera/image_raw"),
        DeclareLaunchArgument("show_window", default_value="false"),
        DeclareLaunchArgument("publish_debug_image", default_value="false"),
        DeclareLaunchArgument("debug_image_rate_hz", default_value="5.0"),
        DeclareLaunchArgument("debug_image_max_width", default_value="1280"),
        DeclareLaunchArgument("process_every_n_frames", default_value="2"),

        DeclareLaunchArgument("enable_motion", default_value="true"),
        DeclareLaunchArgument("align_mode", default_value="pair"),
        DeclareLaunchArgument("target_id", default_value="36"),
        DeclareLaunchArgument("orientation", default_value="auto"),

        Node(
            package="ninja_bot_control",
            executable="esp32_bridge",
            name="esp32_bridge",
            output="screen",
            parameters=[{
                "port": port,
                "command_rate_hz": 20.0,
                "min_pwm": 150,
                "max_pwm": 220,
                "turn_scale": 0.30,
                "drive_guard_norm": 0.20,
                # Faster ramp is ok here because crate_align sends short pulses.
                # It is local to this launch and does not change overhead driving.
                "max_pwm_step_per_tick": 60,
                "linear_full_scale_mps": 0.20,
                "angular_full_scale_radps": 1.20,
                "prevent_reverse_while_driving": True,
                "send_only_on_change": True,
                "pwm_change_threshold": 2,
                "resend_interval_s": 0.40,
            }],
        ),

        Node(
            package="camera_ros",
            executable="camera_node",
            name="camera",
            namespace="ninja",
            output="screen",
            remappings=[
                ("/camera/image_raw", image_topic),
                ("/camera/camera_info", "/ninja/camera/camera_info"),
                ("image_raw", image_topic),
                ("camera_info", "/ninja/camera/camera_info"),
            ],
            parameters=[{
                "camera": camera,
                "width": ParameterValue(width, value_type=int),
                "height": ParameterValue(height, value_type=int),
                "format": fmt,
            }],
        ),

        Node(
            package="ninja_bot_control",
            executable="crate_detector",
            name="crate_detector_node",
            output="screen",
            parameters=[{
                "image_topic": image_topic,
                "show_window": ParameterValue(show_window, value_type=bool),
                "publish_debug_image": ParameterValue(publish_debug_image, value_type=bool),
                "debug_image_topic": "/ninja/vision/debug_image",
                "debug_image_rate_hz": ParameterValue(debug_image_rate_hz, value_type=float),
                "debug_image_max_width": ParameterValue(debug_image_max_width, value_type=int),
                "process_every_n_frames": ParameterValue(process_every_n_frames, value_type=int),
            }],
        ),

        Node(
            package="ninja_bot_control",
            executable="crate_align",
            name="crate_align_node",
            output="screen",
            parameters=[{
                "detection_topic": "/ninja/vision/crate",
                "mode": align_mode,
                "target_id": ParameterValue(target_id, value_type=int),
                "orientation": orientation,
                "enable_motion": ParameterValue(enable_motion, value_type=bool),
                "cmd_vel_topic": "/cmd_vel",

                # Pulse-control defaults tuned for delayed onboard vision.
                "pulse_control": True,
                "turn_pulse_s": 0.18,
                "drive_pulse_s": 0.25,
                "pulse_wait_s": 0.45,
                "min_turn_cmd_radps": 0.15,
                "min_approach_speed": 0.025,
                "center_gain": 0.0010,
                "max_turn": 0.16,
                "approach_speed": 0.030,
                "pair_center_tolerance_px": 35.0,
                "pair_size_tolerance_px": 8.0,
                "separation_tolerance_px": 30.0,
                "stale_timeout_s": 1.2,
                "turn_sign": 1.0,
            }],
        ),
    ])
