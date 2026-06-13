"""
ninja_full_gui_camera.launch.py

Single Pi-side launch for the operator GUI workflow.

Runs the tuned drive stack from ninja_coordinate_drive_tuned.launch.py plus
camera_ros onboard vision and crate alignment. The operator GUI remains a
separate process, normally on the laptop:

  ros2 run overhead_control operator_gui

Motion safety:
  - go_to_point start_enabled defaults false.
  - crate_align enable_motion defaults false.
  - crate_align also requires /ninja/align/enable true before publishing /cmd_vel.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Drive launch args, forwarded to ninja_coordinate_drive_tuned.launch.py.
    port = LaunchConfiguration("port")
    start_enabled = LaunchConfiguration("start_enabled")

    # Bridge args for this full camera/crate-alignment launch.
    # These are deliberately gentler than overhead/global drive settings,
    # because crate centering uses small in-place corrections.
    bridge_min_pwm = LaunchConfiguration("bridge_min_pwm")
    bridge_max_pwm = LaunchConfiguration("bridge_max_pwm")
    bridge_max_pwm_step_per_tick = LaunchConfiguration("bridge_max_pwm_step_per_tick")
    bridge_turn_scale = LaunchConfiguration("bridge_turn_scale")
    bridge_angular_full_scale_radps = LaunchConfiguration("bridge_angular_full_scale_radps")

    # Onboard camera/vision args.
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

    drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ninja_bot_control"),
                "launch",
                "ninja_coordinate_drive_tuned.launch.py",
            ])
        ),
        launch_arguments={
            "port": port,
            "start_enabled": start_enabled,
            "bridge_min_pwm": bridge_min_pwm,
            "bridge_max_pwm": bridge_max_pwm,
            "bridge_max_pwm_step_per_tick": bridge_max_pwm_step_per_tick,
            "bridge_turn_scale": bridge_turn_scale,
            "bridge_angular_full_scale_radps": bridge_angular_full_scale_radps,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("start_enabled", default_value="false"),

        # Keep bridge defaults at the stable values used by normal driving.
        # Crate alignment is slowed in crate_align_node.py instead, so this
        # launch does not break overhead/go_to_point tuning.
        DeclareLaunchArgument("bridge_min_pwm", default_value="150"),
        DeclareLaunchArgument("bridge_max_pwm", default_value="220"),
        DeclareLaunchArgument("bridge_max_pwm_step_per_tick", default_value="12"),
        DeclareLaunchArgument("bridge_turn_scale", default_value="0.30"),
        DeclareLaunchArgument("bridge_angular_full_scale_radps", default_value="1.20"),

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
        DeclareLaunchArgument("publish_debug_image", default_value="true"),
        DeclareLaunchArgument("debug_image_rate_hz", default_value="5.0"),
        DeclareLaunchArgument("debug_image_max_width", default_value="1280"),
        DeclareLaunchArgument("process_every_n_frames", default_value="2"),
        DeclareLaunchArgument("enable_motion", default_value="false"),
        DeclareLaunchArgument("align_mode", default_value="pair"),
        DeclareLaunchArgument("target_id", default_value="36"),
        DeclareLaunchArgument("orientation", default_value="auto"),

        drive_launch,

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
                "publish_debug_image": ParameterValue(
                    publish_debug_image, value_type=bool
                ),
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
            }],
        ),
    ])
