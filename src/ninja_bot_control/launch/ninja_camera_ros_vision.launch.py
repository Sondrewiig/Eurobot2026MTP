from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
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

    return LaunchDescription([
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
    ])
