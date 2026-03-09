from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # --- paths from installed package ---
    sim_share = get_package_share_directory("sondre_bot_sim")
    world_path = os.path.join(sim_share, "worlds", "building_robot.sdf")

    # --- Gazebo ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            # -r starts running immediately
            "gz_args": f"-r {world_path}"
        }.items(),
    )

    # --- Bridge (Gazebo -> ROS) ---
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/camera/left/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/right/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/left/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/camera/right/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/world/Eurobot_world/dynamic_pose/info@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
        ],
        output="screen",
    )

    # --- Stereo processing (THIS is the key fix) ---
    # This makes stereo_image_proc subscribe to /camera/left/* and /camera/right/*
    stereo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("stereo_image_proc"),
                "launch",
                "stereo_image_proc.launch.py",
            )
        ),
        launch_arguments={
            "namespace": "camera"
        }.items(),
    )
    
    vision = Node(
    package="sondre_bot_control",
    executable="vision_drive",
    output="screen",
    parameters=[{"use_sim_time": True}],
)
    
    apriltag = Node(
    package="apriltag_ros",
    executable="apriltag_node",
    name="apriltag",
    output="screen",
    parameters=[
        PathJoinSubstitution([
            FindPackageShare("sondre_bot_sim"),
            "config",
            "apriltag.yaml",
        ])
    ],
    remappings=[
        ("image_rect", "/camera/left/image_raw"),
        ("camera_info", "/camera/left/camera_info"),
    ],
)
    tag_localization = Node(
    package="sondre_bot_control",
    executable="tag_localization",
    output="screen",
    parameters=[{"use_sim_time": True}],
    )

    ground_truth = Node(
        package="sondre_bot_control",
        executable="ground_truth_pose",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        gazebo,
        bridge,
        stereo,
        vision,
        apriltag,
        tag_localization,
        ground_truth,
    ])