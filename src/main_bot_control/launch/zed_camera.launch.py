from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def _load_camera_controls(yaml_path: str) -> dict:
    """Read the non-ROS 'camera_controls' block from the camera YAML."""
    try:
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f) or {}
        return data.get("camera_controls", {}) or {}
    except Exception as e:
        print(f"[launch] could not read camera_controls from {yaml_path}: {e}")
        return {}


def generate_launch_description():
    workspace = os.environ.get("EUROBOT_WS", os.getcwd())
    pkg_share = get_package_share_directory("main_bot_control")

    v4l2_params = os.path.join(pkg_share, "config", "v4l2_camera.yaml")
    camera_controls = os.path.join(pkg_share, "config", "camera_controls.yaml")
    tags_yaml = os.path.join(pkg_share, "config", "tags.yaml")

    left_calib = os.path.join(workspace, "config", "zed_left_camera.yaml")
    right_calib = os.path.join(workspace, "config", "zed_right_camera.yaml")

    video_device = os.environ.get("VIDEO_DEVICE", "/dev/video0")

    v4l2_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="v4l2_camera",
        parameters=[v4l2_params, {"video_device": video_device}],
        remappings=[
            ("image_raw", "/zed/image_raw"),
            ("camera_info", "/zed/camera_info"),
        ],
        output="screen",
    )

    splitter_node = Node(
        package="main_bot_control",
        executable="zed_left_splitter",
        name="zed_left_splitter",
        parameters=[{
            "image_in": "/zed/image_raw",
            "camera_info_in": "/zed/camera_info",
            "left_image_out": "/camera/left/image_raw",
            "right_image_out": "/camera/right/image_raw",
            "left_camera_info_out": "/camera/left/camera_info",
            "right_camera_info_out": "/camera/right/camera_info",
            "left_calibration_yaml": left_calib,
            "right_calibration_yaml": right_calib,
        }],
        output="screen",
    )

    aruco_node = Node(
        package="main_bot_control",
        executable="aruco_detect",
        name="aruco_detect",
        parameters=[{
            "image_topic": "/camera/left/image_raw",
            "camera_info_topic": "/camera/left/camera_info",
            "tags_yaml": tags_yaml,
        }],
        output="screen",
    )

    controls = _load_camera_controls(camera_controls)
    control_actions = [
        ExecuteProcess(
            cmd=["v4l2-ctl", "-d", video_device, "-c", f"{name}={value}"],
            output="screen",
        )
        for name, value in controls.items()
    ]
    apply_controls = TimerAction(period=2.0, actions=control_actions)

    return LaunchDescription([
        v4l2_node,
        splitter_node,
        aruco_node,
        apply_controls,
    ])
