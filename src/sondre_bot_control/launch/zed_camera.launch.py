from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
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
    home = os.path.expanduser("~")
    v4l2_params = os.path.join(home, "Eurobot2026MTP", "config", "v4l2_camera.yaml")
    splitter_calib = os.path.join(home, "Eurobot2026MTP", "config", "zed_left_camera.yaml")
    tags_yaml = os.path.join(
        home,
        "Eurobot2026MTP",
        "src",
        "sondre_bot_control",
        "sondre_bot_control",
        "config",
        "tags.yaml",
    )

    video_device = "/dev/video0"

    v4l2_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="v4l2_camera",
        parameters=[v4l2_params],
        remappings=[
            ("image_raw", "/zed/image_raw"),
            ("camera_info", "/zed/camera_info"),
        ],
        output="screen",
    )

    splitter_node = Node(
        package="sondre_bot_control",
        executable="zed_left_splitter",
        name="zed_left_splitter",
        parameters=[{
            "image_in": "/zed/image_raw",
            "camera_info_in": "/zed/camera_info",
            "image_out": "/camera/left/image_raw",
            "camera_info_out": "/camera/left/camera_info",
            "calibration_yaml": splitter_calib,
        }],
        output="screen",
    )

    aruco_node = Node(
        package="sondre_bot_control",
        executable="aruco_detect",
        name="aruco_detect",
        parameters=[{
            "image_topic": "/camera/left/image_raw",
            "camera_info_topic": "/camera/left/camera_info",
            "tags_yaml": tags_yaml,
        }],
        output="screen",
    )

    # Apply camera controls (brightness, contrast, etc.) from the YAML
    # file after the v4l2 node has opened the device.
    controls = _load_camera_controls(v4l2_params)
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