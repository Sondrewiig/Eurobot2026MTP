from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare('sondre_bot_vision'), 'config', 'overhead_tags.yaml']
    )

    return LaunchDescription([
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['/overhead_cam/image'],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/overhead_cam/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
            ],
            output='screen',
        ),
        Node(
            package='sondre_bot_vision',
            executable='overhead_rectifier_node',
            name='overhead_rectifier_node',
            parameters=[config_file],
            output='screen',
        ),
    ])