import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share_directory = get_package_share_directory('camera_calibration')
    config_file_path = os.path.join(
        package_share_directory,
        'config',
        'camera_calibration.yaml',
    )

    return LaunchDescription([
        Node(
            package='camera_calibration',
            executable='camera_calibration_node',
            name='camera_calibration',
            output='screen',
            parameters=[config_file_path],
        ),
    ])
