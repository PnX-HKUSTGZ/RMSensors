import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share_directory = get_package_share_directory('hk_camera_driver')
    config_file_path = os.path.join(
        package_share_directory,
        'config',
        'hk_camera_driver.yaml',
    )

    return LaunchDescription([
        Node(
            package='hk_camera_driver',
            executable='hk_camera_driver_node',
            name='hk_camera_driver',
            output='screen',
            parameters=[config_file_path],
        ),
    ])
