import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share_directory = get_package_share_directory('image_file_publisher')
    config_file_path = os.path.join(
        package_share_directory,
        'config',
        'image_file_publisher.yaml',
    )

    return LaunchDescription([
        Node(
            package='image_file_publisher',
            executable='image_file_publisher_node',
            name='image_file_publisher_node',
            output='screen',
            parameters=[config_file_path],
        ),
    ])
