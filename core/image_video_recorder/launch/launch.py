import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share_directory = get_package_share_directory('image_video_recorder')
    config_file_path = os.path.join(
        package_share_directory,
        'config',
        'image_video_recorder.yaml',
    )

    shutdown_sigterm_timeout = LaunchConfiguration('shutdown_sigterm_timeout')
    shutdown_sigkill_timeout = LaunchConfiguration('shutdown_sigkill_timeout')

    return LaunchDescription([
        DeclareLaunchArgument(
            'shutdown_sigterm_timeout',
            default_value='120',
            description='Seconds to wait after SIGINT before escalating to SIGTERM.',
        ),
        DeclareLaunchArgument(
            'shutdown_sigkill_timeout',
            default_value='30',
            description='Seconds to wait after SIGTERM before escalating to SIGKILL.',
        ),
        Node(
            package='image_video_recorder',
            executable='image_video_recorder_node',
            name='image_video_recorder_node',
            output='screen',
            parameters=[config_file_path],
            sigterm_timeout=shutdown_sigterm_timeout,
            sigkill_timeout=shutdown_sigkill_timeout,
        ),
    ])
