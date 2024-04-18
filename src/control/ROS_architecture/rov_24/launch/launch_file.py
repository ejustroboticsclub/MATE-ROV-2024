from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory("rov_24"), "config", "pid.yaml")
    return LaunchDescription([
        Node(
            package='rov_24',
            executable='movement_feedback',
            name='kinematic_model',
            output='screen',
            parameters=[config]
        ),
        Node(
            package='rov_24',
            executable='motion_control',
            name='motion_control',
            output='screen',
        )
    ])