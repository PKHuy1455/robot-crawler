"""Launch file for the hardware_driver node."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_hardware')
    params_file = os.path.join(pkg_dir, 'config', 'hardware_params.yaml')

    hardware_driver_node = Node(
        package='robot_hardware',
        executable='hardware_driver',
        name='hardware_driver',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/odom', '/odom'),
            ('/imu/data_raw', '/imu/data_raw'),
        ],
    )

    return LaunchDescription([
        hardware_driver_node,
    ])
