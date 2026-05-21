import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='anomaly_detection',
            executable='anomaly_detector',
            name='anomaly_detector',
            output='screen',
            parameters=[
                {'canny_threshold1': 50},
                {'canny_threshold2': 150},
                {'min_crack_area': 100},
                {'blur_kernel': 5},
                {'cooldown_sec': 3.0}
            ]
        ),
        Node(
            package='anomaly_detection',
            executable='position_bridge',
            name='position_bridge',
            output='screen',
        )
    ])
