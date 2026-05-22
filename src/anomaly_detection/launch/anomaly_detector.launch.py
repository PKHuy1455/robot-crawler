import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Camera V2 node
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            output='screen',
            parameters=[
                {'video_device': '/dev/video0'},
                {'image_size': [640, 480]},
                {'pixel_format': 'YUYV'},
            ]
        ),
        # Anomaly detector node
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
        # Position bridge node
        Node(
            package='anomaly_detection',
            executable='position_bridge',
            name='position_bridge',
            output='screen',
        )
    ])
