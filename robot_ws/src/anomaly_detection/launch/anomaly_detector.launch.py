import os
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
                {'model_path':           '/home/pi/robot_ws/models/best.pt'},
                {'confidence_threshold': 0.50},
                {'imgsz':                320},
                {'cooldown_sec':         5.0},
                {'save_dir':             '/home/pi/robot_data/anomalies'},
            ],
        ),
    ])
