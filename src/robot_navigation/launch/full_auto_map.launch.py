"""
full_auto_map.launch.py — One-click launch for Static Map + AMCL + Nav2 + YOLO + Waypoint
==========================================================================================
Mode: Localization (AMCL) with pre-built map (no SLAM)
Flow: Load map → AMCL localization → Nav2 navigation → Coordinator sends waypoints
      → YOLO anomaly detection → Return home → Generate report
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    robot_navigation_dir = get_package_share_directory('robot_navigation')
    nav2_bringup_dir     = get_package_share_directory('nav2_bringup')
    robot_bringup_dir    = get_package_share_directory('robot_bringup')

    nav2_params_file = os.path.join(
        robot_navigation_dir, 'config', 'nav2_params.yaml')
    map_yaml_file = os.path.join(
        os.path.expanduser('~'), 'robot_ws', 'maps', 'workshop_map.yaml')

    # 0. Reset serial ports
    reset_serial = ExecuteProcess(
        cmd=['bash', '-c',
             'stty -F /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 sane; '
             'stty -F /dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_b2ce7c4d786eef11b49ce8c2c169b110-if00-port0 sane'],
        output='screen'
    )

    # 1. Robot bringup (robot_state_publisher, hardware_driver, rplidar)
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_bringup_dir, 'launch', 'robot_bringup.launch.py')),
    )

    # 2. Nav2 bringup with static map + AMCL (Custom with bond_timeout: 0.0)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_navigation_dir, 'launch', 'nav2_map_custom.launch.py')),
        launch_arguments={
            'map': map_yaml_file,
            'params_file': nav2_params_file,
        }.items(),
    )

    # 4. Camera (YUYV raw format → /image_raw)
    camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [320, 240],
            'pixel_format': 'YUYV',
            'time_per_frame': [1, 2],
        }]
    )

    # 5. Anomaly detector (YOLOv8n)
    anomaly_detector = Node(
        package='anomaly_detection',
        executable='anomaly_detector',
        name='anomaly_detector',
        output='screen',
        parameters=[{
            'model_path':           '/home/pi/robot_ws/models/best.pt',
            'confidence_threshold': 0.28,
            'imgsz':                320,
            'cooldown_sec':         8.0,
            'save_dir':             '/home/pi/robot_data/anomalies',
        }]
    )

    # 6. Position bridge (TF → /robot_position)
    position_bridge = Node(
        package='anomaly_detection',
        executable='position_bridge',
        name='position_bridge',
        output='screen',
    )

    # 7. Coordinator (Waypoint Navigation State Machine)
    coordinator = Node(
        package='anomaly_detection',
        executable='coordinator',
        name='anomaly_coordinator',
        output='screen',
        parameters=[{
            'save_dir': '/home/pi/robot_data/anomalies',
        }]
    )

    return LaunchDescription([
        reset_serial,
        robot_bringup,
        nav2_bringup,
        camera,
        anomaly_detector,
        position_bridge,
        coordinator,
    ])
