import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    robot_navigation_dir = get_package_share_directory('robot_navigation')
    nav2_bringup_dir     = get_package_share_directory('nav2_bringup')
    robot_bringup_dir    = get_package_share_directory('robot_bringup')

    slam_params_file = os.path.join(
        robot_navigation_dir, 'config', 'mapper_params_online_async.yaml')
    nav2_params_file = os.path.join(
        robot_navigation_dir, 'config', 'nav2_params_slam.yaml')

    # 0. Reset serial ports
    reset_serial = ExecuteProcess(
        cmd=['bash', '-c',
             'stty -F /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 sane; stty -F /dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_b2ce7c4d786eef11b49ce8c2c169b110-if00-port0 sane'],
        output='screen'
    )

    # 1. Robot bringup
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_bringup_dir, 'launch', 'robot_bringup.launch.py')),
    )

    # 2. slam_toolbox async
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': False}],
    )

    # 3. Nav2 (custom launch with bond_timeout: 0.0)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_navigation_dir, 'launch', 'nav2_custom.launch.py')),
    )

    # 5. Camera (YUYV raw format to publish /image_raw directly)
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

    # 6. Anomaly detector
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

    # 7. Position bridge
    position_bridge = Node(
        package='anomaly_detection',
        executable='position_bridge',
        name='position_bridge',
        output='screen',
    )

    # 8. Coordinator
    coordinator = Node(
        package='anomaly_detection',
        executable='coordinator',
        name='anomaly_coordinator',
        output='screen',
        parameters=[{
            'save_dir': '/home/pi/robot_data/anomalies',
        }]
    )

    # 9. explore_lite — delay 35 giay de hoan thanh bootstrap
    explore = TimerAction(
        period=35.0,
        actions=[
            Node(
                package='explore_lite',
                name='explore_node',
                executable='explore',
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    'robot_base_frame': 'base_link',
                    'costmap_topic': '/global_costmap/costmap',
                    'costmap_updates_topic': '/global_costmap/costmap_updates',
                    'visualize': True,
                    'planner_frequency': 0.05,
                    'progress_timeout': 90.0,
                    'potential_scale': 1.0,
                    'orientation_scale': 0.0,
                    'gain_scale': 5.0,
                    'transform_tolerance': 0.5,
                    'min_frontier_size': 0.15,
                    'return_to_init': False,
                }],
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            )
        ]
    )

    return LaunchDescription([
        reset_serial,
        robot_bringup,
        slam_toolbox,
        nav2_bringup,
        camera,
        anomaly_detector,
        position_bridge,
        coordinator,
        # explore,  # commented out because we are using waypoint navigation
    ])
