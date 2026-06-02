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
             'stty -F /dev/ttyUSB0 sane; stty -F /dev/ttyUSB1 sane'],
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

    # 3. Nav2
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_file,
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
    )

    # 4. cmd_vel relay
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        arguments=['/cmd_vel_nav', '/cmd_vel'],
    )

    # 5. Camera
    camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [640, 480],
            'pixel_format': 'YUYV',
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
            'confidence_threshold': 0.75,
            'imgsz':                320,
            'cooldown_sec':         5.0,
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

    # 9. explore_lite — delay 15 giay cho SLAM + Nav2 san sang
    explore = TimerAction(
        period=15.0,
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
                    'planner_frequency': 0.2,
                    'progress_timeout': 120.0,
                    'potential_scale': 3.0,
                    'orientation_scale': 0.0,
                    'gain_scale': 1.0,
                    'transform_tolerance': 0.5,
                    'min_frontier_size': 0.2,
                    'return_to_init': True,
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
        cmd_vel_relay,
        camera,
        anomaly_detector,
        position_bridge,
        coordinator,
        explore,
    ])
