import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    robot_navigation_dir = get_package_share_directory('robot_navigation')
    robot_bringup_dir    = get_package_share_directory('robot_bringup')
    nav2_bringup_dir     = get_package_share_directory('nav2_bringup')

    slam_params_file = os.path.join(
        robot_navigation_dir, 'config', 'mapper_params_online_async.yaml')
    nav2_params_file = os.path.join(
        robot_navigation_dir, 'config', 'nav2_params_slam.yaml')

    # 1. Robot bringup (hardware + LiDAR)
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

    # 3. Nav2 bringup (không có AMCL, không có map_server)
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

    return LaunchDescription([
        robot_bringup,
        slam_toolbox,
        nav2_bringup,
        cmd_vel_relay,
    ])
