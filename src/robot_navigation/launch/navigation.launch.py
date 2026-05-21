import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    robot_navigation_dir = get_package_share_directory('robot_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    declare_map_yaml = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            os.path.expanduser('~'),
            'robot_ws', 'maps', 'workshop_map.yaml'),
        description='Full path to map yaml file')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            robot_navigation_dir, 'config', 'nav2_params.yaml'),
        description='Full path to nav2 params file')

    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_bringup'),
                'launch', 'robot_bringup.launch.py')),
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': 'false',
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': 'False',
            'use_respawn': 'False',
            'slam': 'False',
        }.items(),
    )

    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        arguments=['/cmd_vel_nav', '/cmd_vel'],
    )

    return LaunchDescription([
        declare_map_yaml,
        declare_params_file,
        robot_bringup,
        nav2_bringup,
        cmd_vel_relay,
    ])
