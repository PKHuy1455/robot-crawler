"""
Custom Nav2 launch file for Robot Crawler.
Key differences from the standard nav2_bringup/navigation_launch.py:
  - bond_timeout: 0.0 (disables bond monitoring to prevent cascade shutdown on Pi)
  - Removes smoother_server (not needed for slow crawler, saves CPU)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nav2_params_file = os.path.join(
        get_package_share_directory('robot_navigation'),
        'config', 'nav2_params_slam.yaml')

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    return LaunchDescription([
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params_file],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params_file],
            remappings=remappings),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params_file],
            remappings=remappings),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params_file],
            remappings=remappings),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_params_file],
            remappings=remappings),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[nav2_params_file],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'bond_timeout': 0.0,
                'node_names': lifecycle_nodes,
            }]),
    ])
