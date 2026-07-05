"""
Custom Nav2 Map Bringup launch file for Robot Crawler.
Launches both Localization (map_server, amcl) and Navigation (controller, planner, behaviors, bt_navigator, etc.)
with bond_timeout set to 0.0 to prevent deactivation due to Raspberry Pi CPU latency.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_navigation_dir = get_package_share_directory('robot_navigation')

    map_yaml_file = LaunchConfiguration('map')
    nav2_params_file = LaunchConfiguration('params_file')

    declare_map_yaml = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            os.path.expanduser('~'), 'robot_ws', 'maps', 'workshop_map.yaml'),
        description='Full path to map yaml file')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            robot_navigation_dir, 'config', 'nav2_params.yaml'),
        description='Full path to nav2 params file')

    # Node names for lifecycle manager
    lifecycle_nodes = [
        'map_server',
        'amcl',
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # 1. Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params_file, {'yaml_filename': map_yaml_file}],
        remappings=remappings)

    # 2. AMCL (Localization)
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file],
        remappings=remappings)

    # 3. Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')])

    # 4. Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=remappings)

    # 5. Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=remappings)

    # 6. BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file],
        remappings=remappings)

    # 7. Waypoint Follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_file],
        remappings=remappings)

    # 8. Velocity Smoother
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_file],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')])

    # 9. Lifecycle Manager (managing ALL localization and navigation nodes)
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'bond_timeout': 0.0,  # CRITICAL: Disable heartbeat timeouts to prevent node deactivation
            'node_names': lifecycle_nodes,
        }])

    return LaunchDescription([
        declare_map_yaml,
        declare_params_file,
        map_server,
        amcl,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager,
    ])
