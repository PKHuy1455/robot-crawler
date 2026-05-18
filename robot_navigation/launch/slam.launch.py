import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Package paths ───────────────────────────────────────────────────
    robot_navigation_dir = get_package_share_directory('robot_navigation')
    robot_bringup_dir = get_package_share_directory('robot_bringup')

    # ── Default config path ─────────────────────────────────────────────
    default_slam_params_file = os.path.join(
        robot_navigation_dir, 'config', 'mapper_params_online_async.yaml'
    )

    # ── Launch arguments ────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_slam_params_file,
        description='Full path to the slam_toolbox parameter file'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    # ── Include robot bringup (hardware + lidar) ────────────────────────
    robot_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_bringup_dir, 'launch', 'robot_bringup.launch.py')
        ),
    )

    # ── slam_toolbox async mapping node ─────────────────────────────────
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    # ── Build launch description ────────────────────────────────────────
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        slam_params_file_arg,

        # Bringup
        robot_bringup_launch,

        # SLAM
        slam_toolbox_node,
    ])
