
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # ── Package paths ───────────────────────────────────────────────────
    robot_bringup_dir = get_package_share_directory('robot_bringup')
    robot_hardware_dir = get_package_share_directory('robot_hardware')

    # ── File paths ──────────────────────────────────────────────────────
    urdf_file = os.path.join(robot_bringup_dir, 'urdf', 'robot.urdf')
    hardware_params_file = os.path.join(robot_hardware_dir, 'config', 'hardware_params.yaml')

    # ── Load URDF ───────────────────────────────────────────────────────
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    # ── Launch arguments ────────────────────────────────────────────────
    use_hardware_arg = DeclareLaunchArgument(
        'use_hardware',
        default_value='true',
        description='Enable hardware_driver node'
    )

    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Enable RPLiDAR node'
    )

    use_hardware = LaunchConfiguration('use_hardware')
    use_lidar = LaunchConfiguration('use_lidar')

    # ── Log messages ────────────────────────────────────────────────────
    log_rsp = LogInfo(msg='[robot_bringup] Starting robot_state_publisher ...')

    log_hardware = LogInfo(
        condition=IfCondition(use_hardware),
        msg='[robot_bringup] Starting hardware_driver ...'
    )

    log_lidar = LogInfo(
        condition=IfCondition(use_lidar),
        msg='[robot_bringup] Starting rplidar_composition ...'
    )

    # ── Nodes ───────────────────────────────────────────────────────────
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
        }],
    )

    hardware_driver_node = Node(
        condition=IfCondition(use_hardware),
        package='robot_hardware',
        executable='hardware_driver',
        name='hardware_driver',
        output='screen',
        parameters=[hardware_params_file],
    )

    rplidar_node = Node(
        condition=IfCondition(use_lidar),
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_b2ce7c4d786eef11b49ce8c2c169b110-if00-port0',
            'serial_baudrate': 460800,
            'frame_id': 'laser',
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
    )

    # ── Build launch description ────────────────────────────────────────
    return LaunchDescription([
        # Arguments
        use_hardware_arg,
        use_lidar_arg,

        # Logs
        log_rsp,
        log_hardware,
        log_lidar,

        # Nodes
        robot_state_publisher_node,
        hardware_driver_node,
        rplidar_node,
    ])
