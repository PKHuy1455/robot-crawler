from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ]
        )
    ])
