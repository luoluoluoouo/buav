from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('buav'), 'config', 'local_costmap.yaml'
    ])
    return LaunchDescription([
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',   # ← 用這個
            name='local_costmap',
            output='screen',
            parameters=[params, {'use_sim_time': True}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_local',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['/costmap/costmap'],
                'bond_timeout': 0.0
            }],
        ),
    ])
