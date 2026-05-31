from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agv_web_monitor',
            executable='web_monitor_node',
            name='web_monitor',
            parameters=[{
                'port': 8080,
                'agv_ids': ['agv_001', 'agv_002'],
            }],
            output='screen'
        )
    ])
