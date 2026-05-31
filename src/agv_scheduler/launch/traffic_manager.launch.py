"""
交通管理器启动文件
启动交通管理器节点，负责多AGV路径预约和冲突检测
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('agv_scheduler')
    params_file = os.path.join(pkg_share, 'config', 'scheduler_params.yaml')

    return LaunchDescription([
        Node(
            package='agv_scheduler',
            executable='traffic_manager_node',
            name='traffic_manager',
            output='screen',
            parameters=[params_file],
        ),
    ])
