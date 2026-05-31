"""
任务调度器启动文件
启动任务调度器节点，负责AGV任务分配
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
            executable='task_scheduler_node',
            name='task_scheduler',
            output='screen',
            parameters=[params_file],
        ),
    ])
