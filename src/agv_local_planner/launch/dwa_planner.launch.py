# ============================================================
# dwa_planner.launch.py - DWA局部规划器启动文件
# ============================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    planner_dir = get_package_share_directory('agv_local_planner')

    dwa_node = Node(
        package='agv_local_planner',
        executable='dwa_planner_node',
        name='dwa_planner',
        parameters=[
            os.path.join(planner_dir, 'config', 'local_planner_params.yaml'),
        ],
        output='screen'
    )

    return LaunchDescription([dwa_node])
