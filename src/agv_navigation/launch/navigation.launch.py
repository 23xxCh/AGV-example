# ============================================================
# navigation.launch.py - AGV导航系统总启动文件
# ============================================================
#
# 【功能】一键启动整个AGV路径规划系统
#   1. 地图服务器 → 加载并发布地图
#   2. A*全局路径规划器 → 订阅地图，提供路径规划服务
#
# 【使用方法】
#   # 启动完整导航系统（使用默认测试地图）
#   ros2 launch agv_navigation navigation.launch.py
#
#   # 使用自定义地图
#   ros2 launch agv_navigation navigation.launch.py yaml_path:=/path/to/map.yaml
#
# 【启动后测试】
#   # 查看地图（RViz2中添加Map显示，话题选/map）
#   ros2 run rviz2 rviz2
#
#   # 请求路径规划（命令行方式）
#   ros2 service call /plan_path agv_interfaces/srv/PathPlan \
#     '{start: {x: 1.0, y: 1.0, z: 0.0}, goal: {x: 3.0, y: 1.0, z: 0.0},
#       use_current_pose: false, planner_id: "astar"}'
#
#   # 查看规划的路径（RViz2中添加Path显示，话题选/planned_path）

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成导航系统启动描述"""

    # 获取各包的共享目录
    map_dir = get_package_share_directory('agv_map')
    planner_dir = get_package_share_directory('agv_global_planner')

    # ----------------------------------------------------------
    # 声明Launch参数
    # ----------------------------------------------------------
    yaml_arg = DeclareLaunchArgument(
        'yaml_path',
        default_value=os.path.join(map_dir, 'maps', 'warehouse_map.yaml'),
        description='地图YAML配置文件路径'
    )

    # ----------------------------------------------------------
    # 节点1：地图服务器
    # ----------------------------------------------------------
    # 负责加载地图文件并发布OccupancyGrid
    map_server_node = Node(
        package='agv_map',
        executable='map_server_node',
        name='map_server',
        parameters=[{
            'yaml_path': LaunchConfiguration('yaml_path'),
        }],
        output='screen'
    )

    # ----------------------------------------------------------
    # 节点2：A*全局路径规划器
    # ----------------------------------------------------------
    # 订阅地图话题，提供路径规划服务
    # 参数从YAML配置文件加载
    global_planner_node = Node(
        package='agv_global_planner',
        executable='global_planner_node',
        name='astar_planner',
        parameters=[
            # 先加载默认参数文件
            os.path.join(planner_dir, 'config', 'global_planner_params.yaml'),
            # Launch参数可以覆盖YAML中的值
        ],
        output='screen'
    )

    # ----------------------------------------------------------
    # 启动提示
    # ----------------------------------------------------------
    startup_log = LogInfo(msg=[
        '\n============================================================\n',
        '  AGV路径规划系统已启动！\n',
        '  ┌──────────────┐    /map     ┌───────────────────┐\n',
        '  │  地图服务器   │ ─────────→ │  A*全局路径规划器  │\n',
        '  └──────────────┘             └───────────────────┘\n',
        '                                       │\n',
        '                              /planned_path ↓\n',
        '                                       │\n',
        '                                ┌───────────┐\n',
        '                                │   RViz2   │\n',
        '                                └───────────┘\n',
        '\n',
        '  测试命令:\n',
        '  ros2 service call /plan_path agv_interfaces/srv/PathPlan \\\n',
        '    \'{start: {x: 0.5, y: 0.5, z: 0.0}, goal: {x: 3.0, y: 1.0, z: 0.0}, ',
        'use_current_pose: false, planner_id: "astar"}\'\n',
        '============================================================\n',
    ])

    return LaunchDescription([
        yaml_arg,
        startup_log,
        map_server_node,
        global_planner_node,
    ])
