# ============================================================
# navigation.launch.py - AGV导航系统总启动文件
# ============================================================
#
# 【功能】一键启动完整AGV导航系统
#   1. 地图服务器 → 加载并发布地图
#   2. A*全局路径规划器 → 订阅地图，提供路径规划服务
#   3. DWA局部避障器 → 订阅地图+路径，发布速度指令
#   4. 导航协调器 → 串联全局规划+局部避障，提供Navigate Action
#
# 【使用方法】
#   ros2 launch agv_navigation navigation.launch.py
#
# 【系统架构】
#   ┌──────────────┐    /map     ┌───────────────────┐
#   │  地图服务器   │ ─────────→ │  A*全局路径规划器  │
#   └──────────────┘             └───────────────────┘
#                                         │
#                                /planned_path ↓
#                                         │
#   ┌──────────────┐   navigate   ┌───────────────────┐
#   │   客户端     │ ──action──→  │    导航协调器      │
#   └──────────────┘              └───────────────────┘
#                                         │
#                                /planned_path ↓
#                                         │
#                                 ┌───────────────────┐
#                                 │  DWA局部避障器     │
#                                 └───────────────────┘
#                                         │
#                                  /cmd_vel ↓
#                                         │
#                                    ┌─────────┐
#                                    │  AGV    │
#                                    └─────────┘

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
    local_planner_dir = get_package_share_directory('agv_local_planner')

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
    global_planner_node = Node(
        package='agv_global_planner',
        executable='global_planner_node',
        name='astar_planner',
        parameters=[
            os.path.join(planner_dir, 'config', 'global_planner_params.yaml'),
        ],
        output='screen'
    )

    # ----------------------------------------------------------
    # 节点3：DWA局部避障器
    # ----------------------------------------------------------
    dwa_planner_node = Node(
        package='agv_local_planner',
        executable='dwa_planner_node',
        name='dwa_planner',
        parameters=[
            os.path.join(local_planner_dir, 'config', 'local_planner_params.yaml'),
        ],
        output='screen'
    )

    # ----------------------------------------------------------
    # 节点4：导航协调器
    # ----------------------------------------------------------
    navigation_coordinator_node = Node(
        package='agv_navigation',
        executable='navigation_coordinator_node',
        name='navigation_coordinator',
        parameters=[{
            'planner_service': '/plan_path',
            'feedback_rate': 5.0,
            'goal_tolerance_xy': 0.3,
            'goal_tolerance_yaw': 0.2,
            'base_frame': 'base_link',
            'map_frame': 'map',
        }],
        output='screen'
    )

    # ----------------------------------------------------------
    # 启动提示
    # ----------------------------------------------------------
    startup_log = LogInfo(msg=[
        '\n============================================================\n',
        '  AGV导航系统已启动（含局部避障）！\n',
        '  ┌──────────────┐    /map     ┌───────────────────┐\n',
        '  │  地图服务器   │ ─────────→ │  A*全局路径规划器  │\n',
        '  └──────────────┘             └───────────────────┘\n',
        '                                         │\n',
        '                                /planned_path ↓\n',
        '                                         │\n',
        '  ┌──────────────┐   navigate   ┌───────────────────┐\n',
        '  │   客户端     │ ──action──→  │    导航协调器      │\n',
        '  └──────────────┘              └───────────────────┘\n',
        '                                         │\n',
        '                                /planned_path ↓\n',
        '                                         │\n',
        '                                 ┌───────────────────┐\n',
        '                                 │  DWA局部避障器     │\n',
        '                                 └───────────────────┘\n',
        '                                         │\n',
        '                                  /cmd_vel ↓\n',
        '                                         │\n',
        '                                    ┌─────────┐\n',
        '                                    │  AGV    │\n',
        '                                    └─────────┘\n',
        '============================================================\n',
    ])

    return LaunchDescription([
        yaml_arg,
        startup_log,
        map_server_node,
        global_planner_node,
        dwa_planner_node,
        navigation_coordinator_node,
    ])
