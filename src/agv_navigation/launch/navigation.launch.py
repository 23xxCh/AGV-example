# ============================================================
# navigation.launch.py - AGV导航系统总启动文件（含仿真）
# ============================================================
#
# 【功能】一键启动完整AGV导航系统（含2D仿真）
#   1. 地图服务器 → 加载并发布地图
#   2. A*全局路径规划器 → 订阅地图，提供路径规划服务
#   3. DWA局部避障器 → 订阅地图+路径，发布速度指令
#   4. 2D仿真器 → 订阅cmd_vel，发布TF和机器人Marker
#   5. 导航协调器 → 串联全局规划+局部避障
#   6. RViz2 → 可视化
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
#                                 ┌───────────────────┐
#                                 │   2D仿真器        │
#                                 └───────────────────┘
#                                   │          │
#                              TF ↓     Marker ↓
#                                   │          │
#                              ┌───────────────────┐
#                              │      RViz2       │
#                              └───────────────────┘

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

    initial_x_arg = DeclareLaunchArgument('initial_x', default_value='0.15')
    initial_y_arg = DeclareLaunchArgument('initial_y', default_value='0.15')

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
    # 节点4：2D仿真器
    # ----------------------------------------------------------
    sim_node = Node(
        package='agv_simulator',
        executable='agv_sim_node',
        name='agv_simulator',
        parameters=[{
            'initial_x': LaunchConfiguration('initial_x'),
            'initial_y': LaunchConfiguration('initial_y'),
            'initial_theta': 0.0,
            'update_rate': 50.0,
            'base_frame': 'base_link',
        }],
        output='screen'
    )

    # ----------------------------------------------------------
    # 节点5：导航协调器
    # ----------------------------------------------------------
    coordinator_node = Node(
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
    # 节点6：RViz2可视化
    # ----------------------------------------------------------
    rviz_config = os.path.join(
        get_package_share_directory('agv_navigation'),
        '..', '..', '..', 'agv', 'config', 'agv_navigation.rviz')

    # 如果配置文件不存在，使用默认路径
    if not os.path.exists(rviz_config):
        rviz_config = os.path.join(
            os.path.expanduser('~'), 'AGV', 'config', 'agv_navigation.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # ----------------------------------------------------------
    # 启动提示
    # ----------------------------------------------------------
    startup_log = LogInfo(msg=[
        '\n============================================================\n',
        '  AGV导航仿真系统已启动！\n',
        '  \n',
        '  节点：\n',
        '    1. 地图服务器 (/map)\n',
        '    2. A*全局规划器 (/plan_path)\n',
        '    3. DWA局部避障器 (/cmd_vel)\n',
        '    4. 2D仿真器 (TF + Marker)\n',
        '    5. 导航协调器 (Navigate Action)\n',
        '    6. RViz2 可视化\n',
        '  \n',
        '  测试方式：\n',
        '    # 发送导航请求\n',
        '    ros2 action send_goal /navigate agv_interfaces/action/Navigate \\\n',
        '      \'{goal_position: {x: 3.0, y: 2.0, z: 0.0}, goal_theta: 0.0, \\\n',
        '        use_current_pose: true, max_planning_time: 10.0}\'\n',
        '============================================================\n',
    ])

    return LaunchDescription([
        yaml_arg,
        initial_x_arg,
        initial_y_arg,
        startup_log,
        map_server_node,
        global_planner_node,
        dwa_planner_node,
        sim_node,
        coordinator_node,
        rviz_node,
    ])
