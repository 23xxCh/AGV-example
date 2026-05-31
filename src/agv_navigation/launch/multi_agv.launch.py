# ============================================================
# multi_agv.launch.py - 多AGV导航系统启动文件
# ============================================================
#
# 【功能】一键启动多AGV导航仿真系统
#   全局节点（共享）：
#     1. 地图服务器 → 发布全局地图
#     2. A*全局规划器 → 提供路径规划服务
#     3. 交通管理器 → 路径预约和冲突检测
#     4. 任务调度器 → AGV任务分配
#
#   每台AGV（独立命名空间）：
#     - 2D仿真器 → 发布TF和机器人Marker
#     - DWA局部避障器 → 发布速度指令
#     - 导航协调器 → 串联规划和避障
#
# 【使用方法】
#   ros2 launch agv_navigation multi_agv.launch.py
#
#   测试任务分配：
#   ros2 service call /assign_task agv_interfaces/srv/AssignTask \
#     '{task: {goal_x: 2.0, goal_y: 1.5, goal_theta: 0.0, priority: 1}}'

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    LogInfo,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def create_agv_group(agv_id: str, initial_x: float, initial_y: float,
                     map_server_ns: bool = False) -> list:
    """
    为单台AGV创建所有节点（放在独立命名空间中）

    参数:
      agv_id: AGV编号，如 "agv_001"
      initial_x, initial_y: 初始位置
      map_server_ns: 是否将地图服务器也放在命名空间中（False=共享全局地图）
    """
    planner_dir = get_package_share_directory('agv_global_planner')
    local_planner_dir = get_package_share_directory('agv_local_planner')

    # 每台AGV的base_frame用agv_id区分，避免TF冲突
    base_frame = agv_id + '_base_link'

    # DWA参数文件
    dwa_params = os.path.join(local_planner_dir, 'config', 'local_planner_params.yaml')

    # 导航协调器参数
    coordinator_params = {
        # 使用绝对路径调用全局规划服务（不在命名空间内）
        'planner_service': '/plan_path',
        'feedback_rate': 5.0,
        'goal_tolerance_xy': 0.3,
        'goal_tolerance_yaw': 0.2,
        'base_frame': base_frame,
        'map_frame': 'map',
        'agv_id': agv_id,
        'avg_speed': 0.3,
        'max_reservation_retries': 10,
        'reservation_retry_interval': 2.0,
        # 路径重规划参数
        'stuck_timeout': 10.0,      # 卡住10秒触发重规划
        'stuck_distance': 0.1,      # 距离变化<0.1m视为卡住
        'max_replans': 3,           # 最多重规划3次
    }

    # ----------------------------------------------------------
    # AGV仿真器节点
    # ----------------------------------------------------------
    sim_node = Node(
        package='agv_simulator',
        executable='agv_sim_node',
        name='agv_simulator',
        parameters=[{
            'initial_x': initial_x,
            'initial_y': initial_y,
            'initial_theta': 0.0,
            'update_rate': 50.0,
            'base_frame': base_frame,
            'agv_id': agv_id,
        }],
        output='screen',
    )

    # ----------------------------------------------------------
    # DWA局部避障器节点
    # ----------------------------------------------------------
    # 需要将全局 /map 话题重映射到命名空间内
    dwa_node = Node(
        package='agv_local_planner',
        executable='dwa_planner_node',
        name='dwa_planner',
        parameters=[dwa_params, {
            'base_frame': base_frame,
        }],
        remappings=[
            # 将命名空间内的 map 话题映射到全局 /map
            ('map', '/map'),
        ],
        output='screen',
    )

    # ----------------------------------------------------------
    # 导航协调器节点
    # ----------------------------------------------------------
    coordinator_node = Node(
        package='agv_navigation',
        executable='navigation_coordinator_node',
        name='navigation_coordinator',
        parameters=[coordinator_params],
        output='screen',
    )

    # 将所有节点放入命名空间组
    return GroupAction([
        PushRosNamespace(agv_id),
        sim_node,
        dwa_node,
        coordinator_node,
    ])


def generate_launch_description():
    """生成多AGV导航系统启动描述"""

    # 获取各包的共享目录
    map_dir = get_package_share_directory('agv_map')
    planner_dir = get_package_share_directory('agv_global_planner')
    scheduler_dir = get_package_share_directory('agv_scheduler')

    # ----------------------------------------------------------
    # 声明Launch参数
    # ----------------------------------------------------------
    yaml_arg = DeclareLaunchArgument(
        'yaml_path',
        default_value=os.path.join(map_dir, 'maps', 'warehouse_map.yaml'),
        description='地图YAML配置文件路径'
    )

    # ----------------------------------------------------------
    # 全局节点（所有AGV共享）
    # ----------------------------------------------------------

    # 地图服务器
    map_server_node = Node(
        package='agv_map',
        executable='map_server_node',
        name='map_server',
        parameters=[{
            'yaml_path': LaunchConfiguration('yaml_path'),
        }],
        output='screen'
    )

    # A*全局路径规划器
    global_planner_node = Node(
        package='agv_global_planner',
        executable='global_planner_node',
        name='astar_planner',
        parameters=[
            os.path.join(planner_dir, 'config', 'global_planner_params.yaml'),
        ],
        output='screen'
    )

    # 交通管理器
    traffic_manager_node = Node(
        package='agv_scheduler',
        executable='traffic_manager_node',
        name='traffic_manager',
        parameters=[
            os.path.join(scheduler_dir, 'config', 'scheduler_params.yaml'),
        ],
        output='screen'
    )

    # 任务调度器
    task_scheduler_node = Node(
        package='agv_scheduler',
        executable='task_scheduler_node',
        name='task_scheduler',
        parameters=[
            os.path.join(scheduler_dir, 'config', 'scheduler_params.yaml'),
        ],
        output='screen'
    )

    # ----------------------------------------------------------
    # AGV 1（命名空间: agv_001）
    # ----------------------------------------------------------
    agv1_group = create_agv_group('agv_001', initial_x=0.15, initial_y=0.15)

    # ----------------------------------------------------------
    # AGV 2（命名空间: agv_002）
    # ----------------------------------------------------------
    agv2_group = create_agv_group('agv_002', initial_x=0.15, initial_y=0.55)

    # ----------------------------------------------------------
    # RViz2可视化
    # ----------------------------------------------------------
    rviz_config = os.path.join(
        os.path.expanduser('~'), 'AGV', 'config', 'multi_agv_navigation.rviz')

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
        '  多AGV导航仿真系统已启动！\n',
        '  \n',
        '  全局节点：\n',
        '    - 地图服务器 (/map)\n',
        '    - A*全局规划器 (/plan_path)\n',
        '    - 交通管理器 (/reserve_path, /release_path)\n',
        '    - 任务调度器 (/assign_task)\n',
        '  \n',
        '  AGV 1 (agv_001): 初始位置 (0.15, 0.15)\n',
        '  AGV 2 (agv_002): 初始位置 (0.15, 0.55)\n',
        '  \n',
        '  测试任务分配：\n',
        '    ros2 service call /assign_task agv_interfaces/srv/AssignTask \\\n',
        '      \'{task: {goal_x: 2.0, goal_y: 1.5, goal_theta: 0.0, priority: 1}}\'\n',
        '============================================================\n',
    ])

    return LaunchDescription([
        yaml_arg,
        startup_log,
        map_server_node,
        global_planner_node,
        traffic_manager_node,
        task_scheduler_node,
        agv1_group,
        agv2_group,
        rviz_node,
    ])
