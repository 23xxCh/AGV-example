# ============================================================
# gazebo_simulation.launch.py - Gazebo 3D仿真启动文件
# ============================================================
#
# 【功能】
# 启动Gazebo仓库仿真环境，替代2D仿真器：
#   1. Gazebo服务器 + 仓库世界
#   2. 两台AGV（带激光雷达、IMU）
#   3. 机器人状态发布器（TF）
#   4. 导航系统（全局规划 + DWA避障）
#
# 【使用方法】
#   ros2 launch agv_gazebo gazebo_simulation.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    ExecuteProcess,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_agv_spawner(agv_id: str, x: float, y: float, yaw: float,
                        urdf_path: str) -> list:
    """为单台AGV创建Gazebo生成器和状态发布器"""

    # AGV命名空间内的节点
    nodes = []

    # 1. 机器人状态发布器（TF）
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=agv_id,
        name='robot_state_publisher',
        parameters=[{
            'robot_description': open(urdf_path).read(),
            'use_sim_time': True,
        }],
        output='screen'
    )
    nodes.append(rsp_node)

    # 2. Gazebo生成器（在指定位置生成AGV模型）
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', agv_id,
            '-file', urdf_path,
            '-x', str(x),
            '-y', str(y),
            '-z', '0.05',
            '-Y', str(yaw),
            '-robot_namespace', agv_id,
        ],
        output='screen'
    )
    nodes.append(spawn_entity)

    return nodes


def generate_launch_description():
    """生成Gazebo仿真启动描述"""

    # 获取包路径
    gazebo_dir = get_package_share_directory('agv_gazebo')
    planner_dir = get_package_share_directory('agv_global_planner')
    local_planner_dir = get_package_share_directory('agv_local_planner')
    scheduler_dir = get_package_share_directory('agv_scheduler')

    # URDF文件路径
    urdf_path = os.path.join(gazebo_dir, 'urdf', 'agv_model.urdf')

    # 世界文件
    world_path = os.path.join(gazebo_dir, 'worlds', 'warehouse.world')

    # DWA参数
    dwa_params = os.path.join(local_planner_dir, 'config', 'local_planner_params.yaml')

    # ----------------------------------------------------------
    # Launch参数
    # ----------------------------------------------------------
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_path,
        description='Gazebo世界文件路径'
    )

    # ----------------------------------------------------------
    # Gazebo服务器
    # ----------------------------------------------------------
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                        'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                        'launch', 'gzclient.launch.py')
        )
    )

    # ----------------------------------------------------------
    # 全局节点（与2D仿真共享）
    # ----------------------------------------------------------

    # 地图服务器
    map_dir = get_package_share_directory('agv_map')
    map_server_node = Node(
        package='agv_map',
        executable='map_server_node',
        name='map_server',
        parameters=[{
            'yaml_path': os.path.join(map_dir, 'maps', 'warehouse_map.yaml'),
        }],
        output='screen'
    )

    # A*全局规划器
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
    agv1_nodes = create_agv_spawner('agv_001', x=0.15, y=0.15, yaw=0.0,
                                     urdf_path=urdf_path)

    # AGV 1 DWA
    agv1_dwa = Node(
        package='agv_local_planner',
        executable='dwa_planner_node',
        namespace='agv_001',
        name='dwa_planner',
        parameters=[dwa_params, {'base_frame': 'agv_001_base_link'}],
        remappings=[('map', '/map'), ('dynamic_costmap', '/dynamic_costmap')],
        output='screen'
    )

    # AGV 1 导航协调器
    agv1_coordinator = Node(
        package='agv_navigation',
        executable='navigation_coordinator_node',
        namespace='agv_001',
        name='navigation_coordinator',
        parameters=[{
            'planner_service': '/plan_path',
            'feedback_rate': 5.0,
            'goal_tolerance_xy': 0.15,
            'base_frame': 'agv_001_base_link',
            'map_frame': 'map',
            'agv_id': 'agv_001',
        }],
        output='screen'
    )

    # ----------------------------------------------------------
    # AGV 2（命名空间: agv_002）
    # ----------------------------------------------------------
    agv2_nodes = create_agv_spawner('agv_002', x=0.15, y=0.55, yaw=0.0,
                                     urdf_path=urdf_path)

    # AGV 2 DWA
    agv2_dwa = Node(
        package='agv_local_planner',
        executable='dwa_planner_node',
        namespace='agv_002',
        name='dwa_planner',
        parameters=[dwa_params, {'base_frame': 'agv_002_base_link'}],
        remappings=[('map', '/map'), ('dynamic_costmap', '/dynamic_costmap')],
        output='screen'
    )

    # AGV 2 导航协调器
    agv2_coordinator = Node(
        package='agv_navigation',
        executable='navigation_coordinator_node',
        namespace='agv_002',
        name='navigation_coordinator',
        parameters=[{
            'planner_service': '/plan_path',
            'feedback_rate': 5.0,
            'goal_tolerance_xy': 0.15,
            'base_frame': 'agv_002_base_link',
            'map_frame': 'map',
            'agv_id': 'agv_002',
        }],
        output='screen'
    )

    # ----------------------------------------------------------
    # 启动提示
    # ----------------------------------------------------------
    startup_log = LogInfo(msg=[
        '\n============================================================\n',
        '  AGV Gazebo 3D仿真系统已启动！\n',
        '  \n',
        '  Gazebo世界: warehouse.world\n',
        '  AGV 1 (agv_001): (0.15, 0.15)\n',
        '  AGV 2 (agv_002): (0.15, 0.55)\n',
        '  \n',
        '  测试任务分配：\n',
        '    ros2 service call /assign_task agv_interfaces/srv/AssignTask \\\n',
        '      \'{task: {goal_x: 2.0, goal_y: 1.5, goal_theta: 0.0, priority: 1}}\'\n',
        '============================================================\n',
    ])

    return LaunchDescription([
        world_arg,
        startup_log,
        # Gazebo
        gazebo_server,
        gazebo_client,
        # 全局节点
        map_server_node,
        global_planner_node,
        traffic_manager_node,
        task_scheduler_node,
        # AGV 1
        *agv1_nodes,
        agv1_dwa,
        agv1_coordinator,
        # AGV 2
        *agv2_nodes,
        agv2_dwa,
        agv2_coordinator,
    ])
