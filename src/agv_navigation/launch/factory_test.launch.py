# ============================================================
# factory_test.launch.py - 工厂测试场景启动文件
# ============================================================
#
# 【功能】启动工厂场景的多AGV导航测试
#   - 使用工厂地图（8m x 6m，6货架+2堆垛+2工作站）
#   - 两台AGV初始位置在仓库入口
#   - 包含动态障碍物和数据记录器
#
# 【使用方法】
#   ros2 launch agv_navigation factory_test.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    LogInfo,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def create_agv_group(agv_id: str, initial_x: float, initial_y: float) -> list:
    """为单台AGV创建所有节点"""
    planner_dir = get_package_share_directory('agv_global_planner')
    local_planner_dir = get_package_share_directory('agv_local_planner')

    base_frame = agv_id + '_base_link'
    dwa_params = os.path.join(local_planner_dir, 'config', 'local_planner_params.yaml')

    coordinator_params = {
        'planner_service': '/plan_path',
        'feedback_rate': 5.0,
        'goal_tolerance_xy': 0.15,
        'goal_tolerance_yaw': 0.2,
        'base_frame': base_frame,
        'map_frame': 'map',
        'agv_id': agv_id,
        'avg_speed': 0.3,
        'max_reservation_retries': 10,
        'reservation_retry_interval': 2.0,
        'stuck_timeout': 10.0,
        'stuck_distance': 0.1,
        'max_replans': 5,
    }

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
            'initial_battery': 100.0,
            'battery_drain_rate': 0.05,
            'battery_charge_rate': 0.5,
            'low_battery_threshold': 20.0,
            'charging_station_x': 3.8 if agv_id == 'agv_001' else -3.8,
            'charging_station_y': -2.0,
        }],
        output='screen',
    )

    dwa_node = Node(
        package='agv_local_planner',
        executable='dwa_planner_node',
        name='dwa_planner',
        parameters=[dwa_params, {'base_frame': base_frame}],
        remappings=[('map', '/map'), ('dynamic_costmap', '/dynamic_costmap')],
        output='screen',
    )

    coordinator_node = Node(
        package='agv_navigation',
        executable='navigation_coordinator_node',
        name='navigation_coordinator',
        parameters=[coordinator_params],
        output='screen',
    )

    return GroupAction([
        PushRosNamespace(agv_id),
        sim_node,
        dwa_node,
        coordinator_node,
    ])


def generate_launch_description():
    """生成工厂测试启动描述"""

    map_dir = get_package_share_directory('agv_map')
    planner_dir = get_package_share_directory('agv_global_planner')
    scheduler_dir = get_package_share_directory('agv_scheduler')

    # 工厂地图
    factory_map = os.path.join(os.path.expanduser('~'), 'AGV', 'maps', 'factory_map.yaml')

    yaml_arg = DeclareLaunchArgument(
        'yaml_path',
        default_value=factory_map,
        description='工厂地图YAML配置文件路径'
    )

    # 全局节点
    map_server_node = Node(
        package='agv_map',
        executable='map_server_node',
        name='map_server',
        parameters=[{'yaml_path': LaunchConfiguration('yaml_path')}],
        output='screen'
    )

    global_planner_node = Node(
        package='agv_global_planner',
        executable='global_planner_node',
        name='astar_planner',
        parameters=[os.path.join(planner_dir, 'config', 'global_planner_params.yaml')],
        output='screen'
    )

    traffic_manager_node = Node(
        package='agv_scheduler',
        executable='traffic_manager_node',
        name='traffic_manager',
        parameters=[os.path.join(scheduler_dir, 'config', 'scheduler_params.yaml')],
        output='screen'
    )

    task_scheduler_node = Node(
        package='agv_scheduler',
        executable='task_scheduler_node',
        name='task_scheduler',
        parameters=[
            os.path.join(scheduler_dir, 'config', 'scheduler_params.yaml'),
        ],
        output='screen'
    )

    # 动态障碍物（减少数量以提高测试稳定性）
    obstacle_manager_node = Node(
        package='agv_simulator',
        executable='obstacle_manager_node',
        name='obstacle_manager',
        parameters=[{
            'num_pedestrians': 2,
            'num_forklifts': 1,
            'num_carts': 0,
            'update_rate': 10.0,
            'map_width': 8.0,
            'map_height': 6.0,
            'publish_markers': True,
        }],
        output='screen'
    )

    # 数据记录器
    data_recorder_node = Node(
        package='agv_navigation',
        executable='data_recorder_node',
        name='data_recorder',
        parameters=[{
            'log_dir': os.path.join(os.path.expanduser('~'), 'AGV', 'logs'),
            'agv_ids': ['agv_001', 'agv_002'],
            'record_status': True,
            'record_cmd_vel': True,
            'record_path': True,
        }],
        output='screen'
    )

    # AGV 1（仓库入口左侧）
    agv1_group = create_agv_group('agv_001', initial_x=-3.8, initial_y=-2.5)

    # AGV 2（仓库入口右侧）
    agv2_group = create_agv_group('agv_002', initial_x=3.8, initial_y=-2.5)

    # RViz
    rviz_config = os.path.join(os.path.expanduser('~'), 'AGV', 'config', 'multi_agv_navigation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    startup_log = LogInfo(msg=[
        '\n============================================================\n',
        '  工厂测试场景已启动！\n',
        '  \n',
        '  地图: 8m x 6m 工厂仓库\n',
        '  障碍物: 6货架 + 2堆垛 + 2工作站 + 8动态障碍\n',
        '  \n',
        '  AGV 1 (agv_001): 入口左侧 (-3.8, -2.5)\n',
        '  AGV 2 (agv_002): 入口右侧 (3.8, -2.5)\n',
        '  \n',
        '  运行测试:\n',
        '    bash ~/AGV/test/run_factory_tests.sh\n',
        '============================================================\n',
    ])

    return LaunchDescription([
        yaml_arg,
        startup_log,
        map_server_node,
        global_planner_node,
        traffic_manager_node,
        task_scheduler_node,
        obstacle_manager_node,
        data_recorder_node,
        agv1_group,
        agv2_group,
        rviz_node,
    ])
