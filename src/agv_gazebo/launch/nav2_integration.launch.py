# ============================================================
# nav2_integration.launch.py - Nav2集成启动文件
# ============================================================
#
# 【功能】
# 使用Nav2生命周期管理器启动导航系统：
#   1. 地图服务器
#   2. 代价地图（全局+局部）
#   3. 规划器服务器（自定义A*）
#   4. 控制器服务器（自定义DWA）
#   5. 行为树导航器
#   6. 生命周期管理器
#
# 【使用方法】
#   ros2 launch agv_gazebo nav2_integration.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成Nav2集成启动描述"""

    gazebo_dir = get_package_share_directory('agv_gazebo')
    nav2_params = os.path.join(gazebo_dir, 'config', 'nav2_params.yaml')

    # ----------------------------------------------------------
    # Launch参数
    # ----------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时间'
    )

    # ----------------------------------------------------------
    # 地图服务器
    # ----------------------------------------------------------
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

    # ----------------------------------------------------------
    # Nav2 代价地图服务器
    # ----------------------------------------------------------
    global_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='costmap_server',
        name='global_costmap',
        parameters=[nav2_params],
        output='screen'
    )

    local_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='costmap_server',
        name='local_costmap',
        parameters=[nav2_params],
        output='screen'
    )

    # ----------------------------------------------------------
    # 自定义A*规划器（作为独立节点运行）
    # ----------------------------------------------------------
    planner_dir = get_package_share_directory('agv_global_planner')
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
    # 自定义DWA避障器（作为独立节点运行）
    # ----------------------------------------------------------
    local_planner_dir = get_package_share_directory('agv_local_planner')
    dwa_node = Node(
        package='agv_local_planner',
        executable='dwa_planner_node',
        name='dwa_planner',
        parameters=[
            os.path.join(local_planner_dir, 'config', 'local_planner_params.yaml'),
        ],
        remappings=[('map', '/map')],
        output='screen'
    )

    # ----------------------------------------------------------
    # 交通管理器和任务调度器
    # ----------------------------------------------------------
    scheduler_dir = get_package_share_directory('agv_scheduler')

    traffic_manager_node = Node(
        package='agv_scheduler',
        executable='traffic_manager_node',
        name='traffic_manager',
        parameters=[
            os.path.join(scheduler_dir, 'config', 'scheduler_params.yaml'),
        ],
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

    # ----------------------------------------------------------
    # 启动提示
    # ----------------------------------------------------------
    startup_log = LogInfo(msg=[
        '\n============================================================\n',
        '  Nav2集成模式已启动！\n',
        '  \n',
        '  组件：\n',
        '    - 地图服务器 (/map)\n',
        '    - A*全局规划器 (/plan_path)\n',
        '    - DWA局部避障器 (/cmd_vel)\n',
        '    - 交通管理器 (/reserve_path, /release_path)\n',
        '    - 任务调度器 (/assign_task)\n',
        '  \n',
        '  测试任务分配：\n',
        '    ros2 service call /assign_task agv_interfaces/srv/AssignTask \\\n',
        '      \'{task: {goal_x: 2.0, goal_y: 1.5, goal_theta: 0.0, priority: 1}}\'\n',
        '============================================================\n',
    ])

    return LaunchDescription([
        use_sim_time_arg,
        startup_log,
        map_server_node,
        global_costmap_node,
        local_costmap_node,
        global_planner_node,
        dwa_node,
        traffic_manager_node,
        task_scheduler_node,
    ])
