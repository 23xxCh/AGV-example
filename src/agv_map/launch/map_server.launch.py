# ============================================================
# map_server.launch.py - 地图服务器启动文件
# ============================================================
#
# 【Launch文件是什么】
# Launch文件是ROS2的"一键启动脚本"，可以：
# - 同时启动多个节点
# - 传递参数给节点
# - 设置节点间的依赖关系（谁先启动）
# - 加载参数文件
#
# 【为什么用Python写Launch文件】
# ROS2推荐用Python写launch文件（而不是XML），因为：
# - 可以写逻辑（if/else、循环、函数调用）
# - 更灵活，可以动态生成启动配置
# - 支持条件启动、事件处理等高级功能
#
# 【使用方法】
#   ros2 launch agv_map map_server.launch.py
# 或带参数：
#   ros2 launch agv_map map_server.launch.py yaml_path:=/path/to/map.yaml

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成启动描述"""

    # ----------------------------------------------------------
    # 获取包的共享目录路径
    # ----------------------------------------------------------
    # get_package_share_directory 返回安装后的包目录
    # 例如：/home/user/AGV/install/agv_map/share/agv_map
    # 地图文件、配置文件都安装到这个目录下
    map_dir = get_package_share_directory('agv_map')

    # ----------------------------------------------------------
    # 声明Launch参数
    # ----------------------------------------------------------
    # Launch参数可以在命令行中覆盖
    # 格式：ros2 launch package launch_file param_name:=value

    # 地图YAML配置文件路径
    yaml_arg = DeclareLaunchArgument(
        'yaml_path',
        default_value=os.path.join(map_dir, 'maps', 'warehouse_map.yaml'),
        description='地图YAML配置文件的完整路径'
    )

    # ----------------------------------------------------------
    # 定义地图服务器节点
    # ----------------------------------------------------------
    # Node()创建一个ROS2节点启动动作
    # - package: 包名
    # - executable: 可执行文件名（CMakeLists.txt中install的目标名）
    # - name: 节点运行时的名称（覆盖代码中的名称）
    # - parameters: 传递给节点的参数列表
    #   可以是字典（直接指定参数）或YAML文件路径
    # - output: 日志输出方式
    #   'screen' = 输出到终端（方便调试）
    #   'log' = 只输出到日志文件

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
    # 返回LaunchDescription
    # ----------------------------------------------------------
    # LaunchDescription包含所有要执行的动作
    # 动作按顺序执行，但节点是并行运行的
    return LaunchDescription([
        # 先声明参数
        yaml_arg,
        # 然后启动节点
        map_server_node,
    ])
