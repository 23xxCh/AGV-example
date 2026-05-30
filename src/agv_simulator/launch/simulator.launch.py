# ============================================================
# simulator.launch.py - AGV 2D仿真器启动文件
# ============================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    initial_x_arg = DeclareLaunchArgument('initial_x', default_value='0.15')
    initial_y_arg = DeclareLaunchArgument('initial_y', default_value='0.15')
    initial_theta_arg = DeclareLaunchArgument('initial_theta', default_value='0.0')

    sim_node = Node(
        package='agv_simulator',
        executable='agv_sim_node',
        name='agv_simulator',
        parameters=[{
            'initial_x': LaunchConfiguration('initial_x'),
            'initial_y': LaunchConfiguration('initial_y'),
            'initial_theta': LaunchConfiguration('initial_theta'),
            'update_rate': 50.0,
            'base_frame': 'base_link',
        }],
        output='screen'
    )

    return LaunchDescription([
        initial_x_arg,
        initial_y_arg,
        initial_theta_arg,
        sim_node,
    ])
