/**
 * ============================================================
 * global_planner_node.cpp - 全局路径规划器节点入口
 * ============================================================
 *
 * 【这是什么】
 * A*全局路径规划器的main函数入口
 *
 * 【启动方式】
 *  方式1（ros2 run）：
 *    ros2 run agv_global_planner global_planner_node
 *
 *  方式2（带参数）：
 *    ros2 run agv_global_planner global_planner_node \
 *      --ros-args -p heuristic_type:=2 -p allow_diagonal:=true
 *
 *  方式3（通过launch文件，推荐）：
 *    ros2 launch agv_navigation navigation.launch.py
 */

#include "agv_global_planner/astar_planner.hpp"
#include <memory>

int main(int argc, char ** argv)
{
  // 初始化ROS2运行环境
  rclcpp::init(argc, argv);

  // 创建A*规划器节点
  auto planner_node = std::make_shared<agv_global_planner::AstarPlanner>();

  RCLCPP_INFO(planner_node->get_logger(),
    "===== AGV A*全局路径规划器已启动 =====");
  RCLCPP_INFO(planner_node->get_logger(),
    "等待代价地图和路径规划请求...");

  // 进入事件循环
  // spin()会阻塞，持续处理：
  // - 代价地图订阅回调（收到新地图时）
  // - 路径规划服务回调（收到规划请求时）
  rclcpp::spin(planner_node);

  // 关闭ROS2
  rclcpp::shutdown();

  return 0;
}
