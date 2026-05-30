/**
 * dwa_planner_node.cpp - DWA局部规划器节点入口
 *
 * 启动方式：
 *   ros2 run agv_local_planner dwa_planner_node
 * 或通过launch文件：
 *   ros2 launch agv_local_planner dwa_planner.launch.py
 */

#include "agv_local_planner/dwa_planner.hpp"
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<agv_local_planner::DWAPlanner>();

  RCLCPP_INFO(node->get_logger(), "===== DWA局部避障规划器已启动 =====");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
