/**
 * navigation_coordinator_node.cpp - 导航协调器节点入口（占位）
 */

#include "navigation_coordinator.hpp"
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<agv_navigation::NavigationCoordinator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
