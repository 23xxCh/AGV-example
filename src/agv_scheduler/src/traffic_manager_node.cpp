#include "agv_scheduler/traffic_manager.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<agv_scheduler::TrafficManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
