#include "agv_scheduler/task_scheduler.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<agv_scheduler::TaskScheduler>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
