#include "agv_scheduler/task_scheduler.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<agv_scheduler::TaskScheduler>();

  // 使用多线程执行器，避免 wait_for_action_server 阻塞导致死锁
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
