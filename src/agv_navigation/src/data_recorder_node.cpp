#include "data_recorder.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<agv_navigation::DataRecorder>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
