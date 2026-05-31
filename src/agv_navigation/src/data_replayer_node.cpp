#include "data_replayer.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<agv_navigation::DataReplayer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
