cmake_minimum_required(VERSION 3.8)
project(agv_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(agv_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(agv_sim_node src/agv_sim_node.cpp)
ament_target_dependencies(agv_sim_node
  rclcpp geometry_msgs visualization_msgs tf2 tf2_ros agv_interfaces std_msgs
)

add_executable(obstacle_manager_node src/obstacle_manager_node.cpp)
ament_target_dependencies(obstacle_manager_node
  rclcpp geometry_msgs visualization_msgs nav_msgs
)

install(TARGETS agv_sim_node obstacle_manager_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
