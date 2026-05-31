cmake_minimum_required(VERSION 3.8)
project(agv_scheduler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(agv_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 交通管理器节点
add_executable(traffic_manager_node
  src/traffic_manager.cpp
  src/traffic_manager_node.cpp
)
ament_target_dependencies(traffic_manager_node
  rclcpp nav_msgs geometry_msgs std_msgs agv_interfaces tf2_ros
)
target_include_directories(traffic_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 任务调度器节点
add_executable(task_scheduler_node
  src/task_scheduler.cpp
  src/task_scheduler_node.cpp
)
ament_target_dependencies(task_scheduler_node
  rclcpp rclcpp_action nav_msgs geometry_msgs std_msgs agv_interfaces tf2_ros
)
target_include_directories(task_scheduler_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS traffic_manager_node task_scheduler_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
