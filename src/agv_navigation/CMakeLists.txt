cmake_minimum_required(VERSION 3.8)
project(agv_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(agv_interfaces REQUIRED)

# 导航协调器库
add_library(navigation_coordinator_lib SHARED
  src/navigation_coordinator.cpp
)
ament_target_dependencies(navigation_coordinator_lib
  rclcpp rclcpp_action nav_msgs geometry_msgs std_msgs
  tf2_ros nav2_costmap_2d agv_interfaces
)
target_include_directories(navigation_coordinator_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 导航协调器节点
add_executable(navigation_coordinator_node
  src/navigation_coordinator_node.cpp
)
target_link_libraries(navigation_coordinator_node navigation_coordinator_lib)
ament_target_dependencies(navigation_coordinator_node
  rclcpp rclcpp_action agv_interfaces
)

# 协同搬运协调器节点
add_executable(cooperative_transport_node
  src/cooperative_transport.cpp
)
ament_target_dependencies(cooperative_transport_node
  rclcpp nav_msgs geometry_msgs std_msgs
  tf2_ros agv_interfaces
)

# 数据记录器节点
add_executable(data_recorder_node
  src/data_recorder.cpp
  src/data_recorder_node.cpp
)
target_include_directories(data_recorder_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(data_recorder_node
  rclcpp geometry_msgs nav_msgs agv_interfaces
)

# 数据回放器节点
add_executable(data_replayer_node
  src/data_replayer.cpp
  src/data_replayer_node.cpp
)
target_include_directories(data_replayer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(data_replayer_node
  rclcpp geometry_msgs agv_interfaces
)

# 多楼层管理器节点
add_executable(floor_manager_node
  src/floor_manager.cpp
  src/floor_manager_node.cpp
)
target_include_directories(floor_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(floor_manager_node
  rclcpp nav_msgs geometry_msgs agv_interfaces
)

install(TARGETS navigation_coordinator_lib navigation_coordinator_node cooperative_transport_node data_recorder_node data_replayer_node floor_manager_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
