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

install(TARGETS navigation_coordinator_lib navigation_coordinator_node
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
