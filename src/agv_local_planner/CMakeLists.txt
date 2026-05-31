cmake_minimum_required(VERSION 3.8)
project(agv_local_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(agv_interfaces REQUIRED)

# DWA算法核心库
add_library(dwa_planner_lib SHARED
  src/dwa_search.cpp
  src/dwa_planner.cpp
)
ament_target_dependencies(dwa_planner_lib
  rclcpp nav_msgs geometry_msgs std_msgs
  nav2_core nav2_costmap_2d nav2_util
  tf2 tf2_ros tf2_geometry_msgs pluginlib
  visualization_msgs agv_interfaces
)
target_include_directories(dwa_planner_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 独立运行的DWA规划器节点
add_executable(dwa_planner_node
  src/dwa_planner_node.cpp
)
target_link_libraries(dwa_planner_node dwa_planner_lib)
ament_target_dependencies(dwa_planner_node
  rclcpp nav_msgs geometry_msgs agv_interfaces
)

install(TARGETS dwa_planner_lib dwa_planner_node
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
  find_package(ament_cmake_gtest REQUIRED)

  # DWA算法单元测试
  ament_add_gtest(test_dwa_search test/test_dwa_search.cpp)
  target_link_libraries(test_dwa_search dwa_planner_lib)
endif()

ament_package()
