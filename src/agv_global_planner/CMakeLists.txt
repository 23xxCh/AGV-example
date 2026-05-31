cmake_minimum_required(VERSION 3.8)
project(agv_global_planner)

# ============================================================
# agv_global_planner 包的构建配置
# ============================================================

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(agv_interfaces REQUIRED)

# ============================================================
# A*规划器核心库
# ============================================================
# 分为库和节点两部分：
# - 库：包含A*算法实现，可被其他代码复用或作为Nav2插件加载
# - 节点：独立的ROS2节点，直接运行即可使用

add_library(agv_astar_planner SHARED
  src/astar_planner.cpp
  src/astar_search.cpp
)
ament_target_dependencies(agv_astar_planner
  rclcpp nav_msgs geometry_msgs std_msgs
  nav2_core nav2_costmap_2d nav2_util pluginlib tf2_ros
  agv_interfaces
)
target_include_directories(agv_astar_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 独立运行的全局规划器节点（不依赖Nav2也可运行）
add_executable(global_planner_node
  src/global_planner_node.cpp
)
target_link_libraries(global_planner_node agv_astar_planner)
ament_target_dependencies(global_planner_node
  rclcpp nav_msgs nav2_costmap_2d agv_interfaces
)

# ============================================================
# 插件描述文件安装（让Nav2能发现和加载我们的规划器）
# ============================================================
# pluginlib通过读取XML文件来发现可用的插件类
pluginlib_export_plugin_description_file(nav2_core agv_global_planner_plugin.xml)

install(TARGETS agv_astar_planner global_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES agv_global_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
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

  # A*搜索算法单元测试
  ament_add_gtest(test_astar_search test/test_astar_search.cpp)
  target_link_libraries(test_astar_search agv_astar_planner)
endif()

ament_package()
