cmake_minimum_required(VERSION 3.8)
project(agv_map)

# ============================================================
# agv_map 包的构建配置
# ============================================================
# 技术要点：
# - CMake是C/C++项目的标准构建工具，ROS2使用ament_cmake扩展
# - find_package查找依赖，target_link_libraries链接依赖
# - ament_target_dependencies 是ROS2特有的，自动处理包含路径和链接

# 强制使用C++17标准（支持auto、lambda、optional等现代特性）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(agv_interfaces REQUIRED)

# ============================================================
# 构建目标：map_server_node
# ============================================================
# add_library 创建共享库（.so文件），便于其他代码复用
# add_executable 创建可执行程序

# 地图服务器核心库（包含地图加载、处理的逻辑）
add_library(map_server_lib SHARED
  src/map_server.cpp
)
# ament_target_dependencies: 自动添加依赖的头文件路径和链接库
ament_target_dependencies(map_server_lib
  rclcpp nav_msgs geometry_msgs std_msgs tf2 tf2_ros agv_interfaces
)
# target_link_libraries: 链接非ROS包（如yaml-cpp）
target_link_libraries(map_server_lib yaml-cpp)
# 包含头文件目录
target_include_directories(map_server_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 地图服务器可执行节点（包含main函数，启动ROS2节点）
add_executable(map_server_node
  src/map_server_node.cpp
)
target_link_libraries(map_server_node map_server_lib)
ament_target_dependencies(map_server_node
  rclcpp nav_msgs agv_interfaces
)

# ============================================================
# 安装规则：告诉colcon把编译产物放到哪里
# ============================================================
# install(TARGETS ...) 安装编译目标
# install(DIRECTORY ...) 安装资源文件（地图、配置、启动文件）

install(TARGETS map_server_lib map_server_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ============================================================
# 测试配置
# ============================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  # 后续添加单元测试
endif()

ament_package()
