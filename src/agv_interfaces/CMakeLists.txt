cmake_minimum_required(VERSION 3.8)
project(agv_interfaces)

# 技术要点：
# find_package 是CMake查找依赖包的方式，REQUIRED表示必须找到，否则报错
# ros2接口包的核心依赖是 rosidl_default_generators，它会读取 msg/srv/action 文件
# 自动生成C++头文件和Python模块

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# rosidl_generate_interfaces 是核心宏：
# 它会扫描我们定义的 .msg/.srv/.action 文件，生成以下代码：
#   - C++ 头文件（放在 install/ 目录下，其他包 #include 使用）
#   - Python 模块（可以直接 import 使用）
#   - 共享库（运行时需要加载）
#
# 语法：rosidl_generate_interfaces(${PROJECT_NAME} 文件1 文件2 ... DEPENDENCIES 依赖的msg包)
# DEPENDENCIES 里要列出我们 msg 文件中用到的其他包的消息类型

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NavGoal.msg"
  "msg/PathWithCost.msg"
  "msg/AGVStatus.msg"
  "srv/PathPlan.srv"
  "srv/GetMap.srv"
  "action/Navigate.action"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs
)

ament_package()
