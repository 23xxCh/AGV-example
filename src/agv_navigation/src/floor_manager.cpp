/**
 * ============================================================
 * floor_manager.cpp - 多楼层管理器实现
 * ============================================================
 *
 * 【楼层切换流程】
 * 1. 收到跨楼层任务 → 检查AGV当前楼层
 * 2. 规划到电梯入口的路径
 * 3. AGV导航到电梯入口
 * 4. 请求电梯 → 等待电梯到达
 * 5. AGV进入电梯 → 切换楼层地图
 * 6. AGV离开电梯 → 在新楼层继续导航
 */

#include "floor_manager.hpp"
#include <fstream>

namespace agv_navigation
{

FloorManager::FloorManager(const rclcpp::NodeOptions & options)
: Node("floor_manager", options)
{
  // 声明参数
  this->declare_parameter("agv_ids", std::vector<std::string>{"agv_001", "agv_002"});
  this->declare_parameter("map_dir", std::string("~/AGV/maps"));

  agv_ids_ = this->get_parameter("agv_ids").as_string_array();
  map_dir_ = this->get_parameter("map_dir").as_string();

  // 初始化AGV楼层（默认在1楼）
  for (const auto & agv_id : agv_ids_) {
    agv_floor_[agv_id] = 1;
  }

  // 创建服务
  cross_floor_service_ = this->create_service<agv_interfaces::srv::AssignTask>(
    "assign_cross_floor_task",
    std::bind(&FloorManager::handleCrossFloorTask, this,
              std::placeholders::_1, std::placeholders::_2));

  // 创建电梯状态更新定时器
  elevator_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&FloorManager::updateElevatorStatus, this));

  RCLCPP_INFO(this->get_logger(), "多楼层管理器已启动");
}

// ============================================================
// handleCrossFloorTask - 处理跨楼层任务
// ============================================================
void FloorManager::handleCrossFloorTask(
  const std::shared_ptr<agv_interfaces::srv::AssignTask::Request> request,
  std::shared_ptr<agv_interfaces::srv::AssignTask::Response> response)
{
  // 简化实现：记录跨楼层任务
  CrossFloorTask task;
  task.task_id = request->task.task_id;
  task.target_floor = static_cast<int>(request->task.goal_theta);  // 用theta字段存储目标楼层
  task.goal_pose.position.x = request->task.goal_x;
  task.goal_pose.position.y = request->task.goal_y;
  task.status = "pending";

  std::lock_guard<std::mutex> lock(tasks_mutex_);
  cross_floor_tasks_.push_back(task);

  response->success = true;
  RCLCPP_INFO(this->get_logger(),
    "跨楼层任务已接收: %s → %d楼", task.task_id.c_str(), task.target_floor);
}

// ============================================================
// switchFloor - 切换楼层
// ============================================================
bool FloorManager::switchFloor(const std::string & agv_id, int target_floor)
{
  std::lock_guard<std::mutex> lock(floors_mutex_);

  auto it = floors_.find(target_floor);
  if (it == floors_.end()) {
    RCLCPP_ERROR(this->get_logger(), "楼层 %d 不存在", target_floor);
    return false;
  }

  // 更新AGV所在楼层
  agv_floor_[agv_id] = target_floor;

  // 发布新楼层的地图
  auto pub_it = map_pubs_.find(target_floor);
  if (pub_it != map_pubs_.end()) {
    pub_it->second->publish(it->second.costmap);
  }

  RCLCPP_INFO(this->get_logger(), "AGV %s 已切换到 %d 楼", agv_id.c_str(), target_floor);
  return true;
}

// ============================================================
// requestElevator - 请求电梯
// ============================================================
bool FloorManager::requestElevator(const std::string & elevator_id, int target_floor)
{
  std::lock_guard<std::mutex> lock(elevators_mutex_);

  auto it = elevators_.find(elevator_id);
  if (it == elevators_.end()) {
    RCLCPP_ERROR(this->get_logger(), "电梯 %s 不存在", elevator_id.c_str());
    return false;
  }

  Elevator & elevator = it->second;

  // 检查电梯是否可到达目标楼层
  bool accessible = false;
  for (int floor : elevator.accessible_floors) {
    if (floor == target_floor) {
      accessible = true;
      break;
    }
  }

  if (!accessible) {
    RCLCPP_ERROR(this->get_logger(), "电梯 %s 无法到达 %d 楼", elevator_id.c_str(), target_floor);
    return false;
  }

  if (!elevator.is_available) {
    RCLCPP_WARN(this->get_logger(), "电梯 %s 正在使用中", elevator_id.c_str());
    return false;
  }

  // 模拟电梯移动
  elevator.is_available = false;
  elevator.current_floor = target_floor;

  RCLCPP_INFO(this->get_logger(), "电梯 %s 已到达 %d 楼", elevator_id.c_str(), target_floor);
  return true;
}

// ============================================================
// loadFloorMap - 加载楼层地图
// ============================================================
bool FloorManager::loadFloorMap(int floor_number, const std::string & yaml_path)
{
  // 简化实现：记录地图路径
  FloorInfo floor;
  floor.floor_number = floor_number;
  floor.map_yaml_path = yaml_path;

  std::lock_guard<std::mutex> lock(floors_mutex_);
  floors_[floor_number] = floor;

  // 创建地图发布器
  map_pubs_[floor_number] = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/floor_" + std::to_string(floor_number) + "/map", rclcpp::QoS(1).transient_local());

  RCLCPP_INFO(this->get_logger(), "已加载 %d 楼地图: %s", floor_number, yaml_path.c_str());
  return true;
}

// ============================================================
// updateElevatorStatus - 更新电梯状态
// ============================================================
void FloorManager::updateElevatorStatus()
{
  // 简化实现：模拟电梯自动可用
  std::lock_guard<std::mutex> lock(elevators_mutex_);
  for (auto & [id, elevator] : elevators_) {
    if (!elevator.is_available) {
      // 模拟电梯到达后自动可用
      elevator.is_available = true;
    }
  }
}

}  // namespace agv_navigation
