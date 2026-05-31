/**
 * ============================================================
 * floor_manager.hpp - 多楼层管理器
 * ============================================================
 *
 * 【功能】
 * 管理多楼层仓库的导航：
 * 1. 维护每层楼的地图和代价地图
 * 2. 管理电梯/过渡区域
 * 3. 处理跨楼层任务
 * 4. 协调AGV在楼层间的移动
 *
 * 【楼层切换流程】
 * 1. AGV到达电梯入口点
 * 2. 请求楼层切换
 * 3. 等待电梯可用
 * 4. 切换地图和坐标系
 * 5. AGV在新楼层继续导航
 */

#ifndef AGV_NAVIGATION__FLOOR_MANAGER_HPP_
#define AGV_NAVIGATION__FLOOR_MANAGER_HPP_

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "agv_interfaces/srv/assign_task.hpp"

namespace agv_navigation
{

// 电梯信息
struct Elevator
{
  std::string elevator_id;
  int current_floor;
  bool is_available;
  std::vector<int> accessible_floors;
  geometry_msgs::msg::Pose entry_pose;    // 电梯入口位置
  geometry_msgs::msg::Pose exit_pose;     // 电梯出口位置
};

// 楼层信息
struct FloorInfo
{
  int floor_number;
  std::string map_yaml_path;
  nav_msgs::msg::OccupancyGrid costmap;
  std::vector<geometry_msgs::msg::Pose> elevator_entries;
  std::vector<geometry_msgs::msg::Pose> elevator_exits;
};

// 跨楼层任务
struct CrossFloorTask
{
  std::string task_id;
  std::string agv_id;
  int source_floor;
  int target_floor;
  geometry_msgs::msg::Pose goal_pose;
  std::string status;  // "pending", "navigating_to_elevator", "waiting_elevator", "in_elevator", "completed"
};

class FloorManager : public rclcpp::Node
{
public:
  explicit FloorManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // 处理跨楼层任务请求
  void handleCrossFloorTask(
    const std::shared_ptr<agv_interfaces::srv::AssignTask::Request> request,
    std::shared_ptr<agv_interfaces::srv::AssignTask::Response> response);

  // 切换楼层
  bool switchFloor(const std::string & agv_id, int target_floor);

  // 请求电梯
  bool requestElevator(const std::string & elevator_id, int target_floor);

  // 加载楼层地图
  bool loadFloorMap(int floor_number, const std::string & yaml_path);

  // 定时更新电梯状态
  void updateElevatorStatus();

  // 参数
  std::vector<std::string> agv_ids_;
  std::string map_dir_;

  // 楼层信息
  std::map<int, FloorInfo> floors_;
  std::mutex floors_mutex_;

  // 电梯信息
  std::map<std::string, Elevator> elevators_;
  std::mutex elevators_mutex_;

  // 跨楼层任务
  std::vector<CrossFloorTask> cross_floor_tasks_;
  std::mutex tasks_mutex_;

  // 当前每台AGV所在楼层
  std::map<std::string, int> agv_floor_;

  // 地图发布器（每层楼一个）
  std::map<int, rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr> map_pubs_;

  // 服务
  rclcpp::Service<agv_interfaces::srv::AssignTask>::SharedPtr cross_floor_service_;

  // 定时器
  rclcpp::TimerBase::SharedPtr elevator_timer_;
};

}  // namespace agv_navigation

#endif  // AGV_NAVIGATION__FLOOR_MANAGER_HPP_
