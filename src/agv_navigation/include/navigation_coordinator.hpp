/**
 * ============================================================
 * navigation_coordinator.hpp - 导航协调器头文件
 * ============================================================
 *
 * 【模块概述】
 * 导航协调器是AGV导航系统的"大脑"，负责：
 * 1. 接收导航动作请求（Navigate Action）
 * 2. 调用全局规划器获取路径
 * 3. 将路径发布给DWA局部避障器
 * 4. 监控DWA的执行状态，反馈导航进度
 * 5. 处理导航取消请求
 *
 * 【执行流程】
 *
 *   客户端发送Navigate目标
 *        │
 *        ▼
 *   handleGoal() → 接受请求
 *        │
 *        ▼
 *   execute()（独立线程）
 *        │
 *        ├── 1. 调用 /plan_path 服务获取全局路径
 *        ├── 2. 发布路径到 /planned_path（DWA订阅）
 *        ├── 3. 监控循环（feedback_rate Hz）：
 *        │       ├── 查询TF获取机器人位姿
 *        │       ├── 计算到目标距离
 *        │       ├── 检查是否到达（goal_tolerance）
 *        │       ├── 发布Navigate反馈（进度百分比等）
 *        │       └── 检查是否被取消
 *        └── 4. 到达目标/失败/取消 → 发布结果
 */

#ifndef AGV_NAVIGATION__NAVIGATION_COORDINATOR_HPP_
#define AGV_NAVIGATION__NAVIGATION_COORDINATOR_HPP_

#include <string>
#include <memory>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "agv_interfaces/action/navigate.hpp"
#include "agv_interfaces/srv/path_plan.hpp"

namespace agv_navigation
{

class NavigationCoordinator : public rclcpp::Node
{
public:
  using NavigateAction = agv_interfaces::action::Navigate;
  using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<NavigateAction>;

  explicit NavigationCoordinator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Action服务器回调
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateAction::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleNavigate> goal_handle);

  void handleAccepted(const std::shared_ptr<GoalHandleNavigate> goal_handle);

  void execute(const std::shared_ptr<GoalHandleNavigate> goal_handle);

  // Action服务器
  rclcpp_action::Server<NavigateAction>::SharedPtr navigate_action_server_;

  // 全局路径规划服务客户端
  rclcpp::Client<agv_interfaces::srv::PathPlan>::SharedPtr path_plan_client_;

  // 路径发布器（发送给DWA）
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 参数
  std::string planner_service_;
  double feedback_rate_;
  double goal_tolerance_xy_;
  double goal_tolerance_yaw_;
  std::string base_frame_;
  std::string map_frame_;
};

}  // namespace agv_navigation

#endif  // AGV_NAVIGATION__NAVIGATION_COORDINATOR_HPP_
