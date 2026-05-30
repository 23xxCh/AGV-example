/**
 * ============================================================
 * navigation_coordinator.hpp - 导航协调器头文件（占位）
 * ============================================================
 *
 * 【模块概述】
 * 导航协调器是AGV导航系统的"大脑"，负责：
 * 1. 接收导航动作请求（Navigate Action）
 * 2. 调用全局规划器获取路径
 * 3. 将路径发送给局部避障器执行
 * 4. 持续反馈导航进度
 * 5. 处理导航取消请求
 *
 * 【当前状态】框架占位，后续Phase实现
 */

#ifndef AGV_NAVIGATION__NAVIGATION_COORDINATOR_HPP_
#define AGV_NAVIGATION__NAVIGATION_COORDINATOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "agv_interfaces/action/navigate.hpp"

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
};

}  // namespace agv_navigation

#endif  // AGV_NAVIGATION__NAVIGATION_COORDINATOR_HPP_
