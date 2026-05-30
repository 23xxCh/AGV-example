/**
 * ============================================================
 * navigation_coordinator.cpp - 导航协调器实现（占位）
 * ============================================================
 *
 * 【当前状态】
 * 本文件是导航协调器的基础框架，实现了Action服务器的回调接口
 * 后续将逐步添加：
 * - 调用全局规划器服务获取路径
 * - 调用局部避障器执行路径
 * - 导航进度反馈
 * - 多车调度协调
 *
 * 【Action服务器工作流程】
 *
 *   客户端发送Goal → handleGoal()  → 接受/拒绝
 *                                  ↓ 接受
 *                    handleAccepted() → 启动execute()
 *                                  ↓
 *                    execute() → 循环执行导航
 *                       │
 *                       ├── 定期发布Feedback
 *                       ├── 检查是否被取消
 *                       └── 完成时发布Result
 */

#include "navigation_coordinator.hpp"
#include <memory>
#include <thread>

namespace agv_navigation
{

NavigationCoordinator::NavigationCoordinator(const rclcpp::NodeOptions & options)
: Node("navigation_coordinator", options)
{
  // ----------------------------------------------------------
  // 创建Action服务器
  // ----------------------------------------------------------
  // Action服务器需要提供4个回调：
  // 1. handleGoal:      收到新的导航目标时调用，决定接受还是拒绝
  // 2. handleCancel:    客户端请求取消导航时调用，决定是否同意取消
  // 3. handleAccepted:  目标被接受后调用，启动执行线程
  // 4. execute:         实际的导航执行逻辑（在独立线程中运行）

  navigate_action_server_ = rclcpp_action::create_server<NavigateAction>(
    this,
    "navigate",
    std::bind(&NavigationCoordinator::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&NavigationCoordinator::handleCancel, this, std::placeholders::_1),
    std::bind(&NavigationCoordinator::handleAccepted, this, std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(), "导航协调器已启动（基础框架）");
}

// ============================================================
// handleGoal - 处理新的导航目标
// ============================================================
rclcpp_action::GoalResponse NavigationCoordinator::handleGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const NavigateAction::Goal> goal)
{
  (void)uuid;  // 暂时不使用UUID

  RCLCPP_INFO(this->get_logger(),
    "收到导航请求: 目标(%.2f, %.2f), 最大规划时间=%.1fs",
    goal->goal_position.x, goal->goal_position.y, goal->max_planning_time);

  // TODO: 检查目标是否可达、是否有冲突等
  // 目前简单接受所有请求
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// ============================================================
// handleCancel - 处理取消请求
// ============================================================
rclcpp_action::CancelResponse NavigationCoordinator::handleCancel(
  const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "收到取消导航请求");

  // TODO: 停止AGV运动、清理状态
  // 目前简单同意取消
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

// ============================================================
// handleAccepted - 目标被接受后启动执行
// ============================================================
void NavigationCoordinator::handleAccepted(
  const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
  // 在新线程中执行导航，避免阻塞Action服务器的回调线程
  // Action的execute函数通常需要长时间运行（导航过程）
  // 如果在回调线程中运行，会阻塞其他回调的处理
  std::thread{std::bind(&NavigationCoordinator::execute, this, std::placeholders::_1), goal_handle}.detach();
}

// ============================================================
// execute - 执行导航（在独立线程中运行）
// ============================================================
void NavigationCoordinator::execute(
  const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "开始执行导航任务...");

  // TODO: 实际的导航执行逻辑
  // 1. 调用全局规划器获取路径
  // 2. 将路径发送给局部避障器
  // 3. 循环：检查进度 → 发布反馈 → 检查取消
  // 4. 到达目标或失败 → 发布结果

  auto result = std::make_shared<NavigateAction::Result>();
  result->success = false;
  result->error_msg = "导航协调器尚未实现完整逻辑";
  goal_handle->abort(result);

  RCLCPP_WARN(this->get_logger(), "导航协调器: 执行逻辑待实现");
}

}  // namespace agv_navigation
