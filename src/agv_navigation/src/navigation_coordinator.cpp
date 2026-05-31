/**
 * ============================================================
 * navigation_coordinator.cpp - 导航协调器实现
 * ============================================================
 *
 * 【完整执行流程】
 * 1. 收到Navigate Action请求（目标点）
 * 2. 调用A*全局规划器服务获取路径
 * 3. 将路径发布给DWA局部避障器
 * 4. 循环监控：查询机器人位姿，计算进度，发布反馈
 * 5. 到达目标/失败/取消 → 发布结果
 */

#include "navigation_coordinator.hpp"
#include <memory>
#include <thread>
#include <chrono>

namespace agv_navigation
{

NavigationCoordinator::NavigationCoordinator(const rclcpp::NodeOptions & options)
: Node("navigation_coordinator", options)
{
  // ----------------------------------------------------------
  // 声明和读取参数
  // ----------------------------------------------------------
  this->declare_parameter("planner_service", std::string("plan_path"));
  this->declare_parameter("feedback_rate", 5.0);
  this->declare_parameter("goal_tolerance_xy", 0.3);
  this->declare_parameter("goal_tolerance_yaw", 0.2);
  this->declare_parameter("base_frame", std::string("base_link"));
  this->declare_parameter("map_frame", std::string("map"));

  planner_service_ = this->get_parameter("planner_service").as_string();
  feedback_rate_ = this->get_parameter("feedback_rate").as_double();
  goal_tolerance_xy_ = this->get_parameter("goal_tolerance_xy").as_double();
  goal_tolerance_yaw_ = this->get_parameter("goal_tolerance_yaw").as_double();
  base_frame_ = this->get_parameter("base_frame").as_string();
  map_frame_ = this->get_parameter("map_frame").as_string();

  // ----------------------------------------------------------
  // 创建Action服务器
  // ----------------------------------------------------------
  navigate_action_server_ = rclcpp_action::create_server<NavigateAction>(
    this,
    "navigate",
    std::bind(&NavigationCoordinator::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&NavigationCoordinator::handleCancel, this, std::placeholders::_1),
    std::bind(&NavigationCoordinator::handleAccepted, this, std::placeholders::_1)
  );

  // ----------------------------------------------------------
  // 创建全局规划器服务客户端
  // ----------------------------------------------------------
  path_plan_client_ = this->create_client<agv_interfaces::srv::PathPlan>(planner_service_);

  // ----------------------------------------------------------
  // 创建路径发布器
  // ----------------------------------------------------------
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "planned_path", rclcpp::QoS(10).reliable());

  // ----------------------------------------------------------
  // 创建TF监听器
  // ----------------------------------------------------------
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(this->get_logger(), "导航协调器已启动，全局规划服务: %s", planner_service_.c_str());
}

// ============================================================
// handleGoal - 处理新的导航目标
// ============================================================
rclcpp_action::GoalResponse NavigationCoordinator::handleGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const NavigateAction::Goal> goal)
{
  (void)uuid;

  RCLCPP_INFO(this->get_logger(),
    "收到导航请求: 目标(%.2f, %.2f, %.2f), 最大规划时间=%.1fs",
    goal->goal_position.x, goal->goal_position.y, goal->goal_theta,
    goal->max_planning_time);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// ============================================================
// handleCancel - 处理取消请求
// ============================================================
rclcpp_action::CancelResponse NavigationCoordinator::handleCancel(
  const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "收到取消导航请求");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

// ============================================================
// handleAccepted - 目标被接受后启动执行
// ============================================================
void NavigationCoordinator::handleAccepted(
  const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
  std::thread{std::bind(&NavigationCoordinator::execute, this, std::placeholders::_1), goal_handle}.detach();
}

// ============================================================
// execute - 执行导航（完整流程）
// ============================================================
void NavigationCoordinator::execute(
  const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<NavigateAction::Result>();
  auto feedback = std::make_shared<NavigateAction::Feedback>();

  RCLCPP_INFO(this->get_logger(), "开始执行导航任务...");

  // ----------------------------------------------------------
  // 步骤1：调用全局规划器获取路径
  // ----------------------------------------------------------
  // 等待服务可用
  if (!path_plan_client_->wait_for_service(std::chrono::seconds(5))) {
    result->success = false;
    result->error_msg = "全局规划服务不可用: " + planner_service_;
    goal_handle->abort(result);
    RCLCPP_ERROR(this->get_logger(), "%s", result->error_msg.c_str());
    return;
  }

  // 构建规划请求
  auto plan_request = std::make_shared<agv_interfaces::srv::PathPlan::Request>();
  plan_request->goal = goal->goal_position;
  plan_request->use_current_pose = goal->use_current_pose;
  plan_request->planner_id = "astar";

  // 始终从TF查询当前机器人位置作为起点
  try {
    auto tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    plan_request->start.x = tf.transform.translation.x;
    plan_request->start.y = tf.transform.translation.y;
    plan_request->start.z = 0.0;
  } catch (const tf2::TransformException &) {
    // TF查询失败时使用默认起点
    plan_request->start.x = 0.15;
    plan_request->start.y = 0.15;
    plan_request->start.z = 0.0;
    RCLCPP_WARN(this->get_logger(), "TF查询失败，使用默认起点(0.15, 0.15)");
  }

  // 调用全局规划服务
  auto plan_future = path_plan_client_->async_send_request(plan_request);

  // 等待规划结果（最多等待max_planning_time秒）
  auto status = plan_future.wait_for(
    std::chrono::duration<double>(goal->max_planning_time));

  if (status != std::future_status::ready) {
    result->success = false;
    result->error_msg = "全局规划超时";
    goal_handle->abort(result);
    RCLCPP_ERROR(this->get_logger(), "%s", result->error_msg.c_str());
    return;
  }

  auto plan_response = plan_future.get();
  if (!plan_response->success) {
    result->success = false;
    result->error_msg = "全局规划失败: " + plan_response->error_msg;
    goal_handle->abort(result);
    RCLCPP_ERROR(this->get_logger(), "%s", result->error_msg.c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "全局规划成功，路径点数: %zu，发布给DWA...",
    plan_response->path.poses.size());

  // ----------------------------------------------------------
  // 步骤2：发布路径给DWA局部避障器
  // ----------------------------------------------------------
  path_pub_->publish(plan_response->path);

  // ----------------------------------------------------------
  // 步骤3：监控循环
  // ----------------------------------------------------------
  auto start_time = this->now();
  auto feedback_period = std::chrono::duration<double>(1.0 / feedback_rate_);

  // 目标位姿
  double goal_x = goal->goal_position.x;
  double goal_y = goal->goal_position.y;

  while (rclcpp::ok()) {
    // 检查是否被取消
    if (goal_handle->is_canceling()) {
      // 发布停止指令（通过DWA的停止机制）
      result->success = false;
      result->error_msg = "导航被取消";
      result->total_time = (this->now() - start_time).seconds();
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "导航已取消");
      return;
    }

    // 查询机器人当前位姿
    double robot_x = 0.0, robot_y = 0.0;
    try {
      auto tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      robot_x = tf.transform.translation.x;
      robot_y = tf.transform.translation.y;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TF查询失败: %s", ex.what());
      std::this_thread::sleep_for(feedback_period);
      continue;
    }

    // 计算到目标的距离
    double dx = goal_x - robot_x;
    double dy = goal_y - robot_y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // 计算完成百分比
    // 用初始距离作为100%基准
    double elapsed = (this->now() - start_time).seconds();
    double completion = std::min(100.0, std::max(0.0, (1.0 - distance / 10.0) * 100.0));

    // 发布反馈
    feedback->current_position.x = robot_x;
    feedback->current_position.y = robot_y;
    feedback->current_position.z = 0.0;
    feedback->distance_remaining = distance;
    feedback->estimated_time_remaining = distance / 0.3;  // 粗略估计
    feedback->completion_percent = static_cast<float>(completion);
    goal_handle->publish_feedback(feedback);

    // 检查是否到达
    if (distance < goal_tolerance_xy_) {
      result->success = true;
      result->final_position_error = distance;
      result->total_time = elapsed;
      result->error_msg = "";
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(),
        "导航成功! 到达目标(%.2f,%.2f), 耗时=%.1fs, 误差=%.3fm",
        goal_x, goal_y, elapsed, distance);
      return;
    }

    // 超时检查（默认300秒）
    if (elapsed > 300.0) {
      result->success = false;
      result->error_msg = "导航超时（300秒）";
      result->total_time = elapsed;
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), "%s", result->error_msg.c_str());
      return;
    }

    std::this_thread::sleep_for(feedback_period);
  }
}

}  // namespace agv_navigation
