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
#include <algorithm>

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
  this->declare_parameter("goal_tolerance_xy", 0.4);
  this->declare_parameter("goal_tolerance_yaw", 0.2);
  this->declare_parameter("base_frame", std::string("base_link"));
  this->declare_parameter("map_frame", std::string("map"));

  planner_service_ = this->get_parameter("planner_service").as_string();
  feedback_rate_ = this->get_parameter("feedback_rate").as_double();
  goal_tolerance_xy_ = this->get_parameter("goal_tolerance_xy").as_double();
  goal_tolerance_yaw_ = this->get_parameter("goal_tolerance_yaw").as_double();
  base_frame_ = this->get_parameter("base_frame").as_string();
  map_frame_ = this->get_parameter("map_frame").as_string();

  // 交通管理器参数
  this->declare_parameter("agv_id", std::string("agv_001"));
  this->declare_parameter("avg_speed", 0.3);
  this->declare_parameter("max_reservation_retries", 10);
  this->declare_parameter("reservation_retry_interval", 2.0);

  agv_id_ = this->get_parameter("agv_id").as_string();
  avg_speed_ = this->get_parameter("avg_speed").as_double();
  max_reservation_retries_ = this->get_parameter("max_reservation_retries").as_int();
  reservation_retry_interval_ = this->get_parameter("reservation_retry_interval").as_double();

  // 路径重规划参数
  this->declare_parameter("stuck_timeout", 10.0);
  this->declare_parameter("stuck_distance", 0.1);
  this->declare_parameter("max_replans", 3);

  stuck_timeout_ = this->get_parameter("stuck_timeout").as_double();
  stuck_distance_ = this->get_parameter("stuck_distance").as_double();
  max_replans_ = this->get_parameter("max_replans").as_int();

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
  // 创建交通管理器服务客户端
  // ----------------------------------------------------------
  reserve_path_client_ = this->create_client<agv_interfaces::srv::ReservePath>("/reserve_path");
  release_path_client_ = this->create_client<agv_interfaces::srv::ReservePath>("/release_path");

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

  RCLCPP_INFO(this->get_logger(), "导航协调器已启动: AGV=%s, 规划服务=%s",
    agv_id_.c_str(), planner_service_.c_str());
}

// ============================================================
// ~NavigationCoordinator - 析构函数，安全关闭所有执行线程
// ============================================================
NavigationCoordinator::~NavigationCoordinator()
{
  // 设置关闭标志，通知所有执行线程退出
  shutting_down_ = true;

  // 等待所有活跃线程完成，避免use-after-free
  for (auto & t : active_threads_) {
    if (t.joinable()) {
      t.join();
    }
  }
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
  // 清理已完成的线程，防止vector无限增长
  active_threads_.erase(
    std::remove_if(active_threads_.begin(), active_threads_.end(),
      [](const std::thread & t) { return !t.joinable(); }),
    active_threads_.end());

  // 将线程存储到成员变量中，确保节点析构时能安全join
  active_threads_.emplace_back(
    std::bind(&NavigationCoordinator::execute, this, std::placeholders::_1), goal_handle);
}

// ============================================================
// reservePath - 向交通管理器预约路径
// ============================================================
bool NavigationCoordinator::reservePath(const nav_msgs::msg::Path & path)
{
  for (int retry = 0; retry < max_reservation_retries_; ++retry) {
    if (!reserve_path_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_WARN(this->get_logger(), "交通管理器服务不可用，跳过预约");
      return true;  // 服务不可用时直接放行
    }

    auto req = std::make_shared<agv_interfaces::srv::ReservePath::Request>();
    req->agv_id = agv_id_;
    req->path = path;
    req->start_time = this->now().seconds();
    req->speed = avg_speed_;

    auto future = reserve_path_client_->async_send_request(req);
    auto status = future.wait_for(std::chrono::seconds(3));

    if (status != std::future_status::ready) {
      RCLCPP_WARN(this->get_logger(), "预约请求超时，重试 %d/%d", retry + 1, max_reservation_retries_);
      std::this_thread::sleep_for(std::chrono::duration<double>(reservation_retry_interval_));
      continue;
    }

    auto resp = future.get();
    if (resp->success) {
      RCLCPP_INFO(this->get_logger(), "路径预约成功: %s", agv_id_.c_str());
      return true;
    }

    // 有冲突，等待建议时间后重试
    double wait_time = std::max(resp->wait_time, reservation_retry_interval_);
    RCLCPP_INFO(this->get_logger(),
      "路径冲突（与%s），等待%.1fs后重试 %d/%d",
      resp->conflict_agv_id.c_str(), wait_time, retry + 1, max_reservation_retries_);
    std::this_thread::sleep_for(std::chrono::duration<double>(wait_time));
  }

  return false;
}

// ============================================================
// releasePath - 释放路径预约
// ============================================================
void NavigationCoordinator::releasePath()
{
  if (!release_path_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "交通管理器不可用，跳过释放");
    return;
  }
  auto req = std::make_shared<agv_interfaces::srv::ReservePath::Request>();
  req->agv_id = agv_id_;
  release_path_client_->async_send_request(req);
  RCLCPP_INFO(this->get_logger(), "路径已释放: %s", agv_id_.c_str());
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

  // 检查节点是否正在关闭
  if (shutting_down_) {
    result->success = false;
    result->error_msg = "节点正在关闭，取消导航";
    goal_handle->abort(result);
    return;
  }

  // ----------------------------------------------------------
  // 步骤1：调用全局规划器获取路径
  // ----------------------------------------------------------
  auto planPath = [this, &goal]() -> nav_msgs::msg::Path {
    nav_msgs::msg::Path empty_path;

    if (!path_plan_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "全局规划服务不可用: %s", planner_service_.c_str());
      return empty_path;
    }

    auto req = std::make_shared<agv_interfaces::srv::PathPlan::Request>();
    req->goal = goal->goal_position;
    req->use_current_pose = goal->use_current_pose;
    req->planner_id = "astar";

    // 从TF查询当前机器人位置作为起点
    try {
      auto tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      req->start.x = tf.transform.translation.x;
      req->start.y = tf.transform.translation.y;
      req->start.z = 0.0;
    } catch (const tf2::TransformException &) {
      req->start.x = 0.15;
      req->start.y = 0.15;
      req->start.z = 0.0;
      RCLCPP_WARN(this->get_logger(), "TF查询失败，使用默认起点(0.15, 0.15)");
    }

    auto future = path_plan_client_->async_send_request(req);
    auto status = future.wait_for(std::chrono::duration<double>(goal->max_planning_time));

    if (status != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "全局规划超时");
      return empty_path;
    }

    auto resp = future.get();
    if (!resp->success) {
      RCLCPP_ERROR(this->get_logger(), "全局规划失败: %s", resp->error_msg.c_str());
      return empty_path;
    }

    RCLCPP_INFO(this->get_logger(), "全局规划成功，路径点数: %zu", resp->path.poses.size());
    return resp->path;
  };

  // 执行首次规划
  nav_msgs::msg::Path current_path = planPath();
  if (current_path.poses.empty()) {
    result->success = false;
    result->error_msg = "全局规划失败";
    goal_handle->abort(result);
    return;
  }

  // ----------------------------------------------------------
  // 步骤2：向交通管理器预约路径
  // ----------------------------------------------------------
  if (!reservePath(current_path)) {
    result->success = false;
    result->error_msg = "路径预约失败：超过最大重试次数";
    goal_handle->abort(result);
    RCLCPP_ERROR(this->get_logger(), "%s", result->error_msg.c_str());
    return;
  }

  // ----------------------------------------------------------
  // 步骤3：发布路径给DWA局部避障器
  // ----------------------------------------------------------
  path_pub_->publish(current_path);

  // ----------------------------------------------------------
  // 步骤4：监控循环（含路径重规划）
  // ----------------------------------------------------------
  auto start_time = this->now();
  auto feedback_period = std::chrono::duration<double>(1.0 / feedback_rate_);

  double goal_x = goal->goal_position.x;
  double goal_y = goal->goal_position.y;

  // 卡住检测变量
  double last_progress_distance = std::numeric_limits<double>::infinity();
  auto last_progress_time = this->now();
  int replan_count = 0;

  while (rclcpp::ok()) {
    // 检查节点是否正在关闭（避免析构后访问已销毁的成员）
    if (shutting_down_) {
      releasePath();
      result->success = false;
      result->error_msg = "节点关闭，中断导航";
      result->total_time = (this->now() - start_time).seconds();
      goal_handle->abort(result);
      RCLCPP_WARN(this->get_logger(), "节点关闭，导航中断");
      return;
    }

    // 检查是否被取消
    if (goal_handle->is_canceling()) {
      releasePath();
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
    double elapsed = (this->now() - start_time).seconds();
    double completion = std::min(100.0, std::max(0.0, (1.0 - distance / 10.0) * 100.0));

    // 发布反馈
    feedback->current_position.x = robot_x;
    feedback->current_position.y = robot_y;
    feedback->current_position.z = 0.0;
    feedback->distance_remaining = distance;
    feedback->estimated_time_remaining = distance / avg_speed_;
    feedback->completion_percent = static_cast<float>(completion);
    goal_handle->publish_feedback(feedback);

    // 检查是否到达
    if (distance < goal_tolerance_xy_) {
      releasePath();
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
      releasePath();
      result->success = false;
      result->error_msg = "导航超时（300秒）";
      result->total_time = elapsed;
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), "%s", result->error_msg.c_str());
      return;
    }

    // ----------------------------------------------------------
    // 卡住检测与路径重规划
    // ----------------------------------------------------------
    // 检测逻辑：如果距离目标的距离在stuck_timeout_秒内没有明显减小（<stuck_distance_），
    // 则认为AGV被障碍物挡住，触发路径重规划
    if (distance < last_progress_distance - stuck_distance_) {
      // 有进展，更新记录
      last_progress_distance = distance;
      last_progress_time = this->now();
    } else {
      // 检查是否卡住超时
      double stuck_duration = (this->now() - last_progress_time).seconds();
      if (stuck_duration > stuck_timeout_ && replan_count < max_replans_) {
        replan_count++;
        RCLCPP_WARN(this->get_logger(),
          "AGV卡住%.1fs未接近目标（距离=%.2fm），触发路径重规划 %d/%d",
          stuck_duration, distance, replan_count, max_replans_);

        // 释放旧路径预约
        releasePath();

        // 重新规划路径
        nav_msgs::msg::Path new_path = planPath();
        if (new_path.poses.empty()) {
          RCLCPP_ERROR(this->get_logger(), "重规划失败，继续使用原路径");
          // 重新预约原路径
          reservePath(current_path);
        } else {
          // 预约新路径
          if (reservePath(new_path)) {
            // 使用新路径
            current_path = new_path;
            path_pub_->publish(current_path);
            RCLCPP_INFO(this->get_logger(), "重规划成功，新路径点数: %zu", current_path.poses.size());
          } else {
            RCLCPP_WARN(this->get_logger(), "新路径预约失败，继续使用原路径");
            reservePath(current_path);
          }
        }

        // 重置卡住检测
        last_progress_distance = distance;
        last_progress_time = this->now();
      }
    }

    std::this_thread::sleep_for(feedback_period);
  }
}

}  // namespace agv_navigation
