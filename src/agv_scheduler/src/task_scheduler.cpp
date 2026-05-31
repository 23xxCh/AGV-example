/**
 * ============================================================
 * task_scheduler.cpp - 任务调度器实现
 * ============================================================
 *
 * 【调度流程】
 * 1. 收到任务 → 加入待处理队列（按优先级排序）
 * 2. 定时器触发 → 尝试分配任务
 * 3. 找到最近空闲AGV → 发送导航目标
 * 4. AGV完成任务 → 标记为空闲，触发新的分配
 */

#include "agv_scheduler/task_scheduler.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>

namespace agv_scheduler
{

TaskScheduler::TaskScheduler(const rclcpp::NodeOptions & options)
: Node("task_scheduler", options),
  task_counter_(0)
{
  // 声明参数
  this->declare_parameter("agv_ids", std::vector<std::string>{"agv_001", "agv_002"});
  this->declare_parameter("assignment_interval", 1.0);
  this->declare_parameter("task_timeout", 300.0);

  agv_ids_ = this->get_parameter("agv_ids").as_string_array();
  assignment_interval_ = this->get_parameter("assignment_interval").as_double();
  task_timeout_ = this->get_parameter("task_timeout").as_double();

  // 初始化AGV信息
  for (const auto & agv_id : agv_ids_) {
    AGVInfo info;
    info.agv_id = agv_id;
    info.status = 0;  // 空闲
    info.x = 0.0;
    info.y = 0.0;
    agv_infos_.push_back(info);

    // 创建Action客户端
    action_clients_[agv_id] = rclcpp_action::create_client<NavigateAction>(
      this, agv_id + "/navigate");

    // 创建状态订阅器
    status_subs_.push_back(
      this->create_subscription<agv_interfaces::msg::AGVStatus>(
        agv_id + "/status", rclcpp::QoS(10).reliable(),
        [this, agv_id](const agv_interfaces::msg::AGVStatus::SharedPtr msg) {
          agvStatusCallback(agv_id, msg);
        }));
  }

  // 创建任务分配服务
  assign_service_ = this->create_service<agv_interfaces::srv::AssignTask>(
    "assign_task",
    std::bind(&TaskScheduler::handleAssignTask, this,
              std::placeholders::_1, std::placeholders::_2));

  // 创建定时分配定时器
  auto period = std::chrono::duration<double>(assignment_interval_);
  assignment_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&TaskScheduler::assignmentTimerCallback, this));

  // 创建TF监听器（用于查询AGV位置）
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 创建定时更新AGV位置的定时器（每0.5秒更新一次）
  position_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&TaskScheduler::updateAGVPositions, this));

  RCLCPP_INFO(this->get_logger(),
    "任务调度器已启动: %zu台AGV, 分配间隔=%.1fs",
    agv_ids_.size(), assignment_interval_);
}

// ============================================================
// handleAssignTask - 处理任务分配请求
// ============================================================
void TaskScheduler::handleAssignTask(
  const std::shared_ptr<agv_interfaces::srv::AssignTask::Request> request,
  std::shared_ptr<agv_interfaces::srv::AssignTask::Response> response)
{
  // 构建任务
  Task task;
  task.task_id = request->task.task_id;
  if (task.task_id.empty()) {
    task.task_id = "task_" + std::to_string(++task_counter_);
  }
  task.goal_x = request->task.goal_x;
  task.goal_y = request->task.goal_y;
  task.goal_theta = request->task.goal_theta;
  task.priority = request->task.priority;
  task.timeout = request->task.timeout > 0 ? request->task.timeout : task_timeout_;
  task.submit_time = this->now().seconds();
  task.assigned_agv_id = "";

  // 尝试立即分配
  std::string agv_id = findNearestIdleAGV(task.goal_x, task.goal_y);

  if (!agv_id.empty()) {
    // 找到空闲AGV，立即分配
    task.assigned_agv_id = agv_id;
    sendGoalToAGV(agv_id, task);

    std::lock_guard<std::mutex> lock(tasks_mutex_);
    active_tasks_.push_back(task);

    response->success = true;
    response->assigned_agv_id = agv_id;
    RCLCPP_INFO(this->get_logger(),
      "任务 %s 已分配给 %s", task.task_id.c_str(), agv_id.c_str());
  } else {
    // 没有空闲AGV，加入待处理队列
    std::lock_guard<std::mutex> lock(tasks_mutex_);
    pending_tasks_.push_back(task);

    // 按优先级排序（高优先级在前）
    std::sort(pending_tasks_.begin(), pending_tasks_.end(),
      [](const Task & a, const Task & b) {
        return a.priority > b.priority;
      });

    response->success = true;
    response->assigned_agv_id = "";
    RCLCPP_INFO(this->get_logger(),
      "任务 %s 已加入队列（优先级=%d）", task.task_id.c_str(), task.priority);
  }
}

// ============================================================
// assignmentTimerCallback - 定时尝试分配任务
// ============================================================
void TaskScheduler::assignmentTimerCallback()
{
  tryAssignTasks();
}

// ============================================================
// tryAssignTasks - 尝试分配待处理的任务
// ============================================================
void TaskScheduler::tryAssignTasks()
{
  std::lock_guard<std::mutex> lock(tasks_mutex_);

  auto it = pending_tasks_.begin();
  while (it != pending_tasks_.end()) {
    std::string agv_id = findNearestIdleAGV(it->goal_x, it->goal_y);

    if (!agv_id.empty()) {
      // 找到空闲AGV，分配任务
      Task task = *it;
      task.assigned_agv_id = agv_id;
      sendGoalToAGV(agv_id, task);
      active_tasks_.push_back(task);
      it = pending_tasks_.erase(it);

      RCLCPP_INFO(this->get_logger(),
        "任务 %s 已分配给 %s", task.task_id.c_str(), agv_id.c_str());
    } else {
      ++it;
    }
  }
}

// ============================================================
// findNearestIdleAGV - 找到最近的空闲AGV
// ============================================================
std::string TaskScheduler::findNearestIdleAGV(double goal_x, double goal_y)
{
  std::lock_guard<std::mutex> lock(agv_mutex_);

  double min_dist = std::numeric_limits<double>::infinity();
  std::string nearest_id;

  for (const auto & info : agv_infos_) {
    if (info.status == 0) {  // 空闲
      double dx = info.x - goal_x;
      double dy = info.y - goal_y;
      double dist = std::sqrt(dx * dx + dy * dy);

      if (dist < min_dist) {
        min_dist = dist;
        nearest_id = info.agv_id;
      }
    }
  }

  return nearest_id;
}

// ============================================================
// sendGoalToAGV - 发送导航目标给AGV
// ============================================================
void TaskScheduler::sendGoalToAGV(const std::string & agv_id, const Task & task)
{
  auto client_it = action_clients_.find(agv_id);
  if (client_it == action_clients_.end()) {
    RCLCPP_ERROR(this->get_logger(), "找不到AGV的Action客户端: %s", agv_id.c_str());
    return;
  }

  auto client = client_it->second;

  // 等待Action服务器可用
  if (!client->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_WARN(this->get_logger(),
      "AGV %s 的Action服务器不可用", agv_id.c_str());
    return;
  }

  // 构建目标
  auto goal_msg = NavigateAction::Goal();
  goal_msg.goal_position.x = task.goal_x;
  goal_msg.goal_position.y = task.goal_y;
  goal_msg.goal_position.z = 0.0;
  goal_msg.goal_theta = task.goal_theta;
  goal_msg.use_current_pose = true;
  goal_msg.max_planning_time = 10.0;

  // 发送目标（异步）
  auto send_goal_options = rclcpp_action::Client<NavigateAction>::SendGoalOptions();
  send_goal_options.result_callback =
    [this, agv_id, task_id = task.task_id](const GoalHandleNavigate::WrappedResult & result) {
      // 任务完成回调
      std::lock_guard<std::mutex> lock(tasks_mutex_);
      std::lock_guard<std::mutex> lock2(agv_mutex_);

      // 从活跃任务中移除
      active_tasks_.erase(
        std::remove_if(active_tasks_.begin(), active_tasks_.end(),
          [&](const Task & t) { return t.task_id == task_id; }),
        active_tasks_.end());

      // 标记AGV为空闲
      for (auto & info : agv_infos_) {
        if (info.agv_id == agv_id) {
          info.status = 0;
          info.current_task_id = "";
          break;
        }
      }

      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(),
          "任务 %s 完成 (AGV: %s)", task_id.c_str(), agv_id.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(),
          "任务 %s 失败 (AGV: %s)", task_id.c_str(), agv_id.c_str());
      }
    };

  client->async_send_goal(goal_msg, send_goal_options);

  // 更新AGV状态
  {
    std::lock_guard<std::mutex> lock(agv_mutex_);
    for (auto & info : agv_infos_) {
      if (info.agv_id == agv_id) {
        info.status = 1;  // 执行中
        info.current_task_id = task.task_id;
        break;
      }
    }
  }
}

// ============================================================
// agvStatusCallback - AGV状态回调
// ============================================================
void TaskScheduler::agvStatusCallback(
  const std::string & agv_id,
  const agv_interfaces::msg::AGVStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(agv_mutex_);

  for (auto & info : agv_infos_) {
    if (info.agv_id == agv_id) {
      info.x = msg->pose.pose.pose.position.x;
      info.y = msg->pose.pose.pose.position.y;
      // 只更新位置，不覆盖status（由任务分配逻辑控制）
      break;
    }
  }
}

// ============================================================
// updateAGVPositions - 从TF查询AGV位置
// ============================================================
void TaskScheduler::updateAGVPositions()
{
  std::lock_guard<std::mutex> lock(agv_mutex_);

  for (auto & info : agv_infos_) {
    // 每台AGV的TF frame: {agv_id}_base_link
    std::string child_frame = info.agv_id + "_base_link";

    try {
      auto tf = tf_buffer_->lookupTransform(
        "map", child_frame, tf2::TimePointZero,
        std::chrono::milliseconds(100));

      info.x = tf.transform.translation.x;
      info.y = tf.transform.translation.y;
    } catch (const tf2::TransformException &) {
      // TF查询失败，保持上次的位置
    }
  }
}

}  // namespace agv_scheduler
