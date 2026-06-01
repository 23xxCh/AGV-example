/**
 * ============================================================
 * task_scheduler.cpp - 任务调度器实现（优先级队列版）
 * ============================================================
 *
 * 【调度流程】
 * 1. 收到任务 → 计算有效优先级 → 入优先级队列
 * 2. 定时器触发 → 更新老化 → 尝试分配/抢占
 * 3. 找到最近空闲AGV → 发送导航目标
 * 4. AGV完成任务 → 标记为空闲，触发新的分配
 *
 * 【优先级机制】
 * - 原始优先级：1-10（用户指定，10最高）
 * - 有效优先级 = 原始优先级 + 老化加成
 * - 老化加成 = min(wait_time * aging_rate, max_aging_bonus)
 * - 抢占条件：新任务优先级 - 当前任务优先级 >= preemption_threshold
 */

#include "agv_scheduler/task_scheduler.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>

namespace agv_scheduler
{

TaskScheduler::TaskScheduler(const rclcpp::NodeOptions & options)
: Node("task_scheduler", options),
  task_counter_(0),
  total_completed_(0),
  total_failed_(0)
{
  // 声明参数
  this->declare_parameter("agv_ids", std::vector<std::string>{"agv_001", "agv_002"});
  this->declare_parameter("assignment_interval", 1.0);
  this->declare_parameter("task_timeout", 300.0);
  this->declare_parameter("aging_rate", 0.1);         // 每秒增加0.1优先级
  this->declare_parameter("max_aging_bonus", 5.0);    // 最大老化加成5.0
  this->declare_parameter("enable_preemption", true);
  this->declare_parameter("preemption_threshold", 3);  // 优先级差>=3才抢占

  agv_ids_ = this->get_parameter("agv_ids").as_string_array();
  assignment_interval_ = this->get_parameter("assignment_interval").as_double();
  task_timeout_ = this->get_parameter("task_timeout").as_double();
  aging_rate_ = this->get_parameter("aging_rate").as_double();
  max_aging_bonus_ = this->get_parameter("max_aging_bonus").as_double();
  enable_preemption_ = this->get_parameter("enable_preemption").as_bool();
  preemption_threshold_ = static_cast<uint8_t>(
    this->get_parameter("preemption_threshold").as_int());

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

  // 创建车队状态发布器
  fleet_status_pub_ = this->create_publisher<agv_interfaces::msg::FleetStatus>(
    "fleet_status", rclcpp::QoS(5).reliable());

  // 创建定时分配定时器
  auto period = std::chrono::duration<double>(assignment_interval_);
  assignment_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&TaskScheduler::assignmentTimerCallback, this));

  // 车队状态发布定时器（2Hz）
  fleet_status_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&TaskScheduler::publishFleetStatus, this));

  // 创建TF监听器
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 创建定时更新AGV位置的定时器
  position_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&TaskScheduler::updateAGVPositions, this));

  RCLCPP_INFO(this->get_logger(),
    "任务调度器已启动: %zu台AGV, 分配间隔=%.1fs, 老化速率=%.2f/s, 抢占=%s",
    agv_ids_.size(), assignment_interval_, aging_rate_,
    enable_preemption_ ? "启用" : "禁用");
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
  task.priority = std::max(static_cast<uint8_t>(1),
                           std::min(static_cast<uint8_t>(10), request->task.priority));
  task.effective_priority = static_cast<double>(task.priority);
  task.timeout = request->task.timeout > 0 ? request->task.timeout : task_timeout_;
  task.submit_time = this->now().seconds();
  task.assigned_agv_id = "";

  RCLCPP_INFO(this->get_logger(),
    "收到任务 %s: 目标=(%.1f, %.1f), 优先级=%d",
    task.task_id.c_str(), task.goal_x, task.goal_y, task.priority);

  // 尝试立即分配
  std::string agv_id = findNearestIdleAGV(task.goal_x, task.goal_y);

  if (!agv_id.empty()) {
    // 找到空闲AGV，立即分配
    task.assigned_agv_id = agv_id;
    if (sendGoalToAGV(agv_id, task)) {
      std::lock_guard<std::mutex> lock(tasks_mutex_);
      active_tasks_.push_back(task);

      response->success = true;
      response->assigned_agv_id = agv_id;
      RCLCPP_INFO(this->get_logger(),
        "任务 %s 已分配给 %s", task.task_id.c_str(), agv_id.c_str());
    } else {
      response->success = false;
      response->error_msg = "发送目标失败: " + agv_id;
      RCLCPP_ERROR(this->get_logger(),
        "任务 %s 发送失败", task.task_id.c_str());
    }
  } else if (enable_preemption_ && task.priority > preemption_threshold_) {
    // 尝试抢占低优先级任务
    bool preempted = tryPreempt(task);
    if (preempted) {
      response->success = true;
      response->assigned_agv_id = task.assigned_agv_id;
    } else {
      // 抢占失败，加入队列
      std::lock_guard<std::mutex> lock(tasks_mutex_);
      pending_queue_.push(task);
      response->success = true;
      response->assigned_agv_id = "";
      RCLCPP_INFO(this->get_logger(),
        "任务 %s 已加入队列（优先级=%d）", task.task_id.c_str(), task.priority);
    }
  } else {
    // 加入优先级队列
    std::lock_guard<std::mutex> lock(tasks_mutex_);
    pending_queue_.push(task);

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
  updateAging();
  tryAssignTasks();
}

// ============================================================
// updateAging - 更新优先级老化
// ============================================================
void TaskScheduler::updateAging()
{
  std::lock_guard<std::mutex> lock(tasks_mutex_);

  double now = this->now().seconds();

  // 将队列中的任务取出，更新优先级后重新入队
  std::vector<Task> tasks;
  while (!pending_queue_.empty()) {
    tasks.push_back(pending_queue_.top());
    pending_queue_.pop();
  }

  for (auto & task : tasks) {
    double wait_time = now - task.submit_time;
    double aging_bonus = std::min(wait_time * aging_rate_, max_aging_bonus_);
    task.effective_priority = static_cast<double>(task.priority) + aging_bonus;
    pending_queue_.push(task);
  }
}

// ============================================================
// tryAssignTasks - 尝试分配待处理的任务
// ============================================================
void TaskScheduler::tryAssignTasks()
{
  std::lock_guard<std::mutex> lock(tasks_mutex_);

  // 临时存储无法分配的任务
  std::vector<Task> unassigned;

  while (!pending_queue_.empty()) {
    Task task = pending_queue_.top();
    pending_queue_.pop();

    std::string agv_id = findNearestIdleAGV(task.goal_x, task.goal_y);

    if (!agv_id.empty()) {
      // 找到空闲AGV，分配任务
      task.assigned_agv_id = agv_id;
      if (sendGoalToAGV(agv_id, task)) {
        active_tasks_.push_back(task);
        RCLCPP_INFO(this->get_logger(),
          "任务 %s 已分配给 %s (有效优先级=%.1f)",
          task.task_id.c_str(), agv_id.c_str(), task.effective_priority);
      } else {
        // 发送失败，放回队列重试
        unassigned.push_back(task);
        RCLCPP_WARN(this->get_logger(),
          "任务 %s 发送失败，放回队列", task.task_id.c_str());
      }
    } else {
      unassigned.push_back(task);
    }
  }

  // 将未分配的任务放回队列
  for (const auto & task : unassigned) {
    pending_queue_.push(task);
  }
}

// ============================================================
// tryPreempt - 尝试抢占低优先级任务的AGV
// ============================================================
bool TaskScheduler::tryPreempt(const Task & task)
{
  std::lock_guard<std::mutex> lock(tasks_mutex_);

  // 遍历活跃任务，找优先级最低的
  double min_priority = std::numeric_limits<double>::max();
  size_t min_idx = 0;

  for (size_t i = 0; i < active_tasks_.size(); ++i) {
    if (active_tasks_[i].effective_priority < min_priority) {
      min_priority = active_tasks_[i].effective_priority;
      min_idx = i;
    }
  }

  // 检查是否满足抢占条件
  if (!active_tasks_.empty() &&
      task.priority - min_priority >= preemption_threshold_) {
    Task preempted_task = active_tasks_[min_idx];
    std::string agv_id = preempted_task.assigned_agv_id;

    RCLCPP_WARN(this->get_logger(),
      "任务 %s (优先级=%d) 抢占 %s 的任务 %s (优先级=%.1f)",
      task.task_id.c_str(), task.priority,
      agv_id.c_str(), preempted_task.task_id.c_str(),
      preempted_task.effective_priority);

    // 取消当前任务
    cancelAGVTask(agv_id);

    // 被抢占的任务重新入队（保持原优先级）
    preempted_task.assigned_agv_id = "";
    pending_queue_.push(preempted_task);

    // 从活跃列表移除
    active_tasks_.erase(active_tasks_.begin() + min_idx);

    // 分配新任务
    Task new_task = task;
    new_task.assigned_agv_id = agv_id;
    if (sendGoalToAGV(agv_id, new_task)) {
      active_tasks_.push_back(new_task);
    } else {
      // 发送失败，将被抢占的任务恢复
      preempted_task.assigned_agv_id = agv_id;
      pending_queue_.push(preempted_task);
      RCLCPP_ERROR(this->get_logger(), "抢占后发送新任务失败");
    }

    return true;
  }

  return false;
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
// sendGoalToAGV - 发送导航目标给AGV（返回是否成功发送）
// ============================================================
bool TaskScheduler::sendGoalToAGV(const std::string & agv_id, const Task & task)
{
  auto client_it = action_clients_.find(agv_id);
  if (client_it == action_clients_.end()) {
    RCLCPP_ERROR(this->get_logger(), "找不到AGV的Action客户端: %s", agv_id.c_str());
    return false;
  }

  auto client = client_it->second;

  // 等待Action服务器可用
  if (!client->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_WARN(this->get_logger(),
      "AGV %s 的Action服务器不可用", agv_id.c_str());
    return false;
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

  send_goal_options.goal_response_callback =
    [this, agv_id](const GoalHandleNavigate::SharedPtr & goal_handle) {
      if (goal_handle) {
        std::lock_guard<std::mutex> lock(goal_handles_mutex_);
        goal_handles_[agv_id] = goal_handle;
        RCLCPP_DEBUG(this->get_logger(), "AGV %s 目标已接受", agv_id.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "AGV %s 目标被拒绝", agv_id.c_str());
      }
    };

  send_goal_options.result_callback =
    [this, agv_id, task_id = task.task_id](const GoalHandleNavigate::WrappedResult & result) {
      {
        std::lock_guard<std::mutex> lock(goal_handles_mutex_);
        goal_handles_.erase(agv_id);
      }
      {
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
      }

      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        ++total_completed_;
        RCLCPP_INFO(this->get_logger(),
          "任务 %s 完成 (AGV: %s)", task_id.c_str(), agv_id.c_str());
      } else {
        ++total_failed_;
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

  return true;
}

// ============================================================
// cancelAGVTask - 取消AGV当前任务
// ============================================================
void TaskScheduler::cancelAGVTask(const std::string & agv_id)
{
  std::lock_guard<std::mutex> lock(goal_handles_mutex_);

  auto it = goal_handles_.find(agv_id);
  if (it == goal_handles_.end() || !it->second) {
    RCLCPP_WARN(this->get_logger(), "AGV %s 没有可取消的目标", agv_id.c_str());
    return;
  }

  auto goal_handle = it->second;
  RCLCPP_INFO(this->get_logger(), "取消 AGV %s 的当前任务", agv_id.c_str());

  // 通过Action客户端异步取消目标
  auto client_it = action_clients_.find(agv_id);
  if (client_it != action_clients_.end()) {
    client_it->second->async_cancel_goal(goal_handle);
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

// ============================================================
// publishFleetStatus - 发布车队状态
// ============================================================
void TaskScheduler::publishFleetStatus()
{
  agv_interfaces::msg::FleetStatus fleet_msg;
  fleet_msg.header.stamp = this->now();
  fleet_msg.header.frame_id = "map";

  {
    std::lock_guard<std::mutex> lock(agv_mutex_);
    for (const auto & info : agv_infos_) {
      agv_interfaces::msg::AGVStatus status_msg;
      status_msg.agv_id = info.agv_id;
      status_msg.pose.pose.pose.position.x = info.x;
      status_msg.pose.pose.pose.position.y = info.y;
      status_msg.status = info.status;
      status_msg.current_task_id = info.current_task_id;
      fleet_msg.agv_statuses.push_back(status_msg);
    }
  }

  {
    std::lock_guard<std::mutex> lock(tasks_mutex_);
    fleet_msg.active_task_count = static_cast<int32_t>(active_tasks_.size());
    fleet_msg.pending_task_count = static_cast<int32_t>(pending_queue_.size());
  }

  fleet_status_pub_->publish(fleet_msg);
}

}  // namespace agv_scheduler
