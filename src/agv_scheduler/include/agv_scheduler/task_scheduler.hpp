/**
 * ============================================================
 * task_scheduler.hpp - 任务调度器节点
 * ============================================================
 *
 * 【功能】
 * 接收导航任务，自动分配给最合适的AGV：
 * 1. 外部系统通过assign_task服务提交任务
 * 2. 调度器根据"最近空闲"原则选择AGV
 * 3. 调用AGV的navigate Action执行任务
 * 4. 监控任务执行状态
 *
 * 【调度算法】
 * 最近空闲优先（Nearest-Idle）：
 * - 找出所有空闲的AGV
 * - 计算每个AGV到目标点的欧氏距离
 * - 选择最近的AGV执行任务
 */

#ifndef AGV_SCHEDULER__TASK_SCHEDULER_HPP_
#define AGV_SCHEDULER__TASK_SCHEDULER_HPP_

#include <string>
#include <vector>
#include <queue>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "agv_interfaces/msg/task_request.hpp"
#include "agv_interfaces/msg/agv_status.hpp"
#include "agv_interfaces/srv/assign_task.hpp"
#include "agv_interfaces/action/navigate.hpp"

namespace agv_scheduler
{

// 任务信息
struct Task {
  std::string task_id;
  double goal_x, goal_y, goal_theta;
  uint8_t priority;
  double timeout;
  double submit_time;
  std::string assigned_agv_id;  // 空=未分配
};

// AGV信息
struct AGVInfo {
  std::string agv_id;
  uint8_t status;  // 0=空闲, 1=执行中, 2=充电, 3=故障
  double x, y;     // 当前位置
  std::string current_task_id;
};

class TaskScheduler : public rclcpp::Node
{
public:
  using NavigateAction = agv_interfaces::action::Navigate;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateAction>;

  explicit TaskScheduler(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // 任务分配服务回调
  void handleAssignTask(
    const std::shared_ptr<agv_interfaces::srv::AssignTask::Request> request,
    std::shared_ptr<agv_interfaces::srv::AssignTask::Response> response);

  // 定时分配任务
  void assignmentTimerCallback();

  // 尝试分配待处理的任务
  void tryAssignTasks();

  // 找到最近的空闲AGV
  std::string findNearestIdleAGV(double goal_x, double goal_y);

  // 发送导航目标给AGV
  void sendGoalToAGV(const std::string & agv_id, const Task & task);

  // AGV状态回调
  void agvStatusCallback(const std::string & agv_id,
                          const agv_interfaces::msg::AGVStatus::SharedPtr msg);

  // 参数
  std::vector<std::string> agv_ids_;
  double assignment_interval_;
  double task_timeout_;

  // 任务队列（优先级高的在前）
  std::vector<Task> pending_tasks_;
  std::vector<Task> active_tasks_;
  std::mutex tasks_mutex_;

  // AGV信息
  std::vector<AGVInfo> agv_infos_;
  std::mutex agv_mutex_;

  // Action客户端（每个AGV一个）
  std::map<std::string, rclcpp_action::Client<NavigateAction>::SharedPtr> action_clients_;

  // AGV状态订阅器
  std::vector<rclcpp::Subscription<agv_interfaces::msg::AGVStatus>::SharedPtr> status_subs_;

  // 服务
  rclcpp::Service<agv_interfaces::srv::AssignTask>::SharedPtr assign_service_;

  // 定时器
  rclcpp::TimerBase::SharedPtr assignment_timer_;

  // 任务ID计数器
  int task_counter_;

  // TF监听器（用于查询AGV位置）
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 定时更新AGV位置
  void updateAGVPositions();
  rclcpp::TimerBase::SharedPtr position_timer_;
};

}  // namespace agv_scheduler

#endif  // AGV_SCHEDULER__TASK_SCHEDULER_HPP_
