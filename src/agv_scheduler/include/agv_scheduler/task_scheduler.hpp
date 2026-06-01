/**
 * ============================================================
 * task_scheduler.hpp - 任务调度器节点（优先级队列版）
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
 * 最近空闲优先 + 优先级队列：
 * - 任务按优先级入队（priority_queue，高优先级在前）
 * - 优先级老化：等待越久的任务优先级自动提升，防止饥饿
 * - 任务抢占：紧急任务可抢占低优先级任务的AGV
 * - 最近空闲分配：同优先级任务优先分配给最近的空闲AGV
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
#include "agv_interfaces/msg/fleet_status.hpp"
#include "agv_interfaces/srv/assign_task.hpp"
#include "agv_interfaces/action/navigate.hpp"

namespace agv_scheduler
{

// 任务信息
struct Task {
  std::string task_id;
  double goal_x, goal_y, goal_theta;
  uint8_t priority;       // 原始优先级（1-10，越高越紧急）
  double effective_priority;  // 有效优先级（含老化加成）
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

// 优先级比较器（effective_priority高的排前面）
struct TaskComparator {
  bool operator()(const Task & a, const Task & b) const {
    return a.effective_priority < b.effective_priority;
  }
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

  // 更新优先级老化
  void updateAging();

  // 尝试分配待处理的任务
  void tryAssignTasks();

  // 尝试抢占（高优先级任务抢占低优先级任务的AGV）
  bool tryPreempt(const Task & task);

  // 找到最近的空闲AGV
  std::string findNearestIdleAGV(double goal_x, double goal_y);

  // 找到可被抢占的AGV（正在执行低优先级任务）
  std::string findPreemptableAGV(double goal_x, double goal_y, uint8_t min_priority);

  // 发送导航目标给AGV（返回是否成功发送）
  bool sendGoalToAGV(const std::string & agv_id, const Task & task);

  // 取消AGV当前任务
  void cancelAGVTask(const std::string & agv_id);

  // AGV状态回调
  void agvStatusCallback(const std::string & agv_id,
                          const agv_interfaces::msg::AGVStatus::SharedPtr msg);

  // 发布车队状态
  void publishFleetStatus();

  // 参数
  std::vector<std::string> agv_ids_;
  double assignment_interval_;
  double task_timeout_;
  double aging_rate_;           // 优先级老化速率（每秒增加的优先级）
  double max_aging_bonus_;      // 最大老化加成
  bool enable_preemption_;      // 是否启用抢占
  uint8_t preemption_threshold_;  // 抢占阈值（优先级差值）

  // 任务队列（优先级队列，高优先级在前）
  std::priority_queue<Task, std::vector<Task>, TaskComparator> pending_queue_;
  std::vector<Task> active_tasks_;
  std::mutex tasks_mutex_;

  // AGV信息
  std::vector<AGVInfo> agv_infos_;
  std::mutex agv_mutex_;

  // Action客户端（每个AGV一个）
  std::map<std::string, rclcpp_action::Client<NavigateAction>::SharedPtr> action_clients_;

  // Goal句柄（用于取消正在执行的任务）
  std::map<std::string, std::shared_ptr<GoalHandleNavigate>> goal_handles_;
  std::mutex goal_handles_mutex_;

  // AGV状态订阅器
  std::vector<rclcpp::Subscription<agv_interfaces::msg::AGVStatus>::SharedPtr> status_subs_;

  // 服务
  rclcpp::Service<agv_interfaces::srv::AssignTask>::SharedPtr assign_service_;

  // 车队状态发布器
  rclcpp::Publisher<agv_interfaces::msg::FleetStatus>::SharedPtr fleet_status_pub_;

  // 定时器
  rclcpp::TimerBase::SharedPtr assignment_timer_;
  rclcpp::TimerBase::SharedPtr fleet_status_timer_;

  // 任务ID计数器
  int task_counter_;

  // 统计信息
  int total_completed_;
  int total_failed_;

  // TF监听器（用于查询AGV位置）
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 定时更新AGV位置
  void updateAGVPositions();
  rclcpp::TimerBase::SharedPtr position_timer_;
};

}  // namespace agv_scheduler

#endif  // AGV_SCHEDULER__TASK_SCHEDULER_HPP_
