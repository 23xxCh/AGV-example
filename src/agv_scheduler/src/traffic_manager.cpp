/**
 * ============================================================
 * traffic_manager.cpp - 交通管理器实现
 * ============================================================
 *
 * 【核心算法】
 * 1. 路径预约：将路径分解为格子序列，计算每个格子的时间窗口
 * 2. 冲突检测：检查新预约与已有预约是否在时间和空间上重叠
 * 3. 安全距离：不仅检查当前格子，还检查周围格子（防止车体碰撞）
 */

#include "agv_scheduler/traffic_manager.hpp"
#include <cmath>
#include <algorithm>

namespace agv_scheduler
{

TrafficManager::TrafficManager(const rclcpp::NodeOptions & options)
: Node("traffic_manager", options)
{
  // 声明参数
  this->declare_parameter("resolution", 0.05);
  this->declare_parameter("origin_x", 0.0);
  this->declare_parameter("origin_y", 0.0);
  this->declare_parameter("safety_distance", 2);
  this->declare_parameter("reservation_timeout", 60.0);
  this->declare_parameter("deadlock_timeout", 30.0);

  resolution_ = this->get_parameter("resolution").as_double();
  origin_x_ = this->get_parameter("origin_x").as_double();
  origin_y_ = this->get_parameter("origin_y").as_double();
  safety_distance_ = this->get_parameter("safety_distance").as_int();
  reservation_timeout_ = this->get_parameter("reservation_timeout").as_double();
  deadlock_timeout_ = this->get_parameter("deadlock_timeout").as_double();

  // 创建服务
  reserve_service_ = this->create_service<agv_interfaces::srv::ReservePath>(
    "reserve_path",
    std::bind(&TrafficManager::handleReservePath, this,
              std::placeholders::_1, std::placeholders::_2));

  release_service_ = this->create_service<agv_interfaces::srv::ReservePath>(
    "release_path",
    std::bind(&TrafficManager::handleReleasePath, this,
              std::placeholders::_1, std::placeholders::_2));

  // 创建清理定时器（每秒清理一次过期预约）
  cleanup_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&TrafficManager::cleanupExpired, this));

  // 创建死锁检测定时器（每2秒检测一次）
  deadlock_timer_ = this->create_wall_timer(
    std::chrono::seconds(2),
    std::bind(&TrafficManager::detectDeadlock, this));

  RCLCPP_INFO(this->get_logger(),
    "交通管理器已启动: 分辨率=%.3f, 安全距离=%d格, 超时=%.0fs, 死锁超时=%.0fs",
    resolution_, safety_distance_, reservation_timeout_, deadlock_timeout_);
}

// ============================================================
// handleReservePath - 处理路径预约请求
// ============================================================
void TrafficManager::handleReservePath(
  const std::shared_ptr<agv_interfaces::srv::ReservePath::Request> request,
  std::shared_ptr<agv_interfaces::srv::ReservePath::Response> response)
{
  std::string conflict_agv_id;
  double wait_time = 0.0;

  bool has_conflict = checkConflict(
    request->agv_id, request->path,
    request->start_time, request->speed,
    conflict_agv_id, wait_time);

  if (has_conflict) {
    response->success = false;
    response->conflict_agv_id = conflict_agv_id;
    response->wait_time = wait_time;
    response->error_msg = "路径冲突: 与 " + conflict_agv_id + " 的预约冲突";

    // 记录等待依赖（用于死锁检测）
    {
      std::lock_guard<std::mutex> lock(wait_mutex_);
      wait_graph_[request->agv_id] = conflict_agv_id;
      if (wait_start_times_.find(request->agv_id) == wait_start_times_.end()) {
        wait_start_times_[request->agv_id] = this->now().seconds();
      }
    }

    RCLCPP_WARN(this->get_logger(),
      "预约被拒: %s 与 %s 冲突, 建议等待 %.1fs",
      request->agv_id.c_str(), conflict_agv_id.c_str(), wait_time);
    return;
  }

  // 无冲突，记录预约
  std::lock_guard<std::mutex> lock(table_mutex_);
  double current_time = request->start_time;
  double speed = request->speed;

  for (size_t i = 0; i < request->path.poses.size(); ++i) {
    double px = request->path.poses[i].pose.position.x;
    double py = request->path.poses[i].pose.position.y;
    CellKey cell = worldToGrid(px, py);

    // 计算到达下一个格子的时间
    double next_time = current_time;
    if (i < request->path.poses.size() - 1) {
      double nx = request->path.poses[i + 1].pose.position.x;
      double ny = request->path.poses[i + 1].pose.position.y;
      double dist = std::sqrt((nx - px) * (nx - px) + (ny - py) * (ny - py));
      next_time = current_time + dist / speed;
    } else {
      next_time = current_time + 1.0;  // 最后一个格子停留1秒
    }

    // 对安全距离内的所有格子都预约
    for (int dx = -safety_distance_; dx <= safety_distance_; ++dx) {
      for (int dy = -safety_distance_; dy <= safety_distance_; ++dy) {
        CellKey neighbor = {cell.x + dx, cell.y + dy};
        Reservation res;
        res.agv_id = request->agv_id;
        res.start_time = current_time;
        res.end_time = next_time;
        reservation_table_[neighbor].push_back(res);
      }
    }

    current_time = next_time;
  }

  response->success = true;
  response->error_msg = "";

  // 清除等待依赖（预约成功）
  {
    std::lock_guard<std::mutex> lock(wait_mutex_);
    wait_graph_.erase(request->agv_id);
    wait_start_times_.erase(request->agv_id);
  }

  RCLCPP_INFO(this->get_logger(),
    "预约成功: %s, %zu个路径点", request->agv_id.c_str(), request->path.poses.size());
}

// ============================================================
// handleReleasePath - 处理路径释放请求
// ============================================================
void TrafficManager::handleReleasePath(
  const std::shared_ptr<agv_interfaces::srv::ReservePath::Request> request,
  std::shared_ptr<agv_interfaces::srv::ReservePath::Response> response)
{
  std::lock_guard<std::mutex> lock(table_mutex_);

  int removed = 0;
  for (auto & [cell, reservations] : reservation_table_) {
    auto it = std::remove_if(reservations.begin(), reservations.end(),
      [&](const Reservation & r) {
        return r.agv_id == request->agv_id;
      });
    removed += std::distance(it, reservations.end());
    reservations.erase(it, reservations.end());
  }

  response->success = true;
  response->error_msg = "";

  // 清除等待依赖（路径释放）
  {
    std::lock_guard<std::mutex> lock(wait_mutex_);
    wait_graph_.erase(request->agv_id);
    wait_start_times_.erase(request->agv_id);
  }

  RCLCPP_INFO(this->get_logger(),
    "释放预约: %s, 清除%d条记录", request->agv_id.c_str(), removed);
}

// ============================================================
// cleanupExpired - 清理过期预约
// ============================================================
void TrafficManager::cleanupExpired()
{
  std::lock_guard<std::mutex> lock(table_mutex_);
  double now = this->now().seconds();

  int cleaned = 0;
  for (auto & [cell, reservations] : reservation_table_) {
    auto it = std::remove_if(reservations.begin(), reservations.end(),
      [&](const Reservation & r) {
        return (now - r.end_time) > reservation_timeout_;
      });
    cleaned += std::distance(it, reservations.end());
    reservations.erase(it, reservations.end());
  }

  if (cleaned > 0) {
    RCLCPP_DEBUG(this->get_logger(), "清理过期预约: %d条", cleaned);
  }
}

// ============================================================
// checkConflict - 检查路径是否有冲突
// ============================================================
bool TrafficManager::checkConflict(
  const std::string & agv_id,
  const nav_msgs::msg::Path & path,
  double start_time,
  double speed,
  std::string & conflict_agv_id,
  double & wait_time)
{
  std::lock_guard<std::mutex> lock(table_mutex_);
  double current_time = start_time;
  double max_wait = 0.0;

  for (size_t i = 0; i < path.poses.size(); ++i) {
    double px = path.poses[i].pose.position.x;
    double py = path.poses[i].pose.position.y;
    CellKey cell = worldToGrid(px, py);

    // 计算到达下一个格子的时间
    double next_time = current_time;
    if (i < path.poses.size() - 1) {
      double nx = path.poses[i + 1].pose.position.x;
      double ny = path.poses[i + 1].pose.position.y;
      double dist = std::sqrt((nx - px) * (nx - px) + (ny - py) * (ny - py));
      next_time = current_time + dist / speed;
    } else {
      next_time = current_time + 1.0;
    }

    // 检查安全距离内的所有格子
    for (int dx = -safety_distance_; dx <= safety_distance_; ++dx) {
      for (int dy = -safety_distance_; dy <= safety_distance_; ++dy) {
        CellKey neighbor = {cell.x + dx, cell.y + dy};
        auto it = reservation_table_.find(neighbor);
        if (it != reservation_table_.end()) {
          for (const auto & res : it->second) {
            // 跳过自己的预约
            if (res.agv_id == agv_id) continue;

            // 检查时间重叠
            if (current_time < res.end_time && next_time > res.start_time) {
              conflict_agv_id = res.agv_id;
              wait_time = res.end_time - current_time;
              if (wait_time > max_wait) max_wait = wait_time;
              return true;
            }
          }
        }
      }
    }

    current_time = next_time;
  }

  wait_time = 0.0;
  return false;
}

// ============================================================
// worldToGrid - 世界坐标转格子坐标
// ============================================================
CellKey TrafficManager::worldToGrid(double x, double y) const
{
  CellKey cell;
  cell.x = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  cell.y = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  return cell;
}

// ============================================================
// detectDeadlock - 检测死锁
// ============================================================
// 死锁检测算法：
// 1. 构建等待图（AGV A -> AGV B 表示A在等待B）
// 2. 检测图中是否有环（A等B，B等A = 死锁）
// 3. 如果有环，选择"牺牲者"（等待时间最长的AGV）
// 4. 强制释放牺牲者的路径预约，让它重新规划
void TrafficManager::detectDeadlock()
{
  std::string victim_agv;

  if (!hasCycle(victim_agv)) {
    return;  // 没有死锁
  }

  // 检查等待时间是否超过阈值
  {
    std::lock_guard<std::mutex> lock(wait_mutex_);
    auto it = wait_start_times_.find(victim_agv);
    if (it != wait_start_times_.end()) {
      double wait_duration = this->now().seconds() - it->second;
      if (wait_duration < deadlock_timeout_) {
        RCLCPP_INFO(this->get_logger(),
          "检测到潜在死锁（%s），但等待时间%.1fs未超时%.0fs，暂不处理",
          victim_agv.c_str(), wait_duration, deadlock_timeout_);
        return;
      }
    }
  }

  // 强制释放牺牲者的路径
  RCLCPP_WARN(this->get_logger(),
    "检测到死锁！强制释放 %s 的路径预约", victim_agv.c_str());
  forceRelease(victim_agv);
}

// ============================================================
// hasCycle - 检测等待图中是否有环
// ============================================================
// 使用Floyd判圈算法（龟兔赛跑）检测环
// 如果有环，返回true并通过victim_agv返回牺牲者（等待时间最长的AGV）
bool TrafficManager::hasCycle(std::string & victim_agv)
{
  std::lock_guard<std::mutex> lock(wait_mutex_);

  if (wait_graph_.empty()) {
    return false;
  }

  // 检查每个AGV的等待链
  for (const auto & [start_agv, _] : wait_graph_) {
    std::string slow = start_agv;
    std::string fast = start_agv;

    // 龟兔赛跑找环
    while (true) {
      // 慢指针走一步
      auto slow_it = wait_graph_.find(slow);
      if (slow_it == wait_graph_.end()) break;
      slow = slow_it->second;

      // 快指针走两步
      auto fast_it1 = wait_graph_.find(fast);
      if (fast_it1 == wait_graph_.end()) break;
      fast = fast_it1->second;

      auto fast_it2 = wait_graph_.find(fast);
      if (fast_it2 == wait_graph_.end()) break;
      fast = fast_it2->second;

      // 如果快慢指针相遇，说明有环
      if (slow == fast) {
        // 找到环，选择等待时间最长的AGV作为牺牲者
        double max_wait = 0.0;
        for (const auto & [agv, _] : wait_graph_) {
          auto time_it = wait_start_times_.find(agv);
          if (time_it != wait_start_times_.end()) {
            double wait_duration = this->now().seconds() - time_it->second;
            if (wait_duration > max_wait) {
              max_wait = wait_duration;
              victim_agv = agv;
            }
          }
        }
        return true;
      }
    }
  }

  return false;
}

// ============================================================
// forceRelease - 强制释放AGV的路径预约
// ============================================================
void TrafficManager::forceRelease(const std::string & agv_id)
{
  std::lock_guard<std::mutex> lock(table_mutex_);

  int removed = 0;
  for (auto & [cell, reservations] : reservation_table_) {
    auto it = std::remove_if(reservations.begin(), reservations.end(),
      [&](const Reservation & r) {
        return r.agv_id == agv_id;
      });
    removed += std::distance(it, reservations.end());
    reservations.erase(it, reservations.end());
  }

  // 清除等待依赖
  {
    std::lock_guard<std::mutex> wait_lock(wait_mutex_);
    wait_graph_.erase(agv_id);
    wait_start_times_.erase(agv_id);
  }

  RCLCPP_WARN(this->get_logger(),
    "强制释放 %s 的路径预约（死锁解除），清除%d条记录", agv_id.c_str(), removed);
}

}  // namespace agv_scheduler
