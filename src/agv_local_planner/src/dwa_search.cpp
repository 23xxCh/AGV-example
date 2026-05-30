/**
 * ============================================================
 * dwa_search.cpp - DWA算法实现
 * ============================================================
 *
 * 本文件实现DWA算法的全部逻辑，每一步都有详细中文注释
 * 建议按照 computeVelocity() 的调用顺序阅读
 */

#include "agv_local_planner/dwa_search.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>

namespace agv_local_planner
{

// ============================================================
// 构造函数
// ============================================================
DWASearch::DWASearch(const DWAParams & params)
: params_(params)
{
}

// ============================================================
// computeVelocity - DWA主入口
// ============================================================
Velocity DWASearch::computeVelocity(
  const Pose2D & robot_pose,
  const Velocity & robot_vel,
  const Pose2D & goal_pose,
  const std::vector<uint8_t> & costmap,
  unsigned int map_width,
  unsigned int map_height,
  double map_resolution,
  double map_origin_x,
  double map_origin_y)
{
  // 清空上一次的候选轨迹
  recent_trajectories_.clear();

  // ----------------------------------------------------------
  // 步骤1：计算动态窗口
  // ----------------------------------------------------------
  // 动态窗口是当前可选的速度范围
  // 它是"硬件限速"和"当前速度可达范围"的交集
  DynamicWindow dw = calculateDynamicWindow(robot_vel);

  // ----------------------------------------------------------
  // 步骤2-4：遍历所有候选速度，模拟轨迹并评分
  // ----------------------------------------------------------
  double best_score = -std::numeric_limits<double>::infinity();
  Velocity best_vel(0.0, 0.0);
  bool found_valid = false;

  // 在动态窗口内均匀采样速度对 (v, ω)
  // 例如：vx_samples=10, vtheta_samples=20
  // 则总共评估 10*20 = 200 条候选轨迹
  double v_step = (dw.v_max - dw.v_min) / std::max(params_.vx_samples - 1, 1);
  double omega_step = (dw.omega_max - dw.omega_min) / std::max(params_.vtheta_samples - 1, 1);

  for (int i = 0; i < params_.vx_samples; ++i) {
    double v = dw.v_min + i * v_step;

    for (int j = 0; j < params_.vtheta_samples; ++j) {
      double omega = dw.omega_min + j * omega_step;

      // 步骤2：前向模拟轨迹
      // 假设机器人以(v, ω)匀速运动sim_time秒，计算会走出什么路径
      Trajectory traj = simulateTrajectory(robot_pose, v, omega);

      // 步骤3：检查碰撞
      // 如果轨迹上任何一点在障碍物上，该轨迹不可选
      // 评分函数返回-1表示碰撞
      double score = scoreTrajectory(
        traj, goal_pose, costmap,
        map_width, map_height, map_resolution, map_origin_x, map_origin_y);

      // 碰撞轨迹直接跳过
      if (score < 0.0) {
        continue;
      }

      // 保存轨迹用于可视化（可选，保留最近的轨迹）
      if (recent_trajectories_.size() < 200) {
        recent_trajectories_.push_back(traj);
      }

      // 步骤4：选择最高分的轨迹
      if (score > best_score) {
        best_score = score;
        best_vel.v = v;
        best_vel.omega = omega;
        found_valid = true;
      }
    }
  }

  // 如果没有找到任何可行轨迹（全部碰撞），则紧急停止
  if (!found_valid) {
    // 尝试原地旋转寻找出路
    // 如果连旋转都碰撞，就完全停止
    Velocity stop_vel(0.0, 0.0);

    // 尝试原地左右转
    for (double try_omega : {params_.max_vel_theta, -params_.max_vel_theta}) {
      Trajectory traj = simulateTrajectory(robot_pose, 0.0, try_omega);
      double score = scoreTrajectory(
        traj, goal_pose, costmap,
        map_width, map_height, map_resolution, map_origin_x, map_origin_y);
      if (score >= 0.0) {
        stop_vel.omega = try_omega * 0.5;  // 减速旋转
        break;
      }
    }

    return stop_vel;
  }

  return best_vel;
}

// ============================================================
// calculateDynamicWindow - 计算动态窗口
// ============================================================
DynamicWindow DWASearch::calculateDynamicWindow(const Velocity & current_vel) const
{
  DynamicWindow dw;

  // 第一层限制：硬件物理限制（Vs）
  // 机器人硬件能提供的最大/最小速度
  dw.v_min = params_.min_vel_x;
  dw.v_max = params_.max_vel_x;
  dw.omega_min = params_.min_vel_theta;
  dw.omega_max = params_.max_vel_theta;

  // 第二层限制：当前速度可达范围（Vd）
  // 考虑加速度限制，在模拟时间内能达到的速度范围
  // 使用sim_time作为时间窗口，这样机器人可以在整个模拟周期内加速
  double dt = params_.sim_time;  // 使用模拟时间（如2秒）
  double v_min_reachable = current_vel.v - params_.acc_lim_x * dt;
  double v_max_reachable = current_vel.v + params_.acc_lim_x * dt;
  double omega_min_reachable = current_vel.omega - params_.acc_lim_theta * dt;
  double omega_max_reachable = current_vel.omega + params_.acc_lim_theta * dt;

  // 取交集：确保速度同时满足硬件限制和可达性
  dw.v_min = std::max(dw.v_min, v_min_reachable);
  dw.v_max = std::min(dw.v_max, v_max_reachable);
  dw.omega_min = std::max(dw.omega_min, omega_min_reachable);
  dw.omega_max = std::min(dw.omega_max, omega_max_reachable);

  // 确保窗口有效（最小值不大于最大值）
  dw.v_min = std::min(dw.v_min, dw.v_max);
  dw.omega_min = std::min(dw.omega_min, dw.omega_max);

  return dw;
}

// ============================================================
// simulateTrajectory - 前向模拟轨迹
// ============================================================
Trajectory DWASearch::simulateTrajectory(
  const Pose2D & start_pose,
  double v,
  double omega) const
{
  Trajectory trajectory;
  trajectory.v = v;
  trajectory.omega = omega;

  // 前向模拟使用差分运动学模型
  // 这是两轮差分驱动AGV的基本运动模型：
  //
  //   左轮速度(vl) 和 右轮速度(vr) 决定了机器人的运动：
  //   - 线速度 v = (vl + vr) / 2
  //   - 角速度 ω = (vr - vl) / 轮距
  //
  //   运动方程（每步更新）：
  //   x_new     = x     + v * cos(θ) * dt
  //   y_new     = y     + v * sin(θ) * dt
  //   theta_new = theta + ω * dt
  //
  //   当 ω ≠ 0 时，实际走出的是圆弧：
  //   半径 R = v / ω
  //   圆心 = (x - R*sin(θ), y + R*cos(θ))
  //
  //   当 ω = 0 时，走出的是直线

  double x = start_pose.x;
  double y = start_pose.y;
  double theta = start_pose.theta;

  // 模拟时间步长和总时长
  double dt = params_.sim_granularity;  // 通常0.1秒
  double total_time = params_.sim_time;  // 通常2.0秒

  // 每隔dt秒记录一个位姿
  for (double t = 0.0; t < total_time; t += dt) {
    // 更新位姿
    x += v * std::cos(theta) * dt;
    y += v * std::sin(theta) * dt;
    theta += omega * dt;

    // 角度归一化到 [-π, π]
    // 避免角度无限增长导致数值问题
    while (theta > M_PI) theta -= 2.0 * M_PI;
    while (theta < -M_PI) theta += 2.0 * M_PI;

    trajectory.poses.emplace_back(x, y, theta);
  }

  return trajectory;
}

// ============================================================
// scoreTrajectory - 评分函数
// ============================================================
double DWASearch::scoreTrajectory(
  const Trajectory & trajectory,
  const Pose2D & goal,
  const std::vector<uint8_t> & costmap,
  unsigned int map_width,
  unsigned int map_height,
  double map_resolution,
  double map_origin_x,
  double map_origin_y) const
{
  // ---- 首先检查碰撞 ----
  // 如果轨迹上任何一点在障碍物上，直接返回-1（不可选）
  // 这是最基本的安全检查
  double clearance = computeClearanceScore(
    trajectory, costmap,
    map_width, map_height, map_resolution, map_origin_x, map_origin_y);

  if (clearance < 0.0) {
    return -1.0;  // 碰撞！
  }

  // ---- 计算三个评分分量 ----
  // 所有分量都归一化到[0,1]，然后加权求和

  // 分量1：heading - 轨迹末端朝向与目标方向的对齐程度
  // 越朝向目标，得分越高
  double heading = computeHeadingScore(trajectory.endPose(), goal);

  // 分量2：clearance - 轨迹与障碍物的最小距离
  // 已在上面计算，值越大表示越远离障碍物
  // clearance已在[0,1]范围内

  // 分量3：velocity - 前进速度
  // 速度越快得分越高（鼓励高效移动）
  double velocity = computeVelocityScore(trajectory.v);

  // ---- 加权求和 ----
  // G = α * heading + β * clearance + γ * velocity
  //
  // 权重调参指南：
  // - α 大：路径跟踪紧，但可能忽略障碍物
  // - β 大：朝向目标强，但可能偏离路径
  // - γ 大：避障保守，但可能走得太慢
  //
  // 仓储场景建议：α=0.6, β=0.8, γ=0.5
  double score = params_.path_distance_bias * heading
               + params_.goal_distance_bias * clearance
               + params_.occdist_scale * velocity;

  // 额外惩罚后退运动（鼓励机器人朝前走）
  if (trajectory.v < 0.0) {
    score *= 0.3;  // 后退轨迹得分打3折
  }

  return score;
}

// ============================================================
// computeHeadingScore - 计算朝向评分
// ============================================================
double DWASearch::computeHeadingScore(
  const Pose2D & end_pose,
  const Pose2D & goal) const
{
  // 计算轨迹末端到目标的方向角
  double dx = goal.x - end_pose.x;
  double dy = goal.y - end_pose.y;
  double angle_to_goal = std::atan2(dy, dx);

  // 计算轨迹末端的朝向与目标方向的角度差
  double angle_diff = std::abs(angle_to_goal - end_pose.theta);

  // 角度差归一化到 [0, π]
  while (angle_diff > M_PI) angle_diff = 2.0 * M_PI - angle_diff;

  // 归一化到 [0, 1]
  // angle_diff=0 表示完美朝向目标，得分为1
  // angle_diff=π 表示完全背向目标，得分为0
  double heading_score = (M_PI - angle_diff) / M_PI;

  return heading_score;
}

// ============================================================
// computeClearanceScore - 计算避障评分
// ============================================================
double DWASearch::computeClearanceScore(
  const Trajectory & trajectory,
  const std::vector<uint8_t> & costmap,
  unsigned int map_width,
  unsigned int map_height,
  double map_resolution,
  double map_origin_x,
  double map_origin_y) const
{
  // 遍历轨迹上每个点，查询代价地图
  // 找到轨迹与最近障碍物的最小距离
  double min_distance = std::numeric_limits<double>::infinity();

  for (const auto & pose : trajectory.poses) {
    // 查询该点在代价地图上的值
    int gx, gy;
    if (!worldToGrid(pose.x, pose.y, map_width, map_height,
                     map_resolution, map_origin_x, map_origin_y, gx, gy)) {
      // 点在地图外，视为碰撞
      return -1.0;
    }

    size_t idx = static_cast<size_t>(gy) * map_width + static_cast<size_t>(gx);
    uint8_t cost = costmap[idx];

    // 代价地图值含义：
    // 0 = 完全自由，1~252 = 代价递增，253 = 内切障碍，254 = 外切障碍，255 = 致命
    if (cost >= 253) {
      // 致命障碍或内切障碍，该轨迹碰撞！
      return -1.0;
    }

    // 代价越高，距离障碍物越近
    // 将代价转换为距离估计（粗略）
    // cost=0 → 距离很远，cost=252 → 距离很近
    double distance = (252.0 - cost) / 252.0 * 10.0;  // 粗略映射到0~10米
    min_distance = std::min(min_distance, distance);
  }

  // 归一化到 [0, 1]
  // 使用sigmoid函数将距离映射到(0,1)
  // 距离越远，得分越接近1
  double clearance_score = min_distance / (min_distance + 1.0);

  return clearance_score;
}

// ============================================================
// computeVelocityScore - 计算速度评分
// ============================================================
double DWASearch::computeVelocityScore(double v) const
{
  // 归一化到 [0, 1]
  // 鼓励机器人尽可能快地移动
  // v=0 → 得分0，v=max_vel_x → 得分1
  if (params_.max_vel_x <= 0.0) {
    return 0.0;
  }
  return std::abs(v) / params_.max_vel_x;
}

// ============================================================
// worldToGrid - 世界坐标转格子坐标
// ============================================================
bool DWASearch::worldToGrid(
  double wx, double wy,
  unsigned int map_width, unsigned int map_height,
  double map_resolution, double map_origin_x, double map_origin_y,
  int & grid_x, int & grid_y) const
{
  grid_x = static_cast<int>(std::floor((wx - map_origin_x) / map_resolution));
  grid_y = static_cast<int>(std::floor((wy - map_origin_y) / map_resolution));

  return (grid_x >= 0 && grid_y >= 0 &&
          static_cast<unsigned int>(grid_x) < map_width &&
          static_cast<unsigned int>(grid_y) < map_height);
}

// ============================================================
// isTraversable - 检查是否可通行
// ============================================================
bool DWASearch::isTraversable(
  double wx, double wy,
  const std::vector<uint8_t> & costmap,
  unsigned int map_width,
  unsigned int map_height,
  double map_resolution,
  double map_origin_x,
  double map_origin_y) const
{
  int gx, gy;
  if (!worldToGrid(wx, wy, map_width, map_height,
                   map_resolution, map_origin_x, map_origin_y, gx, gy)) {
    return false;
  }

  size_t idx = static_cast<size_t>(gy) * map_width + static_cast<size_t>(gx);
  return costmap[idx] < 253;
}

// ============================================================
// isGoalReached - 检查是否到达目标
// ============================================================
bool DWASearch::isGoalReached(
  const Pose2D & robot_pose,
  const Pose2D & goal_pose) const
{
  double dx = goal_pose.x - robot_pose.x;
  double dy = goal_pose.y - robot_pose.y;
  double distance = std::sqrt(dx * dx + dy * dy);

  double angle_diff = std::abs(goal_pose.theta - robot_pose.theta);
  while (angle_diff > M_PI) angle_diff = 2.0 * M_PI - angle_diff;

  return (distance < params_.goal_tolerance_xy &&
          angle_diff < params_.goal_tolerance_yaw);
}

}  // namespace agv_local_planner
