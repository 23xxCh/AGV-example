/**
 * ============================================================
 * dwa_planner.cpp - DWA局部规划器ROS2节点实现
 * ============================================================
 *
 * 本文件实现DWAPlanner类，将DWA算法封装为ROS2节点
 * 核心流程：订阅地图+路径 → 查询TF → DWA计算 → 发布速度
 */

#include "agv_local_planner/dwa_planner.hpp"
#include <functional>
#include <algorithm>

namespace agv_local_planner
{

// ============================================================
// 构造函数
// ============================================================
DWAPlanner::DWAPlanner(const rclcpp::NodeOptions & options)
: Node("dwa_planner", options),
  costmap_received_(false),
  dynamic_costmap_received_(false),
  path_received_(false),
  local_goal_index_(0)
{
  // ----------------------------------------------------------
  // 声明和读取参数
  // ----------------------------------------------------------
  this->declare_parameter("base_frame", std::string("base_link"));
  this->declare_parameter("map_frame", std::string("map"));
  this->declare_parameter("control_frequency", 10.0);
  this->declare_parameter("costmap_topic", std::string("map"));
  this->declare_parameter("path_topic", std::string("planned_path"));

  base_frame_ = this->get_parameter("base_frame").as_string();
  map_frame_ = this->get_parameter("map_frame").as_string();
  control_frequency_ = this->get_parameter("control_frequency").as_double();

  // ----------------------------------------------------------
  // 加载DWA参数并创建算法实例
  // ----------------------------------------------------------
  DWAParams params = loadParams();
  dwa_ = std::make_unique<DWASearch>(params);
  goal_tolerance_xy_ = params.goal_tolerance_xy;

  RCLCPP_INFO(this->get_logger(), "DWA规划器初始化: max_v=%.2f, max_w=%.2f, sim_time=%.2f",
    params.max_vel_x, params.max_vel_theta, params.sim_time);

  // ----------------------------------------------------------
  // 创建TF监听器
  // ----------------------------------------------------------
  // TF监听器会在后台持续接收坐标变换数据
  // 我们稍后通过 tf_buffer_->lookupTransform() 查询机器人位姿
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ----------------------------------------------------------
  // 创建订阅器
  // ----------------------------------------------------------
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

  std::string costmap_topic = this->get_parameter("costmap_topic").as_string();
  std::string path_topic = this->get_parameter("path_topic").as_string();

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    costmap_topic, qos,
    std::bind(&DWAPlanner::costmapCallback, this, std::placeholders::_1));

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    path_topic, rclcpp::QoS(10).reliable(),
    std::bind(&DWAPlanner::pathCallback, this, std::placeholders::_1));

  // 动态障碍物代价地图订阅
  dynamic_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "dynamic_costmap", rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&DWAPlanner::dynamicCostmapCallback, this, std::placeholders::_1));

  // ----------------------------------------------------------
  // 创建发布器
  // ----------------------------------------------------------
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(10).reliable());

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "dwa_markers", rclcpp::QoS(10).reliable());

  // ----------------------------------------------------------
  // 创建控制定时器
  // ----------------------------------------------------------
  // 定时器以固定频率触发controlTimerCallback
  // 这是DWA控制循环的"心跳"
  auto period = std::chrono::duration<double>(1.0 / control_frequency_);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&DWAPlanner::controlTimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "DWA规划器启动完成，控制频率=%.1f Hz", control_frequency_);
}

// ============================================================
// loadParams - 加载DWA参数
// ============================================================
DWAParams DWAPlanner::loadParams()
{
  DWAParams p;

  // 声明所有参数（带默认值）
  this->declare_parameter("max_vel_x", 0.5);
  this->declare_parameter("min_vel_x", -0.1);
  this->declare_parameter("max_vel_theta", 1.5);
  this->declare_parameter("min_vel_theta", -1.5);
  this->declare_parameter("acc_lim_x", 1.0);
  this->declare_parameter("acc_lim_theta", 2.0);
  this->declare_parameter("sim_time", 2.0);
  this->declare_parameter("sim_granularity", 0.1);
  this->declare_parameter("vx_samples", 10);
  this->declare_parameter("vtheta_samples", 20);
  this->declare_parameter("path_distance_bias", 0.6);
  this->declare_parameter("goal_distance_bias", 0.8);
  this->declare_parameter("occdist_scale", 0.5);
  this->declare_parameter("goal_tolerance_xy", 0.10);
  this->declare_parameter("goal_tolerance_yaw", 0.2);
  this->declare_parameter("stop_time_buffer", 0.5);
  this->declare_parameter("oscillation_dist", 0.3);

  // 读取参数值
  p.max_vel_x = this->get_parameter("max_vel_x").as_double();
  p.min_vel_x = this->get_parameter("min_vel_x").as_double();
  p.max_vel_theta = this->get_parameter("max_vel_theta").as_double();
  p.min_vel_theta = this->get_parameter("min_vel_theta").as_double();
  p.acc_lim_x = this->get_parameter("acc_lim_x").as_double();
  p.acc_lim_theta = this->get_parameter("acc_lim_theta").as_double();
  p.sim_time = this->get_parameter("sim_time").as_double();
  p.sim_granularity = this->get_parameter("sim_granularity").as_double();
  p.vx_samples = this->get_parameter("vx_samples").as_int();
  p.vtheta_samples = this->get_parameter("vtheta_samples").as_int();
  p.path_distance_bias = this->get_parameter("path_distance_bias").as_double();
  p.goal_distance_bias = this->get_parameter("goal_distance_bias").as_double();
  p.occdist_scale = this->get_parameter("occdist_scale").as_double();
  p.goal_tolerance_xy = this->get_parameter("goal_tolerance_xy").as_double();
  p.goal_tolerance_yaw = this->get_parameter("goal_tolerance_yaw").as_double();
  p.stop_time_buffer = this->get_parameter("stop_time_buffer").as_double();
  p.oscillation_dist = this->get_parameter("oscillation_dist").as_double();
  p.control_frequency = control_frequency_;

  return p;
}

// ============================================================
// costmapCallback - 代价地图回调
// ============================================================
void DWAPlanner::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  cached_costmap_ = *msg;
  costmap_received_ = true;

  RCLCPP_INFO_ONCE(this->get_logger(),
    "DWA收到代价地图: %dx%d, 分辨率=%.3f",
    msg->info.width, msg->info.height, msg->info.resolution);
}

// ============================================================
// pathCallback - 全局路径回调
// ============================================================
void DWAPlanner::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  global_path_ = *msg;
  path_received_ = true;
  local_goal_index_ = 0;  // 重置局部目标索引

  RCLCPP_INFO(this->get_logger(),
    "DWA收到全局路径: %zu个路径点", msg->poses.size());
}

// ============================================================
// dynamicCostmapCallback - 动态障碍物代价地图回调
// ============================================================
void DWAPlanner::dynamicCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(dynamic_costmap_mutex_);
  dynamic_costmap_ = *msg;
  dynamic_costmap_received_ = true;
}

// ============================================================
// controlTimerCallback - 控制定时器（DWA主循环）
// ============================================================
void DWAPlanner::controlTimerCallback()
{
  // ----------------------------------------------------------
  // 前置检查
  // ----------------------------------------------------------
  if (!costmap_received_ || !path_received_) {
    // 还没收到地图或路径，什么都不做
    return;
  }

  // ----------------------------------------------------------
  // 步骤1：通过TF查询机器人当前位姿
  // ----------------------------------------------------------
  // lookupTransform("map", "base_link", ...) 的含义：
  //   "告诉我base_link在map坐标系中的位姿"
  //   即：机器人在地图中的位置和朝向
  //
  // 参数说明：
  //   target_frame: 目标坐标系（我们想知道什么坐标系下的位姿）
  //   source_frame: 源坐标系（机器人本体）
  //   time: 查询时间点（ros::Time(0)表示最新的可用变换）
  //   timeout: 等待超时时间
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      map_frame_, base_frame_, tf2::TimePointZero,
      std::chrono::milliseconds(100));
  } catch (const tf2::TransformException & ex) {
    // TF查询失败（可能是机器人还没发布TF）
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "无法查询TF (%s → %s): %s", map_frame_.c_str(), base_frame_.c_str(), ex.what());
    return;
  }

  // 从TF变换中提取位姿
  Pose2D robot_pose;
  robot_pose.x = transform.transform.translation.x;
  robot_pose.y = transform.transform.translation.y;

  // 从四元数提取yaw角
  // 四元数(qw, qx, qy, qz) → yaw = atan2(2*(qw*qz), 1 - 2*qz²)
  auto q = transform.transform.rotation;
  robot_pose.theta = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  // 获取机器人当前速度（使用上次发布的速度指令作为估计）
  Velocity robot_vel = last_cmd_vel_;

  // ----------------------------------------------------------
  // 步骤2：找到局部目标点
  // ----------------------------------------------------------
  Pose2D local_goal = findLocalGoal(robot_pose);

  // ----------------------------------------------------------
  // 步骤3：检查是否到达最终目标
  // ----------------------------------------------------------
  Pose2D final_goal;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (global_path_.poses.empty()) return;
    auto & last = global_path_.poses.back();
    final_goal.x = last.pose.position.x;
    final_goal.y = last.pose.position.y;
  }

  double dist_to_final = std::sqrt(
    std::pow(final_goal.x - robot_pose.x, 2) +
    std::pow(final_goal.y - robot_pose.y, 2));

  if (dist_to_final < goal_tolerance_xy_) {
    // 到达最终目标，停止
    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_pub_->publish(stop_cmd);
    RCLCPP_INFO(this->get_logger(), "DWA: 已到达目标！停止。 goal=(%.2f,%.2f) robot=(%.2f,%.2f) dist=%.3f tol=%.3f",
      final_goal.x, final_goal.y, robot_pose.x, robot_pose.y, dist_to_final, goal_tolerance_xy_);
    return;
  }

  // 接近目标时减速，防止过冲振荡
  double speed_scale = 1.0;
  if (dist_to_final < goal_tolerance_xy_ * 3.0) {
    speed_scale = dist_to_final / (goal_tolerance_xy_ * 3.0);
    speed_scale = std::max(0.05, speed_scale);  // 最低保持5%速度，允许缓慢接近目标
  }

  // ----------------------------------------------------------
  // 步骤4：调用DWA算法计算最优速度
  // ----------------------------------------------------------
  nav_msgs::msg::OccupancyGrid costmap_copy;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    costmap_copy = cached_costmap_;
  }

  // 将OccupancyGrid转换为uint8数组（与全局规划器相同的转换逻辑）
  std::vector<uint8_t> costmap_data(costmap_copy.data.size());
  for (size_t i = 0; i < costmap_copy.data.size(); ++i) {
    int8_t val = costmap_copy.data[i];
    if (val < 0) {
      costmap_data[i] = 255;  // 未知 → 致命
    } else if (val == 0) {
      costmap_data[i] = 0;    // 自由
    } else if (val == 100) {
      costmap_data[i] = 254;  // 障碍 → 外切
    } else {
      costmap_data[i] = static_cast<uint8_t>((val * 253) / 99);
    }
  }

  // 合并动态障碍物代价地图
  // 动态障碍物地图可能与静态地图有不同的分辨率和尺寸
  // 需要将动态障碍物的世界坐标转换为静态地图的格子坐标
  {
    std::lock_guard<std::mutex> lock(dynamic_costmap_mutex_);
    if (dynamic_costmap_received_) {
      double dyn_res = dynamic_costmap_.info.resolution;
      double dyn_ox = dynamic_costmap_.info.origin.position.x;
      double dyn_oy = dynamic_costmap_.info.origin.position.y;
      unsigned int dyn_w = dynamic_costmap_.info.width;
      unsigned int dyn_h = dynamic_costmap_.info.height;

      double stat_res = costmap_copy.info.resolution;
      double stat_ox = costmap_copy.info.origin.position.x;
      double stat_oy = costmap_copy.info.origin.position.y;
      unsigned int stat_w = costmap_copy.info.width;
      unsigned int stat_h = costmap_copy.info.height;

      // 遍历动态障碍物地图的每个格子
      for (unsigned int dy = 0; dy < dyn_h; ++dy) {
        for (unsigned int dx = 0; dx < dyn_w; ++dx) {
          int idx = dy * dyn_w + dx;
          if (dynamic_costmap_.data[idx] <= 0) continue;

          // 转换为世界坐标
          double wx = dyn_ox + (dx + 0.5) * dyn_res;
          double wy = dyn_oy + (dy + 0.5) * dyn_res;

          // 转换为静态地图格子坐标
          int sx = static_cast<int>((wx - stat_ox) / stat_res);
          int sy = static_cast<int>((wy - stat_oy) / stat_res);

          if (sx >= 0 && sy >= 0 &&
              static_cast<unsigned int>(sx) < stat_w &&
              static_cast<unsigned int>(sy) < stat_h) {
            size_t stat_idx = static_cast<size_t>(sy) * stat_w + sx;
            uint8_t dynamic_cost = static_cast<uint8_t>(
              (dynamic_costmap_.data[idx] * 253) / 99);
            costmap_data[stat_idx] = std::max(costmap_data[stat_idx], dynamic_cost);
          }
        }
      }
    }
  }

  // 调用DWA算法
  Velocity best_vel = dwa_->computeVelocity(
    robot_pose, robot_vel, local_goal, costmap_data,
    costmap_copy.info.width, costmap_copy.info.height,
    costmap_copy.info.resolution,
    costmap_copy.info.origin.position.x,
    costmap_copy.info.origin.position.y);

  // ----------------------------------------------------------
  // 步骤5：发布速度指令（接近目标时减速）
  // ----------------------------------------------------------
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = best_vel.v * speed_scale;      // 前进速度（m/s）
  cmd_vel.angular.z = best_vel.omega * speed_scale;  // 转向速度（rad/s）
  cmd_vel_pub_->publish(cmd_vel);

  // 记录上次发布的速度，用于下次DWA计算
  last_cmd_vel_.v = best_vel.v;
  last_cmd_vel_.omega = best_vel.omega;

  // ----------------------------------------------------------
  // 步骤6：发布可视化Marker
  // ----------------------------------------------------------
  publishVisualization(robot_pose, best_vel, local_goal,
                       dwa_->getRecentTrajectories());

  RCLCPP_DEBUG(this->get_logger(),
    "DWA: 位姿(%.2f,%.2f,%.2f) 目标(%.2f,%.2f) → 速度(%.2f, %.2f)",
    robot_pose.x, robot_pose.y, robot_pose.theta,
    local_goal.x, local_goal.y, best_vel.v, best_vel.omega);
}

// ============================================================
// findLocalGoal - 在全局路径上找局部目标
// ============================================================
Pose2D DWAPlanner::findLocalGoal(const Pose2D & robot_pose) const
{
  std::lock_guard<std::mutex> lock(path_mutex_);

  Pose2D local_goal;

  if (global_path_.poses.empty()) {
    // 没有路径，目标就是当前位置（停止）
    local_goal = robot_pose;
    return local_goal;
  }

  // ----------------------------------------------------------
  // 找到路径上距离机器人最近的点
  // ----------------------------------------------------------
  // 从local_goal_index_开始搜索（避免回头找前面的点）
  double min_dist = std::numeric_limits<double>::infinity();
  int nearest_idx = local_goal_index_;

  for (size_t i = local_goal_index_; i < global_path_.poses.size(); ++i) {
    double px = global_path_.poses[i].pose.position.x;
    double py = global_path_.poses[i].pose.position.y;
    double dx = px - robot_pose.x;
    double dy = py - robot_pose.y;
    double dist = dx * dx + dy * dy;  // 用平方距离避免sqrt

    if (dist < min_dist) {
      min_dist = dist;
      nearest_idx = static_cast<int>(i);
    }
  }

  // ----------------------------------------------------------
  // 局部目标 = 最近点前方一定距离的点
  // ----------------------------------------------------------
  // 使用距离而非点数作为lookahead，避免Chaikin平滑后路径点过密导致目标太近
  double lookahead_dist = 0.5;  // 向前看0.5米
  double accum_dist = 0.0;
  int goal_idx = nearest_idx;
  for (size_t i = nearest_idx + 1; i < global_path_.poses.size(); ++i) {
    double dx = global_path_.poses[i].pose.position.x - global_path_.poses[i-1].pose.position.x;
    double dy = global_path_.poses[i].pose.position.y - global_path_.poses[i-1].pose.position.y;
    accum_dist += std::sqrt(dx * dx + dy * dy);
    if (accum_dist >= lookahead_dist) {
      goal_idx = static_cast<int>(i);
      break;
    }
    goal_idx = static_cast<int>(i);
  }

  // 更新local_goal_index_，避免下次回头找
  local_goal_index_ = nearest_idx;

  local_goal.x = global_path_.poses[goal_idx].pose.position.x;
  local_goal.y = global_path_.poses[goal_idx].pose.position.y;

  // 计算目标朝向（指向下一个路径点）
  if (goal_idx < static_cast<int>(global_path_.poses.size()) - 1) {
    double next_x = global_path_.poses[goal_idx + 1].pose.position.x;
    double next_y = global_path_.poses[goal_idx + 1].pose.position.y;
    local_goal.theta = std::atan2(next_y - local_goal.y, next_x - local_goal.x);
  } else {
    // 最后一个点，使用路径末尾的朝向
    auto & last = global_path_.poses.back();
    auto q = last.pose.orientation;
    local_goal.theta = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  return local_goal;
}

// ============================================================
// publishVisualization - 发布可视化Marker
// ============================================================
void DWAPlanner::publishVisualization(
  const Pose2D & robot_pose,
  const Velocity & best_vel,
  const Pose2D & local_goal,
  const std::vector<Trajectory> & trajectories)
{
  visualization_msgs::msg::MarkerArray markers;
  auto now = this->now();

  // ----------------------------------------------------------
  // Marker1：候选轨迹（灰色线段，最多显示50条）
  // ----------------------------------------------------------
  // DWA会评估200条候选轨迹，全部显示太密
  // 只显示一部分，让用户看到DWA在"思考"哪些路径
  size_t traj_step = std::max(trajectories.size() / 50, size_t(1));

  for (size_t i = 0; i < trajectories.size(); i += traj_step) {
    const auto & traj = trajectories[i];

    visualization_msgs::msg::Marker line;
    line.header.frame_id = "map";
    line.header.stamp = now;
    line.ns = "dwa_trajectories";
    line.id = static_cast<int>(i);
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.02;  // 线宽2cm

    // 灰色半透明，表示"候选"轨迹
    line.color.r = 0.7;
    line.color.g = 0.7;
    line.color.b = 0.7;
    line.color.a = 0.3;

    // 将轨迹点转为Marker点
    for (const auto & pose : traj.poses) {
      geometry_msgs::msg::Point p;
      p.x = pose.x;
      p.y = pose.y;
      p.z = 0.02;  // 略高于地面，避免Z-fighting
      line.points.push_back(p);
    }

    line.lifetime = rclcpp::Duration::from_seconds(0.2);
    markers.markers.push_back(line);
  }

  // ----------------------------------------------------------
  // Marker2：最优轨迹（绿色粗线）
  // ----------------------------------------------------------
  // 这是DWA最终选择的轨迹，用绿色高亮显示
  if (!trajectories.empty()) {
    // 找到得分最高的轨迹（最后一条是最近评估的）
    // 简化：使用best_vel对应的轨迹
    visualization_msgs::msg::Marker best_line;
    best_line.header.frame_id = "map";
    best_line.header.stamp = now;
    best_line.ns = "dwa_best_trajectory";
    best_line.id = 0;
    best_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    best_line.action = visualization_msgs::msg::Marker::ADD;
    best_line.pose.orientation.w = 1.0;
    best_line.scale.x = 0.05;  // 线宽5cm，比候选轨迹粗

    // 绿色高亮，表示"最优"轨迹
    best_line.color.r = 0.0;
    best_line.color.g = 1.0;
    best_line.color.b = 0.0;
    best_line.color.a = 0.8;

    // 模拟最优速度的轨迹
    Trajectory best_traj = dwa_->simulateTrajectory(robot_pose, best_vel.v, best_vel.omega);
    for (const auto & pose : best_traj.poses) {
      geometry_msgs::msg::Point p;
      p.x = pose.x;
      p.y = pose.y;
      p.z = 0.03;
      best_line.points.push_back(p);
    }

    best_line.lifetime = rclcpp::Duration::from_seconds(0.2);
    markers.markers.push_back(best_line);
  }

  // ----------------------------------------------------------
  // Marker3：局部目标点（黄色球体）
  // ----------------------------------------------------------
  // 显示DWA正在追踪的局部目标位置
  visualization_msgs::msg::Marker goal_marker;
  goal_marker.header.frame_id = "map";
  goal_marker.header.stamp = now;
  goal_marker.ns = "dwa_local_goal";
  goal_marker.id = 0;
  goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
  goal_marker.action = visualization_msgs::msg::Marker::ADD;
  goal_marker.pose.position.x = local_goal.x;
  goal_marker.pose.position.y = local_goal.y;
  goal_marker.pose.position.z = 0.1;
  goal_marker.pose.orientation.w = 1.0;
  goal_marker.scale.x = 0.2;
  goal_marker.scale.y = 0.2;
  goal_marker.scale.z = 0.2;
  goal_marker.color.r = 1.0;
  goal_marker.color.g = 1.0;
  goal_marker.color.b = 0.0;
  goal_marker.color.a = 0.8;
  goal_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  markers.markers.push_back(goal_marker);

  // ----------------------------------------------------------
  // Marker4：速度箭头（蓝色箭头）
  // ----------------------------------------------------------
  // 显示当前速度指令的方向和大小
  if (std::abs(best_vel.v) > 0.01 || std::abs(best_vel.omega) > 0.01) {
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = "map";
    arrow.header.stamp = now;
    arrow.ns = "dwa_velocity";
    arrow.id = 0;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;

    // 箭头起点：机器人位置
    geometry_msgs::msg::Point start;
    start.x = robot_pose.x;
    start.y = robot_pose.y;
    start.z = 0.15;

    // 箭头终点：机器人位置 + 速度方向 * 速度大小
    geometry_msgs::msg::Point end;
    double speed = std::sqrt(best_vel.v * best_vel.v);
    double arrow_len = speed * 2.0;  // 放大2倍方便观察
    end.x = robot_pose.x + std::cos(robot_pose.theta) * arrow_len;
    end.y = robot_pose.y + std::sin(robot_pose.theta) * arrow_len;
    end.z = 0.15;

    arrow.points.push_back(start);
    arrow.points.push_back(end);
    arrow.scale.x = 0.03;  // 箭头杆粗细
    arrow.scale.y = 0.06;  // 箭头头部宽度
    arrow.scale.z = 0.0;   // 不使用

    arrow.color.r = 0.0;
    arrow.color.g = 0.5;
    arrow.color.b = 1.0;
    arrow.color.a = 0.8;
    arrow.lifetime = rclcpp::Duration::from_seconds(0.2);
    markers.markers.push_back(arrow);
  }

  // ----------------------------------------------------------
  // Marker5：机器人轮廓（蓝色矩形）
  // ----------------------------------------------------------
  // 显示机器人的实际尺寸和朝向
  visualization_msgs::msg::Marker footprint;
  footprint.header.frame_id = "map";
  footprint.header.stamp = now;
  footprint.ns = "dwa_footprint";
  footprint.id = 0;
  footprint.type = visualization_msgs::msg::Marker::CUBE;
  footprint.action = visualization_msgs::msg::Marker::ADD;
  footprint.pose.position.x = robot_pose.x;
  footprint.pose.position.y = robot_pose.y;
  footprint.pose.position.z = 0.02;
  footprint.pose.orientation.w = std::cos(robot_pose.theta / 2.0);
  footprint.pose.orientation.z = std::sin(robot_pose.theta / 2.0);
  footprint.scale.x = 0.35;  // 机器人长度
  footprint.scale.y = 0.25;  // 机器人宽度
  footprint.scale.z = 0.04;  // 机器人高度
  footprint.color.r = 0.0;
  footprint.color.g = 0.3;
  footprint.color.b = 1.0;
  footprint.color.a = 0.4;  // 半透明
  footprint.lifetime = rclcpp::Duration::from_seconds(0.2);
  markers.markers.push_back(footprint);

  // ----------------------------------------------------------
  // 发布所有Marker
  // ----------------------------------------------------------
  marker_pub_->publish(markers);
}

}  // namespace agv_local_planner
