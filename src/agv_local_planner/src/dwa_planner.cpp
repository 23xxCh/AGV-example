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
  path_received_(false),
  local_goal_index_(0)
{
  // ----------------------------------------------------------
  // 声明和读取参数
  // ----------------------------------------------------------
  this->declare_parameter("base_frame", std::string("base_link"));
  this->declare_parameter("map_frame", std::string("map"));
  this->declare_parameter("control_frequency", 10.0);
  this->declare_parameter("costmap_topic", std::string("/map"));
  this->declare_parameter("path_topic", std::string("/planned_path"));

  base_frame_ = this->get_parameter("base_frame").as_string();
  map_frame_ = this->get_parameter("map_frame").as_string();
  control_frequency_ = this->get_parameter("control_frequency").as_double();

  // ----------------------------------------------------------
  // 加载DWA参数并创建算法实例
  // ----------------------------------------------------------
  DWAParams params = loadParams();
  dwa_ = std::make_unique<DWASearch>(params);

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

  // ----------------------------------------------------------
  // 创建发布器
  // ----------------------------------------------------------
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(10).reliable());

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
  this->declare_parameter("goal_tolerance_xy", 0.3);
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

  RCLCPP_INFO_ONCE(this->get_logger(),
    "DWA收到全局路径: %zu个路径点", msg->poses.size());
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

  // 获取机器人当前速度（从TF变化率估算，简化处理使用0）
  Velocity robot_vel(0.0, 0.0);

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

  if (dist_to_final < 0.3) {
    // 到达最终目标，停止
    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_pub_->publish(stop_cmd);
    RCLCPP_INFO(this->get_logger(), "DWA: 已到达目标！停止。");
    return;
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

  // 调用DWA算法
  Velocity best_vel = dwa_->computeVelocity(
    robot_pose, robot_vel, local_goal, costmap_data,
    costmap_copy.info.width, costmap_copy.info.height,
    costmap_copy.info.resolution,
    costmap_copy.info.origin.position.x,
    costmap_copy.info.origin.position.y);

  // ----------------------------------------------------------
  // 步骤5：发布速度指令
  // ----------------------------------------------------------
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = best_vel.v;      // 前进速度（m/s）
  cmd_vel.angular.z = best_vel.omega;  // 转向速度（rad/s）
  cmd_vel_pub_->publish(cmd_vel);

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
  // 为什么不在最近点？
  // 因为如果目标就在机器人脚下，DWA会原地打转
  // 给一个前方的目标，DWA会朝着它前进
  int lookahead = 10;  // 向前看10个路径点
  int goal_idx = std::min(nearest_idx + lookahead,
                          static_cast<int>(global_path_.poses.size()) - 1);

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

}  // namespace agv_local_planner
