/**
 * ============================================================
 * dwa_planner.hpp - DWA局部规划器ROS2节点头文件
 * ============================================================
 *
 * 【节点通信架构】
 *
 *   ┌──────────┐    /map      ┌─────────────────┐
 *   │ 地图服务器│ ──────────→ │                 │
 *   └──────────┘              │   DWAPlanner    │
 *                             │    节点         │
 *   ┌──────────┐  /planned    │                 │  /cmd_vel  ┌──────┐
 *   │ A*规划器  │ ──path───→  │                 │ ────────→ │ AGV  │
 *   └──────────┘              │                 │           └──────┘
 *                             └─────────────────┘
 *
 * 【数据流】
 * 1. 地图服务器发布代价地图 → DWAPlanner订阅并缓存
 * 2. A*规划器发布全局路径 → DWAPlanner订阅并存储
 * 3. TF查询机器人当前位姿（map → base_link）
 * 4. DWA算法计算最优速度 → 发布到 /cmd_vel
 *
 * 【为什么需要TF？】
 * - 机器人在地图中的位置不是直接发布的，而是通过TF坐标变换
 * - TF就像一个"坐标簿"，记录了各个坐标系之间的关系
 * - "map → base_link" 变换告诉我们：机器人在地图中的位姿
 * - base_link 是机器人本体的坐标系原点（通常在底盘中心）
 */

#ifndef AGV_LOCAL_PLANNER__DWA_PLANNER_HPP_
#define AGV_LOCAL_PLANNER__DWA_PLANNER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "agv_local_planner/dwa_search.hpp"

namespace agv_local_planner
{

class DWAPlanner : public rclcpp::Node
{
public:
  explicit DWAPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * 代价地图回调
   * 收到新地图时更新缓存
   */
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  /**
   * 全局路径回调
   * 收到新的全局路径时存储，并找到路径上距离机器人最近的点作为局部目标
   */
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

  /**
   * 动态障碍物代价地图回调
   * 收到动态障碍物的代价地图后缓存，在控制循环中与静态地图合并
   */
  void dynamicCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  /**
   * 控制定时器回调
   * 以固定频率（如10Hz）执行DWA计算，发布速度指令
   *
   * 这是整个节点的"心跳"，每个周期：
   * 1. 检查是否收到地图和路径
   * 2. 查询TF获取机器人位姿
   * 3. 找到路径上的局部目标点
   * 4. 调用DWA算法计算最优速度
   * 5. 发布速度指令
   */
  void controlTimerCallback();

  /**
   * 在全局路径上找到距离机器人最近的点作为局部目标
   *
   * 为什么不直接用全局路径的终点？
   * 因为全局路径可能很长，DWA应该只关注"眼前"的目标
   * 这样DWA才能灵活地绕开障碍物，而不是试图直接到达终点
   *
   * @param robot_pose 机器人当前位姿
   * @return 局部目标点
   */
  Pose2D findLocalGoal(const Pose2D & robot_pose) const;

  /**
   * 读取DWA参数
   */
  DWAParams loadParams();

  /**
   * 发布可视化Marker
   * 包括：候选轨迹、局部目标、速度箭头、机器人轮廓
   */
  void publishVisualization(
    const Pose2D & robot_pose,
    const Velocity & best_vel,
    const Pose2D & local_goal,
    const std::vector<Trajectory> & trajectories);

  // ---- 成员变量 ----

  // DWA算法实例
  std::unique_ptr<DWASearch> dwa_;

  // 代价地图缓存
  nav_msgs::msg::OccupancyGrid cached_costmap_;
  bool costmap_received_;
  mutable std::mutex costmap_mutex_;  // mutable允许在const方法中加锁

  // 动态障碍物代价地图缓存
  nav_msgs::msg::OccupancyGrid dynamic_costmap_;
  bool dynamic_costmap_received_;
  mutable std::mutex dynamic_costmap_mutex_;

  // 全局路径缓存
  nav_msgs::msg::Path global_path_;
  bool path_received_;
  mutable std::mutex path_mutex_;
  mutable int local_goal_index_;      // mutable允许在const方法中更新

  // TF：用于查询机器人在地图中的位姿
  // tf2_ros::Buffer 存储所有坐标变换
  // tf2_ros::TransformListener 监听并更新变换
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 订阅器
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr dynamic_costmap_sub_;

  // 发布器：速度指令
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // 发布器：可视化Marker
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // 控制定时器
  rclcpp::TimerBase::SharedPtr control_timer_;

  // 参数
  std::string base_frame_;   // 机器人本体坐标系名（如 "base_link"）
  std::string map_frame_;    // 地图坐标系名（如 "map"）
  double control_frequency_; // 控制频率（Hz）

  // 上次发布的速度（用于DWA动态窗口计算）
  Velocity last_cmd_vel_{0.0, 0.0};
};

}  // namespace agv_local_planner

#endif  // AGV_LOCAL_PLANNER__DWA_PLANNER_HPP_
