/**
 * ============================================================
 * astar_planner.hpp - A*全局路径规划器ROS2封装
 * ============================================================
 *
 * 【模块概述】
 * AstarPlanner将A*搜索算法包装为ROS2节点，提供：
 * 1. 订阅代价地图话题，实时获取环境信息
 * 2. 提供路径规划服务（PathPlan srv），响应规划请求
 * 3. 发布规划路径话题，供可视化和其他模块使用
 *
 * 【节点通信架构】
 *
 *   ┌──────────┐    /costmap     ┌───────────────┐
 *   │ 代价地图  │ ─────────────→ │               │
 *   │  节点     │                │  AstarPlanner  │
 *   └──────────┘                │    节点        │
 *                                │               │
 *   ┌──────────┐  PathPlan.srv  │               │  /planned_path  ┌───────┐
 *   │  调度器   │ ←───────────→ │               │ ─────────────→ │ RViz2 │
 *   │  节点     │  (请求/响应)   │               │  (Path消息)    │ 可视化│
 *   └──────────┘                └───────────────┘                └───────┘
 *
 * 【数据流】
 * 1. 代价地图节点持续发布Costmap2D → AstarPlanner缓存最新地图
 * 2. 调度器发送PathPlan请求（起点+终点）→ AstarPlanner执行A*搜索
 * 3. AstarPlanner返回PathPlan响应（路径+代价）→ 调度器收到结果
 * 4. 同时，AstarPlanner发布路径到话题 → RViz2可视化显示
 *
 * 【Costmap2D vs OccupancyGrid】
 * - OccupancyGrid: 静态地图，0/100/-1三种值
 * - Costmap2D: 动态代价地图，0~255连续值，包含障碍物膨胀层
 * - 膨胀层的作用：障碍物周围的格子代价逐渐递增
 *   ┌───┬───┬───┬───┬───┐
 *   │ 0 │ 0 │ 0 │ 0 │ 0 │  远离障碍物，代价=0
 *   ├───┼───┼───┼───┼───┤
 *   │ 0 │10 │20 │10 │ 0 │  接近障碍物，代价渐增
 *   ├───┼───┼───┼───┼───┤
 *   │ 0 │20 │254│20 │ 0 │  254=外切障碍，紧贴障碍物
 *   ├───┼───┼───┼───┼───┤
 *   │ 0 │10 │20 │10 │ 0 │
 *   └───┴───┴───┴───┴───┘
 * - 膨胀层使AGV自动远离障碍物，不需要额外加安全距离
 */

#ifndef AGV_GLOBAL_PLANNER__ASTAR_PLANNER_HPP_
#define AGV_GLOBAL_PLANNER__ASTAR_PLANNER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "agv_global_planner/astar_search.hpp"
#include "agv_interfaces/srv/path_plan.hpp"
#include "agv_interfaces/msg/path_with_cost.hpp"

namespace agv_global_planner
{

class AstarPlanner : public rclcpp::Node
{
public:
  explicit AstarPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * 代价地图回调函数
   * 当收到新的代价地图时触发，缓存最新地图数据
   */
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  /**
   * 路径规划服务回调
   * 收到规划请求后执行A*搜索，返回路径结果
   */
  void handlePathPlanRequest(
    const std::shared_ptr<agv_interfaces::srv::PathPlan::Request> request,
    std::shared_ptr<agv_interfaces::srv::PathPlan::Response> response);

  /**
   * 将格子坐标转换为世界坐标
   * @param cell 格子坐标
   * @param map_info 地图元数据
   * @return 世界坐标（米）
   *
   * 转换公式：
   *   world_x = origin_x + cell.x * resolution
   *   world_y = origin_y + cell.y * resolution
   *
   * 图示：
   *   格子坐标(3,2) + 分辨率0.05 + 原点(1.0,0.5)
   *   → 世界坐标(1.0 + 3×0.05, 0.5 + 2×0.05) = (1.15, 0.60)
   */
  geometry_msgs::msg::Point gridToWorld(
    const GridCell & cell,
    const nav_msgs::msg::MapMetaData & map_info) const;

  /**
   * 将世界坐标转换为格子坐标
   * @param world_point 世界坐标点
   * @param map_info 地图元数据
   * @return 格子坐标
   *
   * 转换公式（反向）：
   *   cell_x = floor((world_x - origin_x) / resolution)
   *   cell_y = floor((world_y - origin_y) / resolution)
   *
   * floor: 向下取整，确保落在正确的格子内
   * 例如：world_x=1.153, origin_x=1.0, resolution=0.05
   *       cell_x = floor(0.153/0.05) = floor(3.06) = 3
   */
  GridCell worldToGrid(
    const geometry_msgs::msg::Point & world_point,
    const nav_msgs::msg::MapMetaData & map_info) const;

  /**
   * 将格子路径转换为ROS2 Path消息
   * @param grid_path 格子坐标路径
   * @param map_info 地图元数据
   * @return nav_msgs/Path消息（包含一系列PoseStamped）
   */
  nav_msgs::msg::Path gridPathToPathMsg(
    const std::vector<GridCell> & grid_path,
    const nav_msgs::msg::MapMetaData & map_info) const;

  // ---- 成员变量 ----

  // A*搜索算法实例
  std::unique_ptr<AstarSearch> astar_;

  // 缓存的代价地图数据
  nav_msgs::msg::OccupancyGrid cached_costmap_;
  bool costmap_received_;  // 是否已收到代价地图
  std::mutex costmap_mutex_;  // 互斥锁，保护代价地图的并发访问

  // 订阅器：订阅代价地图
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;

  // 服务服务器：响应路径规划请求
  rclcpp::Service<agv_interfaces::srv::PathPlan>::SharedPtr plan_service_;

  // 发布器：发布规划出的路径（供可视化和其他节点使用）
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<agv_interfaces::msg::PathWithCost>::SharedPtr path_with_cost_pub_;
};

}  // namespace agv_global_planner

#endif  // AGV_GLOBAL_PLANNER__ASTAR_PLANNER_HPP_
