/**
 * ============================================================
 * astar_planner.cpp - A*全局路径规划器ROS2节点实现
 * ============================================================
 *
 * 本文件实现AstarPlanner类，将A*搜索算法封装为ROS2节点
 * 核心流程：接收代价地图 → 缓存 → 响应规划请求 → A*搜索 → 返回路径
 */

#include "agv_global_planner/astar_planner.hpp"
#include <chrono>
#include <functional>

namespace agv_global_planner
{

// ============================================================
// 构造函数
// ============================================================
AstarPlanner::AstarPlanner(const rclcpp::NodeOptions & options)
: Node("astar_planner", options),
  costmap_received_(false)
{
  // ----------------------------------------------------------
  // 声明ROS2参数
  // ----------------------------------------------------------
  // 参数可以在启动时通过YAML文件或命令行覆盖
  // 格式：ros2 run agv_global_planner global_planner_node --ros-args -p heuristic_type:=1

  // 启发函数类型：0=曼哈顿，1=欧几里得，2=对角线(Octile，推荐)
  this->declare_parameter("heuristic_type", 2);

  // 是否允许对角线移动（推荐开启，路径更短更自然）
  this->declare_parameter("allow_diagonal", true);

  // 代价缩放因子：越大路径越远离障碍物
  this->declare_parameter("cost_factor", 1.0);

  // 致命障碍阈值：代价值>=此值的格子不可通行
  this->declare_parameter("lethal_cost", 254);

  // 代价地图话题名
  this->declare_parameter("costmap_topic", std::string("map"));

  // ----------------------------------------------------------
  // 读取参数并创建A*搜索实例
  // ----------------------------------------------------------
  int heuristic_type = this->get_parameter("heuristic_type").as_int();
  bool allow_diagonal = this->get_parameter("allow_diagonal").as_bool();
  double cost_factor = this->get_parameter("cost_factor").as_double();
  uint8_t lethal_cost = static_cast<uint8_t>(this->get_parameter("lethal_cost").as_int());

  // 根据参数选择启发函数类型
  HeuristicType h_type;
  switch (heuristic_type) {
    case 0: h_type = HeuristicType::MANHATTAN; break;
    case 1: h_type = HeuristicType::EUCLIDEAN; break;
    case 2:
    default: h_type = HeuristicType::OCTILE; break;
  }

  // 创建A*搜索实例
  astar_ = std::make_unique<AstarSearch>(h_type, allow_diagonal, cost_factor);
  astar_->setLethalCost(lethal_cost);

  RCLCPP_INFO(this->get_logger(),
    "A*规划器初始化: 启发函数=%d, 对角线=%d, 代价因子=%.2f, 致命阈值=%d",
    heuristic_type, allow_diagonal, cost_factor, lethal_cost);

  // ----------------------------------------------------------
  // 创建订阅器：订阅代价地图
  // ----------------------------------------------------------
  // QoS配置：
  // - keep_last(1): 只保留最新1帧地图（地图可能很大，不需要缓存多帧）
  // - reliable: 保证消息可靠送达
  // - transient_local: latched，确保规划器启动时就能收到地图
  //   （即使地图在规划器之前就发布了）
  std::string costmap_topic = this->get_parameter("costmap_topic").as_string();
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    costmap_topic, qos,
    std::bind(&AstarPlanner::costmapCallback, this, std::placeholders::_1));

  // ----------------------------------------------------------
  // 创建服务服务器：响应路径规划请求
  // ----------------------------------------------------------
  plan_service_ = this->create_service<agv_interfaces::srv::PathPlan>(
    "plan_path",
    std::bind(&AstarPlanner::handlePathPlanRequest, this,
              std::placeholders::_1, std::placeholders::_2));

  // ----------------------------------------------------------
  // 创建发布器：发布规划路径
  // ----------------------------------------------------------
  // 路径使用reliable QoS，确保可视化端收到完整路径
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "planned_path", rclcpp::QoS(10).reliable());

  path_with_cost_pub_ = this->create_publisher<agv_interfaces::msg::PathWithCost>(
    "planned_path_with_cost", rclcpp::QoS(10).reliable());

  RCLCPP_INFO(this->get_logger(), "A*全局路径规划器节点启动完成！");
}

// ============================================================
// costmapCallback - 代价地图回调
// ============================================================
void AstarPlanner::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  // 使用互斥锁保护代价地图的写入
  // 为什么需要互斥锁？
  // - 代价地图在订阅回调线程中更新
  // - 规划服务在服务回调线程中读取
  // - 两个线程可能同时访问，需要互斥锁防止数据竞争
  // - 数据竞争会导致：读到半新半旧的数据，路径计算错误
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  cached_costmap_ = *msg;
  costmap_received_ = true;

  RCLCPP_INFO_ONCE(this->get_logger(),
    "收到代价地图: %dx%d, 分辨率=%.3f m/像素",
    msg->info.width, msg->info.height, msg->info.resolution);
}

// ============================================================
// handlePathPlanRequest - 处理路径规划请求
// ============================================================
void AstarPlanner::handlePathPlanRequest(
  const std::shared_ptr<agv_interfaces::srv::PathPlan::Request> request,
  std::shared_ptr<agv_interfaces::srv::PathPlan::Response> response)
{
  // ----------------------------------------------------------
  // 检查是否已收到代价地图
  // ----------------------------------------------------------
  if (!costmap_received_) {
    response->success = false;
    response->error_msg = "尚未收到代价地图，无法规划路径";
    RCLCPP_WARN(this->get_logger(), "规划请求被拒绝: %s", response->error_msg.c_str());
    return;
  }

  // 获取代价地图的线程安全副本
  // 使用互斥锁确保读取时不被其他线程修改
  nav_msgs::msg::OccupancyGrid costmap_copy;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    costmap_copy = cached_costmap_;
  }

  // ----------------------------------------------------------
  // 坐标转换：世界坐标 → 格子坐标
  // ----------------------------------------------------------
  // 用户/调度器发送的是世界坐标（米），A*算法需要格子坐标（整数）
  // 必须做这个转换

  GridCell start_cell, goal_cell;

  if (request->use_current_pose) {
    // 使用当前位置作为起点
    // TODO: 后续通过TF获取AGV当前位置
    // 目前暂时使用请求中的start坐标
    start_cell = worldToGrid(request->start, costmap_copy.info);
  } else {
    start_cell = worldToGrid(request->start, costmap_copy.info);
  }

  goal_cell = worldToGrid(request->goal, costmap_copy.info);

  RCLCPP_INFO(this->get_logger(),
    "路径规划请求: 起点(%.2f,%.2f)→格子(%d,%d), 终点(%.2f,%.2f)→格子(%d,%d)",
    request->start.x, request->start.y, start_cell.x, start_cell.y,
    request->goal.x, request->goal.y, goal_cell.x, goal_cell.y);

  // ----------------------------------------------------------
  // 执行A*搜索
  // ----------------------------------------------------------
  // OccupancyGrid的值是int8(-128~127)
  // 需要转换为uint8(0~255)给A*搜索使用
  // 转换规则：
  //   -1(未知) → 255(致命障碍，不可通行)
  //   0(自由) → 0(自由)
  //   100(障碍) → 254(外切障碍)
  //   1~99 → 按比例映射到 1~253
  std::vector<uint8_t> costmap_data(costmap_copy.data.size());
  for (size_t i = 0; i < costmap_copy.data.size(); ++i) {
    int8_t val = costmap_copy.data[i];
    if (val < 0) {
      // 未知区域视为致命障碍（保守策略：未知=危险）
      costmap_data[i] = 255;
    } else if (val == 0) {
      // 自由区域
      costmap_data[i] = 0;
    } else if (val == 100) {
      // 障碍物映射为外切障碍
      costmap_data[i] = 254;
    } else {
      // 1~99按比例映射到1~253
      costmap_data[i] = static_cast<uint8_t>((val * 253) / 99);
    }
  }

  // 开始计时
  auto start_time = std::chrono::high_resolution_clock::now();

  // 执行A*搜索
  std::vector<GridCell> grid_path;
  bool found = astar_->search(
    costmap_data,
    costmap_copy.info.width,
    costmap_copy.info.height,
    start_cell,
    goal_cell,
    grid_path);

  // 结束计时
  auto end_time = std::chrono::high_resolution_clock::now();
  double planning_time_ms = std::chrono::duration<double, std::milli>(
    end_time - start_time).count();

  // ----------------------------------------------------------
  // 构建响应
  // ----------------------------------------------------------
  if (found) {
    // 规划成功：将格子路径转换为世界坐标路径
    nav_msgs::msg::Path raw_path = gridPathToPathMsg(grid_path, costmap_copy.info);

    // 路径平滑处理
    nav_msgs::msg::Path path_msg = smoothPath(raw_path, costmap_copy, 3);

    response->success = true;
    response->path = path_msg;
    response->cost = static_cast<double>(grid_path.size());  // 简化：用格子数作为代价
    response->error_msg = "";

    // 发布路径到话题（供RViz2可视化）
    path_pub_->publish(path_msg);

    // 发布带代价的路径
    agv_interfaces::msg::PathWithCost path_with_cost;
    path_with_cost.header = path_msg.header;
    path_with_cost.path = path_msg;
    path_with_cost.total_cost = response->cost;
    path_with_cost.planning_time_ms = planning_time_ms;
    // 计算每个点的累积代价
    path_with_cost.costs.resize(grid_path.size(), 0.0);
    path_with_cost_pub_->publish(path_with_cost);

    RCLCPP_INFO(this->get_logger(),
      "路径规划成功! 路径长度=%zu格子, 耗时=%.2fms",
      grid_path.size(), planning_time_ms);
  } else {
    // 规划失败
    response->success = false;
    response->cost = 0.0;
    response->error_msg = "未找到可行路径（起点或终点不可达）";

    RCLCPP_WARN(this->get_logger(),
      "路径规划失败! 耗时=%.2fms: %s",
      planning_time_ms, response->error_msg.c_str());
  }
}

// ============================================================
// gridToWorld - 格子坐标转世界坐标
// ============================================================
geometry_msgs::msg::Point AstarPlanner::gridToWorld(
  const GridCell & cell,
  const nav_msgs::msg::MapMetaData & map_info) const
{
  geometry_msgs::msg::Point world_point;

  // 转换公式：
  //   world_x = origin_x + (cell_x + 0.5) * resolution
  //   world_y = origin_y + (cell_y + 0.5) * resolution
  //
  // 为什么 +0.5？
  // 因为格子坐标(cell_x, cell_y)代表格子的左下角
  // 格子的中心点在 (cell_x+0.5, cell_y+0.5) 处
  // 我们想让路径点在格子中心，而不是格子边缘
  //
  // 图示：
  //   ┌────────┐
  //   │   ×    │  × 是格子中心，坐标=(cell_x+0.5, cell_y+0.5)
  //   │ (3,2)  │  (3,2) 是格子的左下角
  //   └────────┘
  world_point.x = map_info.origin.position.x + (cell.x + 0.5) * map_info.resolution;
  world_point.y = map_info.origin.position.y + (cell.y + 0.5) * map_info.resolution;
  world_point.z = 0.0;  // 2D地图，Z坐标为0

  return world_point;
}

// ============================================================
// worldToGrid - 世界坐标转格子坐标
// ============================================================
GridCell AstarPlanner::worldToGrid(
  const geometry_msgs::msg::Point & world_point,
  const nav_msgs::msg::MapMetaData & map_info) const
{
  GridCell cell;

  // 反向转换公式：
  //   cell_x = floor((world_x - origin_x) / resolution)
  //   cell_y = floor((world_y - origin_y) / resolution)
  //
  // floor向下取整：确保世界坐标落在正确的格子内
  // 例如：world_x=1.153, origin_x=1.0, resolution=0.05
  //       cell_x = floor(0.153/0.05) = floor(3.06) = 3
  cell.x = static_cast<int>(std::floor(
    (world_point.x - map_info.origin.position.x) / map_info.resolution));
  cell.y = static_cast<int>(std::floor(
    (world_point.y - map_info.origin.position.y) / map_info.resolution));

  return cell;
}

// ============================================================
// gridPathToPathMsg - 格子路径转ROS2 Path消息
// ============================================================
nav_msgs::msg::Path AstarPlanner::gridPathToPathMsg(
  const std::vector<GridCell> & grid_path,
  const nav_msgs::msg::MapMetaData & map_info) const
{
  nav_msgs::msg::Path path_msg;

  // 设置消息头
  path_msg.header.stamp = this->now();
  path_msg.header.frame_id = "map";  // 路径在map坐标系下

  // 将每个格子转换为PoseStamped
  // PoseStamped = Header + Pose
  // Pose = Position(x,y,z) + Orientation(四元数)
  path_msg.poses.resize(grid_path.size());

  for (size_t i = 0; i < grid_path.size(); ++i) {
    // 设置每个路径点的Header
    path_msg.poses[i].header.stamp = this->now();
    path_msg.poses[i].header.frame_id = "map";

    // 设置位置（格子→世界坐标）
    geometry_msgs::msg::Point world_pt = gridToWorld(grid_path[i], map_info);
    path_msg.poses[i].pose.position.x = world_pt.x;
    path_msg.poses[i].pose.position.y = world_pt.y;
    path_msg.poses[i].pose.position.z = 0.0;

    // 设置朝向（根据路径方向计算）
    // 路径中每个点的朝向 = 当前点指向下一个点的方向
    // 最后一个点保持倒数第二个点的朝向
    double yaw = 0.0;
    if (i < grid_path.size() - 1) {
      // 计算当前点指向下一个点的角度
      // atan2(dy, dx) 返回 (-PI, PI] 范围的角度
      double dx = static_cast<double>(grid_path[i + 1].x - grid_path[i].x);
      double dy = static_cast<double>(grid_path[i + 1].y - grid_path[i].y);
      yaw = std::atan2(dy, dx);
    } else if (i > 0) {
      // 最后一个点：使用前一段的方向
      double dx = static_cast<double>(grid_path[i].x - grid_path[i - 1].x);
      double dy = static_cast<double>(grid_path[i].y - grid_path[i - 1].y);
      yaw = std::atan2(dy, dx);
    }

    // 将yaw角转换为四元数
    // 四元数公式（绕Z轴旋转yaw角）：
    //   w = cos(yaw/2)
    //   x = 0 (不绕X轴旋转)
    //   y = 0 (不绕Y轴旋转)
    //   z = sin(yaw/2)
    path_msg.poses[i].pose.orientation.w = std::cos(yaw / 2.0);
    path_msg.poses[i].pose.orientation.x = 0.0;
    path_msg.poses[i].pose.orientation.y = 0.0;
    path_msg.poses[i].pose.orientation.z = std::sin(yaw / 2.0);
  }

  return path_msg;
}

// ============================================================
// smoothPath - Chaikin角切路径平滑
// ============================================================
nav_msgs::msg::Path AstarPlanner::smoothPath(
  const nav_msgs::msg::Path & path,
  const nav_msgs::msg::OccupancyGrid & costmap,
  int iterations) const
{
  if (path.poses.size() < 3) {
    return path;  // 点太少，无法平滑
  }

  // 提取路径点为简单的(x,y)列表
  struct Point2D { double x, y; };
  std::vector<Point2D> points;
  points.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    points.push_back({pose.pose.position.x, pose.pose.position.y});
  }

  // Chaikin角切迭代
  for (int iter = 0; iter < iterations; ++iter) {
    std::vector<Point2D> new_points;
    new_points.reserve(points.size() * 2);

    // 保留起点
    new_points.push_back(points.front());

    for (size_t i = 0; i < points.size() - 1; ++i) {
      const auto & p1 = points[i];
      const auto & p2 = points[i + 1];

      // 在25%和75%处生成新点
      Point2D q, r;
      q.x = p1.x + 0.25 * (p2.x - p1.x);
      q.y = p1.y + 0.25 * (p2.y - p1.y);
      r.x = p1.x + 0.75 * (p2.x - p1.x);
      r.y = p1.y + 0.75 * (p2.y - p1.y);

      new_points.push_back(q);
      new_points.push_back(r);
    }

    // 保留终点
    new_points.push_back(points.back());

    points = std::move(new_points);
  }

  // 碰撞检测：检查平滑后的路径是否安全
  // 如果某个点在障碍物上，回退到该段的中点
  double resolution = costmap.info.resolution;
  double origin_x = costmap.info.origin.position.x;
  double origin_y = costmap.info.origin.position.y;
  unsigned int width = costmap.info.width;
  unsigned int height = costmap.info.height;

  auto isSafe = [&](double wx, double wy) -> bool {
    int gx = static_cast<int>((wx - origin_x) / resolution);
    int gy = static_cast<int>((wy - origin_y) / resolution);
    if (gx < 0 || gy < 0 ||
        static_cast<unsigned int>(gx) >= width ||
        static_cast<unsigned int>(gy) >= height) {
      return false;
    }
    size_t idx = static_cast<size_t>(gy) * width + gx;
    int8_t val = costmap.data[idx];
    return (val >= 0 && val < 80);  // 80以下视为安全
  };

  // 验证平滑路径，不安全的点保留原始位置
  for (size_t i = 1; i < points.size() - 1; ++i) {
    if (!isSafe(points[i].x, points[i].y)) {
      // 找到最近的原始路径点作为替代
      double min_dist = std::numeric_limits<double>::infinity();
      size_t nearest = 0;
      for (size_t j = 0; j < path.poses.size(); ++j) {
        double dx = points[i].x - path.poses[j].pose.position.x;
        double dy = points[i].y - path.poses[j].pose.position.y;
        double d = dx * dx + dy * dy;
        if (d < min_dist) {
          min_dist = d;
          nearest = j;
        }
      }
      points[i].x = path.poses[nearest].pose.position.x;
      points[i].y = path.poses[nearest].pose.position.y;
    }
  }

  // 构建平滑后的Path消息
  nav_msgs::msg::Path smoothed;
  smoothed.header = path.header;
  smoothed.poses.resize(points.size());

  for (size_t i = 0; i < points.size(); ++i) {
    smoothed.poses[i].header = path.header;
    smoothed.poses[i].pose.position.x = points[i].x;
    smoothed.poses[i].pose.position.y = points[i].y;
    smoothed.poses[i].pose.position.z = 0.0;

    // 计算朝向
    double yaw = 0.0;
    if (i < points.size() - 1) {
      double dx = points[i + 1].x - points[i].x;
      double dy = points[i + 1].y - points[i].y;
      yaw = std::atan2(dy, dx);
    } else if (i > 0) {
      double dx = points[i].x - points[i - 1].x;
      double dy = points[i].y - points[i - 1].y;
      yaw = std::atan2(dy, dx);
    }
    smoothed.poses[i].pose.orientation.w = std::cos(yaw / 2.0);
    smoothed.poses[i].pose.orientation.z = std::sin(yaw / 2.0);
  }

  return smoothed;
}

}  // namespace agv_global_planner
