/**
 * ============================================================
 * obstacle_manager_node.cpp - 动态障碍物管理器
 * ============================================================
 *
 * 【功能】
 * 在仓库场景中生成和管理动态障碍物（叉车、行人、货架推车等）
 * 每个障碍物沿预设路径移动，模拟真实仓库环境
 *
 * 【数据流】
 *   ObstacleManager → /dynamic_obstacles (MarkerArray) → RViz2可视化
 *                    → /dynamic_obstacles_info (自定义) → DWA避障
 *
 * 【障碍物类型】
 * 1. 行人 - 小型，速度慢，随机方向
 * 2. 叉车 - 中型，速度中等，沿固定路线
 * 3. 货架推车 - 大型，速度慢，直线移动
 */

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <random>
#include <vector>
#include <cmath>
#include <algorithm>

namespace agv_simulator
{

// 障碍物类型
enum class ObstacleType {
  PEDESTRIAN,   // 行人 - 小型慢速
  FORKLIFT,     // 叉车 - 中型中速
  CART          // 货架推车 - 大型慢速
};

// 单个动态障碍物的状态
struct DynamicObstacle {
  int id;                  // 唯一标识
  ObstacleType type;       // 类型
  double x, y;             // 当前位置
  double theta;            // 朝向
  double vx, vy;           // 速度分量
  double radius;           // 碰撞半径（米）
  double speed;            // 移动速度（米/秒）

  // 路径点（障碍物沿路径点循环移动）
  std::vector<geometry_msgs::msg::Point> waypoints;
  int current_waypoint;    // 当前目标路径点索引
};

class ObstacleManager : public rclcpp::Node
{
public:
  ObstacleManager()
  : Node("obstacle_manager"), rng_(42)
  {
    // ----------------------------------------------------------
    // 参数声明
    // ----------------------------------------------------------
    this->declare_parameter("num_pedestrians", 3);      // 行人数量
    this->declare_parameter("num_forklifts", 2);        // 叉车数量
    this->declare_parameter("num_carts", 1);            // 推车数量
    this->declare_parameter("update_rate", 10.0);       // 更新频率
    this->declare_parameter("map_width", 3.5);          // 地图宽度(米)
    this->declare_parameter("map_height", 2.5);         // 地图高度(米)
    this->declare_parameter("publish_markers", true);   // 是否发布可视化

    int num_ped = this->get_parameter("num_pedestrians").as_int();
    int num_fork = this->get_parameter("num_forklifts").as_int();
    int num_cart = this->get_parameter("num_carts").as_int();
    double rate = this->get_parameter("update_rate").as_double();
    map_width_ = this->get_parameter("map_width").as_double();
    map_height_ = this->get_parameter("map_height").as_double();
    bool pub_markers = this->get_parameter("publish_markers").as_bool();

    // ----------------------------------------------------------
    // 创建发布器
    // ----------------------------------------------------------
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "dynamic_obstacles", rclcpp::QoS(10).reliable());

    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "dynamic_costmap", rclcpp::QoS(1).reliable().transient_local());

    // ----------------------------------------------------------
    // 生成障碍物
    // ----------------------------------------------------------
    int id = 0;
    for (int i = 0; i < num_ped; ++i) {
      obstacles_.push_back(createObstacle(id++, ObstacleType::PEDESTRIAN));
    }
    for (int i = 0; i < num_fork; ++i) {
      obstacles_.push_back(createObstacle(id++, ObstacleType::FORKLIFT));
    }
    for (int i = 0; i < num_cart; ++i) {
      obstacles_.push_back(createObstacle(id++, ObstacleType::CART));
    }

    RCLCPP_INFO(this->get_logger(),
      "动态障碍物管理器启动: 行人=%d, 叉车=%d, 推车=%d, 共=%zu个",
      num_ped, num_fork, num_cart, obstacles_.size());

    // ----------------------------------------------------------
    // 创建更新定时器
    // ----------------------------------------------------------
    auto period = std::chrono::duration<double>(1.0 / rate);
    update_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this, pub_markers]() {
        updateObstacles();
        if (pub_markers) {
          publishMarkers();
        }
        publishDynamicCostmap();
      });
  }

private:
  // ============================================================
  // createObstacle - 创建一个动态障碍物
  // ============================================================
  DynamicObstacle createObstacle(int id, ObstacleType type)
  {
    DynamicObstacle obs;
    obs.id = id;
    obs.type = type;
    obs.current_waypoint = 0;

    std::uniform_real_distribution<double> dist_x(0.3, map_width_ - 0.3);
    std::uniform_real_distribution<double> dist_y(0.3, map_height_ - 0.3);

    // 根据类型设置属性
    switch (type) {
      case ObstacleType::PEDESTRIAN:
        obs.radius = 0.15;   // 15cm半径
        obs.speed = 0.3;     // 0.3m/s 步行速度
        // 行人生成2-4个随机路径点
        generateRandomWaypoints(obs, 2 + rng_() % 3);
        break;

      case ObstacleType::FORKLIFT:
        obs.radius = 0.3;    // 30cm半径
        obs.speed = 0.5;     // 0.5m/s 叉车速度
        // 叉车沿固定路线（仓库通道）
        generateForkliftWaypoints(obs);
        break;

      case ObstacleType::CART:
        obs.radius = 0.25;   // 25cm半径
        obs.speed = 0.2;     // 0.2m/s 推车速度
        // 推车沿直线来回
        generateCartWaypoints(obs);
        break;
    }

    // 初始位置设为第一个路径点
    if (!obs.waypoints.empty()) {
      obs.x = obs.waypoints[0].x;
      obs.y = obs.waypoints[0].y;
    } else {
      obs.x = dist_x(rng_);
      obs.y = dist_y(rng_);
    }

    obs.theta = 0.0;
    obs.vx = 0.0;
    obs.vy = 0.0;

    return obs;
  }

  // ============================================================
  // generateRandomWaypoints - 生成随机路径点
  // ============================================================
  void generateRandomWaypoints(DynamicObstacle & obs, int count)
  {
    std::uniform_real_distribution<double> dist_x(0.3, map_width_ - 0.3);
    std::uniform_real_distribution<double> dist_y(0.3, map_height_ - 0.3);

    for (int i = 0; i < count; ++i) {
      geometry_msgs::msg::Point p;
      p.x = dist_x(rng_);
      p.y = dist_y(rng_);
      p.z = 0.0;
      obs.waypoints.push_back(p);
    }
  }

  // ============================================================
  // generateForkliftWaypoints - 生成叉车路径（仓库通道）
  // ============================================================
  void generateForkliftWaypoints(DynamicObstacle & obs)
  {
    // 叉车在仓库通道中来回移动
    std::uniform_real_distribution<double> channel_dist(0.4, map_height_ - 0.4);
    double y = channel_dist(rng_);

    // 从左到右再回来
    geometry_msgs::msg::Point p1, p2;
    p1.x = 0.4;  p1.y = y;  p1.z = 0.0;
    p2.x = map_width_ - 0.4;  p2.y = y;  p2.z = 0.0;

    obs.waypoints.push_back(p1);
    obs.waypoints.push_back(p2);
  }

  // ============================================================
  // generateCartWaypoints - 生成推车路径（直线来回）
  // ============================================================
  void generateCartWaypoints(DynamicObstacle & obs)
  {
    std::uniform_real_distribution<double> dist_x(0.4, map_width_ - 0.4);
    std::uniform_real_distribution<double> dist_y(0.4, map_height_ - 0.4);

    double x1 = dist_x(rng_);
    double y1 = dist_y(rng_);
    double x2 = dist_x(rng_);
    double y2 = dist_y(rng_);

    geometry_msgs::msg::Point p1, p2;
    p1.x = x1;  p1.y = y1;  p1.z = 0.0;
    p2.x = x2;  p2.y = y2;  p2.z = 0.0;

    obs.waypoints.push_back(p1);
    obs.waypoints.push_back(p2);
  }

  // ============================================================
  // updateObstacles - 更新所有障碍物位置
  // ============================================================
  void updateObstacles()
  {
    double dt = 0.1;  // 10Hz更新

    for (auto & obs : obstacles_) {
      if (obs.waypoints.empty()) continue;

      // 获取当前目标路径点
      auto & target = obs.waypoints[obs.current_waypoint];

      // 计算到目标的方向
      double dx = target.x - obs.x;
      double dy = target.y - obs.y;
      double dist = std::sqrt(dx * dx + dy * dy);

      if (dist < 0.1) {
        // 到达目标，切换到下一个路径点
        obs.current_waypoint =
          (obs.current_waypoint + 1) % static_cast<int>(obs.waypoints.size());
        continue;
      }

      // 归一化方向并乘以速度
      obs.vx = (dx / dist) * obs.speed;
      obs.vy = (dy / dist) * obs.speed;

      // 更新位置
      obs.x += obs.vx * dt;
      obs.y += obs.vy * dt;

      // 更新朝向
      obs.theta = std::atan2(obs.vy, obs.vx);

      // 边界约束
      obs.x = std::clamp(obs.x, obs.radius, map_width_ - obs.radius);
      obs.y = std::clamp(obs.y, obs.radius, map_height_ - obs.radius);
    }
  }

  // ============================================================
  // publishMarkers - 发布可视化Marker
  // ============================================================
  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray markers;
    auto now = this->now();

    for (const auto & obs : obstacles_) {
      // 障碍物主体（圆柱体）
      visualization_msgs::msg::Marker cylinder;
      cylinder.header.frame_id = "map";
      cylinder.header.stamp = now;
      cylinder.ns = "dynamic_obstacles";
      cylinder.id = obs.id;
      cylinder.type = visualization_msgs::msg::Marker::CYLINDER;
      cylinder.action = visualization_msgs::msg::Marker::ADD;

      cylinder.pose.position.x = obs.x;
      cylinder.pose.position.y = obs.y;
      cylinder.pose.position.z = 0.15;
      cylinder.pose.orientation.w = 1.0;

      cylinder.scale.x = obs.radius * 2;
      cylinder.scale.y = obs.radius * 2;
      cylinder.scale.z = 0.3;

      // 根据类型设置颜色
      switch (obs.type) {
        case ObstacleType::PEDESTRIAN:
          cylinder.color.r = 1.0;  // 红色 - 行人
          cylinder.color.g = 0.3;
          cylinder.color.b = 0.3;
          break;
        case ObstacleType::FORKLIFT:
          cylinder.color.r = 1.0;  // 橙色 - 叉车
          cylinder.color.g = 0.6;
          cylinder.color.b = 0.0;
          break;
        case ObstacleType::CART:
          cylinder.color.r = 0.8;  // 紫色 - 推车
          cylinder.color.g = 0.2;
          cylinder.color.b = 0.8;
          break;
      }
      cylinder.color.a = 0.8;

      cylinder.lifetime = rclcpp::Duration::from_seconds(0.3);
      markers.markers.push_back(cylinder);

      // 速度方向箭头
      if (std::abs(obs.vx) > 0.01 || std::abs(obs.vy) > 0.01) {
        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = "map";
        arrow.header.stamp = now;
        arrow.ns = "obstacle_velocity";
        arrow.id = obs.id;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point start, end;
        start.x = obs.x;
        start.y = obs.y;
        start.z = 0.3;
        double speed = std::sqrt(obs.vx * obs.vx + obs.vy * obs.vy);
        end.x = obs.x + (obs.vx / speed) * 0.3;
        end.y = obs.y + (obs.vy / speed) * 0.3;
        end.z = 0.3;

        arrow.points.push_back(start);
        arrow.points.push_back(end);
        arrow.scale.x = 0.02;
        arrow.scale.y = 0.04;
        arrow.color = cylinder.color;
        arrow.color.a = 0.6;
        arrow.lifetime = rclcpp::Duration::from_seconds(0.3);
        markers.markers.push_back(arrow);
      }

      // 路径点连线（显示障碍物的移动路线）
      if (obs.waypoints.size() > 1) {
        visualization_msgs::msg::Marker path_line;
        path_line.header.frame_id = "map";
        path_line.header.stamp = now;
        path_line.ns = "obstacle_path";
        path_line.id = obs.id;
        path_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_line.action = visualization_msgs::msg::Marker::ADD;
        path_line.pose.orientation.w = 1.0;
        path_line.scale.x = 0.01;
        path_line.color.r = 0.5;
        path_line.color.g = 0.5;
        path_line.color.b = 0.5;
        path_line.color.a = 0.3;

        for (const auto & wp : obs.waypoints) {
          geometry_msgs::msg::Point p;
          p.x = wp.x;
          p.y = wp.y;
          p.z = 0.01;
          path_line.points.push_back(p);
        }
        // 闭合路径
        path_line.points.push_back(path_line.points.front());
        path_line.lifetime = rclcpp::Duration::from_seconds(0.3);
        markers.markers.push_back(path_line);
      }
    }

    marker_pub_->publish(markers);
  }

  // ============================================================
  // publishDynamicCostmap - 发布包含动态障碍物的代价地图
  // ============================================================
  void publishDynamicCostmap()
  {
    // 使用固定分辨率
    double resolution = 0.05;  // 5cm分辨率
    int width = static_cast<int>(map_width_ / resolution);
    int height = static_cast<int>(map_height_ / resolution);

    nav_msgs::msg::OccupancyGrid costmap;
    costmap.header.frame_id = "map";
    costmap.header.stamp = this->now();
    costmap.info.resolution = resolution;
    costmap.info.width = width;
    costmap.info.height = height;
    costmap.info.origin.position.x = 0.0;
    costmap.info.origin.position.y = 0.0;
    costmap.info.origin.position.z = 0.0;
    costmap.info.origin.orientation.w = 1.0;

    // 初始化为自由空间
    costmap.data.resize(width * height, 0);

    // 将动态障碍物注入代价地图
    for (const auto & obs : obstacles_) {
      // 障碍物占据的格子范围（含膨胀）
      int cx = static_cast<int>(obs.x / resolution);
      int cy = static_cast<int>(obs.y / resolution);
      int r = static_cast<int>((obs.radius + 0.1) / resolution);  // 额外10cm膨胀

      for (int dx = -r; dx <= r; ++dx) {
        for (int dy = -r; dy <= r; ++dy) {
          if (dx * dx + dy * dy > r * r) continue;  // 圆形膨胀

          int gx = cx + dx;
          int gy = cy + dy;
          if (gx >= 0 && gy >= 0 && gx < width && gy < height) {
            int idx = gy * width + gx;
            costmap.data[idx] = 100;  // 障碍物
          }
        }
      }
    }

    costmap_pub_->publish(costmap);
  }

  // ============================================================
  // 成员变量
  // ============================================================
  std::vector<DynamicObstacle> obstacles_;
  std::mt19937 rng_;
  double map_width_;
  double map_height_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::TimerBase::SharedPtr update_timer_;
};

}  // namespace agv_simulator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<agv_simulator::ObstacleManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
