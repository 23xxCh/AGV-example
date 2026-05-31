/**
 * ============================================================
 * traffic_manager.hpp - 交通管理器节点
 * ============================================================
 *
 * 【功能】
 * 通过路径预约机制防止多车碰撞：
 * 1. AGV规划好路径后，调用reserve_path服务预约
 * 2. 交通管理器检查路径上是否有时间冲突
 * 3. 无冲突 → 批准，AGV开始行驶
 * 4. 有冲突 → 返回等待时间，AGV稍后重试
 *
 * 【预约表原理】
 * 使用时间-空间预约表，记录每个格子在什么时间段被哪个AGV占用
 * 新路径预约时，检查是否会与已有预约在时间和空间上重叠
 */

#ifndef AGV_SCHEDULER__TRAFFIC_MANAGER_HPP_
#define AGV_SCHEDULER__TRAFFIC_MANAGER_HPP_

#include <string>
#include <vector>
#include <unordered_map>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "agv_interfaces/srv/reserve_path.hpp"

namespace agv_scheduler
{

// 格子坐标
struct CellKey {
  int x, y;
  bool operator==(const CellKey & other) const {
    return x == other.x && y == other.y;
  }
};

// 为CellKey定义哈希函数
struct CellKeyHash {
  size_t operator()(const CellKey & k) const {
    return std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 16);
  }
};

// 单条预约记录
struct Reservation {
  std::string agv_id;
  double start_time;  // 进入该格子的时间
  double end_time;    // 离开该格子的时间
};

class TrafficManager : public rclcpp::Node
{
public:
  explicit TrafficManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // 路径预约服务回调
  void handleReservePath(
    const std::shared_ptr<agv_interfaces::srv::ReservePath::Request> request,
    std::shared_ptr<agv_interfaces::srv::ReservePath::Response> response);

  // 路径释放服务回调
  void handleReleasePath(
    const std::shared_ptr<agv_interfaces::srv::ReservePath::Request> request,
    std::shared_ptr<agv_interfaces::srv::ReservePath::Response> response);

  // 清理过期预约
  void cleanupExpired();

  // 检查路径是否有冲突
  bool checkConflict(
    const std::string & agv_id,
    const nav_msgs::msg::Path & path,
    double start_time,
    double speed,
    std::string & conflict_agv_id,
    double & wait_time);

  // 世界坐标转格子坐标
  CellKey worldToGrid(double x, double y) const;

  // 预约表：格子坐标 -> 预约列表
  std::unordered_map<CellKey, std::vector<Reservation>, CellKeyHash> reservation_table_;
  std::mutex table_mutex_;

  // 参数
  double resolution_;       // 地图分辨率
  double origin_x_;         // 地图原点X
  double origin_y_;         // 地图原点Y
  int safety_distance_;     // 安全距离（格子数）
  double reservation_timeout_;  // 预约超时时间

  // 服务
  rclcpp::Service<agv_interfaces::srv::ReservePath>::SharedPtr reserve_service_;
  rclcpp::Service<agv_interfaces::srv::ReservePath>::SharedPtr release_service_;

  // 定时器：清理过期预约
  rclcpp::TimerBase::SharedPtr cleanup_timer_;
};

}  // namespace agv_scheduler

#endif  // AGV_SCHEDULER__TRAFFIC_MANAGER_HPP_
