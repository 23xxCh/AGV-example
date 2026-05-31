/**
 * ============================================================
 * data_replayer.hpp - 数据回放器节点
 * ============================================================
 *
 * 【功能】
 * 读取JSON Lines日志文件，按时间戳回放数据：
 * - 按原始时间间隔发布消息
 * - 支持倍速回放（0.5x, 1x, 2x, 5x）
 * - 支持暂停/继续
 */

#ifndef AGV_NAVIGATION__DATA_REPLAYER_HPP_
#define AGV_NAVIGATION__DATA_REPLAYER_HPP_

#include <string>
#include <fstream>
#include <queue>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "agv_interfaces/msg/agv_status.hpp"

namespace agv_navigation
{

struct LogRecord {
  double timestamp;
  std::string type;
  std::string agv_id;
  std::string json_data;
};

class DataReplayer : public rclcpp::Node
{
public:
  explicit DataReplayer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // 加载日志文件
  bool loadLogFile(const std::string & filename);

  // 回放定时器
  void replayTimerCallback();

  // 发布状态记录
  void publishStatus(const LogRecord & record);

  // 发布速度指令记录
  void publishCmdVel(const LogRecord & record);

  // 解析JSON值
  double parseJsonDouble(const std::string & json, const std::string & key);
  int parseJsonInt(const std::string & json, const std::string & key);
  std::string parseJsonString(const std::string & json, const std::string & key);

  // 参数
  std::string log_file_;
  double playback_speed_;
  bool loop_playback_;

  // 日志记录队列
  std::vector<LogRecord> records_;
  size_t current_index_;
  double start_time_;
  double replay_start_time_;
  bool paused_;

  // 发布器
  std::map<std::string, rclcpp::Publisher<agv_interfaces::msg::AGVStatus>::SharedPtr> status_pubs_;
  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> cmd_vel_pubs_;

  // 回放定时器
  rclcpp::TimerBase::SharedPtr replay_timer_;
};

}  // namespace agv_navigation

#endif  // AGV_NAVIGATION__DATA_REPLAYER_HPP_
