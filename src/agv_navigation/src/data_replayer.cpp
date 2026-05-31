/**
 * ============================================================
 * data_replayer.cpp - 数据回放器实现
 * ============================================================
 *
 * 【回放流程】
 * 1. 加载JSON Lines日志文件到内存
 * 2. 按时间戳排序
 * 3. 定时器以100Hz检查，按时间间隔发布消息
 * 4. 支持倍速和循环
 */

#include "data_replayer.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <regex>

namespace agv_navigation
{

DataReplayer::DataReplayer(const rclcpp::NodeOptions & options)
: Node("data_replayer", options),
  current_index_(0),
  start_time_(0),
  replay_start_time_(0),
  paused_(false)
{
  // 声明参数
  this->declare_parameter("log_file", std::string(""));
  this->declare_parameter("playback_speed", 1.0);
  this->declare_parameter("loop_playback", false);

  log_file_ = this->get_parameter("log_file").as_string();
  playback_speed_ = this->get_parameter("playback_speed").as_double();
  loop_playback_ = this->get_parameter("loop_playback").as_bool();

  if (log_file_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "未指定日志文件");
    return;
  }

  // 加载日志文件
  if (!loadLogFile(log_file_)) {
    RCLCPP_ERROR(this->get_logger(), "加载日志文件失败: %s", log_file_.c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(),
    "数据回放器启动: %s (%zu条记录, %.1fx速度)",
    log_file_.c_str(), records_.size(), playback_speed_);

  // 创建回放定时器（100Hz检查）
  replay_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&DataReplayer::replayTimerCallback, this));
}

// ============================================================
// loadLogFile - 加载JSON Lines日志文件
// ============================================================
bool DataReplayer::loadLogFile(const std::string & filename)
{
  std::ifstream file(filename);
  if (!file.is_open()) {
    return false;
  }

  std::string line;
  while (std::getline(file, line)) {
    if (line.empty()) continue;

    LogRecord record;
    record.json_data = line;

    // 解析基本字段
    record.timestamp = parseJsonDouble(line, "ts");
    record.type = parseJsonString(line, "type");
    record.agv_id = parseJsonString(line, "agv");

    if (record.type.empty() || record.agv_id.empty()) continue;

    records_.push_back(record);
  }

  // 按时间戳排序
  std::sort(records_.begin(), records_.end(),
    [](const LogRecord & a, const LogRecord & b) {
      return a.timestamp < b.timestamp;
    });

  return !records_.empty();
}

// ============================================================
// replayTimerCallback - 回放定时器
// ============================================================
void DataReplayer::replayTimerCallback()
{
  if (records_.empty() || paused_) return;

  double now = this->now().seconds();

  // 初始化回放起始时间
  if (replay_start_time_ == 0) {
    replay_start_time_ = now;
    start_time_ = records_[0].timestamp;
  }

  // 计算当前应该回放到的时间点
  double elapsed = (now - replay_start_time_) * playback_speed_;
  double target_time = start_time_ + elapsed;

  // 发布所有时间戳 <= target_time 的记录
  while (current_index_ < records_.size()) {
    const auto & record = records_[current_index_];
    if (record.timestamp > target_time) break;

    // 创建发布器（如果不存在）
    if (record.type == "status" && status_pubs_.find(record.agv_id) == status_pubs_.end()) {
      status_pubs_[record.agv_id] = this->create_publisher<agv_interfaces::msg::AGVStatus>(
        record.agv_id + "/status/replay", rclcpp::QoS(10).reliable());
    }
    if (record.type == "cmd_vel" && cmd_vel_pubs_.find(record.agv_id) == cmd_vel_pubs_.end()) {
      cmd_vel_pubs_[record.agv_id] = this->create_publisher<geometry_msgs::msg::Twist>(
        record.agv_id + "/cmd_vel/replay", rclcpp::QoS(10).reliable());
    }

    // 发布数据
    if (record.type == "status") {
      publishStatus(record);
    } else if (record.type == "cmd_vel") {
      publishCmdVel(record);
    }

    ++current_index_;
  }

  // 检查是否回放完成
  if (current_index_ >= records_.size()) {
    if (loop_playback_) {
      current_index_ = 0;
      replay_start_time_ = 0;
      RCLCPP_INFO(this->get_logger(), "回放循环重新开始");
    } else {
      RCLCPP_INFO(this->get_logger(), "回放完成");
      replay_timer_->cancel();
    }
  }
}

// ============================================================
// publishStatus - 发布状态记录
// ============================================================
void DataReplayer::publishStatus(const LogRecord & record)
{
  auto it = status_pubs_.find(record.agv_id);
  if (it == status_pubs_.end()) return;

  agv_interfaces::msg::AGVStatus msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";
  msg.agv_id = record.agv_id;
  msg.pose.pose.pose.position.x = parseJsonDouble(record.json_data, "x");
  msg.pose.pose.pose.position.y = parseJsonDouble(record.json_data, "y");
  msg.linear_velocity = parseJsonDouble(record.json_data, "v");
  msg.angular_velocity = parseJsonDouble(record.json_data, "w");
  msg.battery_level = parseJsonDouble(record.json_data, "battery");
  msg.status = static_cast<uint8_t>(parseJsonInt(record.json_data, "status"));

  it->second->publish(msg);
}

// ============================================================
// publishCmdVel - 发布速度指令记录
// ============================================================
void DataReplayer::publishCmdVel(const LogRecord & record)
{
  auto it = cmd_vel_pubs_.find(record.agv_id);
  if (it == cmd_vel_pubs_.end()) return;

  geometry_msgs::msg::Twist msg;
  msg.linear.x = parseJsonDouble(record.json_data, "linear_x");
  msg.linear.y = parseJsonDouble(record.json_data, "linear_y");
  msg.angular.z = parseJsonDouble(record.json_data, "angular_z");

  it->second->publish(msg);
}

// ============================================================
// JSON解析辅助函数（简单实现，避免引入外部依赖）
// ============================================================
double DataReplayer::parseJsonDouble(const std::string & json, const std::string & key)
{
  std::string search = "\"" + key + "\":";
  auto pos = json.find(search);
  if (pos == std::string::npos) return 0.0;

  pos += search.size();
  auto end = json.find_first_of(",}", pos);
  if (end == std::string::npos) end = json.size();

  try {
    return std::stod(json.substr(pos, end - pos));
  } catch (...) {
    return 0.0;
  }
}

int DataReplayer::parseJsonInt(const std::string & json, const std::string & key)
{
  return static_cast<int>(parseJsonDouble(json, key));
}

std::string DataReplayer::parseJsonString(const std::string & json, const std::string & key)
{
  std::string search = "\"" + key + "\":\"";
  auto pos = json.find(search);
  if (pos == std::string::npos) return "";

  pos += search.size();
  auto end = json.find("\"", pos);
  if (end == std::string::npos) return "";

  return json.substr(pos, end - pos);
}

}  // namespace agv_navigation
