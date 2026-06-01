/**
 * ============================================================
 * data_recorder.cpp - 数据记录器实现
 * ============================================================
 *
 * 【日志格式】JSON Lines（每行一个JSON对象）
 * 示例：
 * {"ts":1234567890.123,"type":"status","agv":"agv_001","x":1.0,"y":2.0,"v":0.3,"battery":85.0,"status":1}
 * {"ts":1234567890.456,"type":"cmd_vel","agv":"agv_001","linear_x":0.3,"angular_z":0.1}
 * {"ts":1234567890.789,"type":"path","agv":"agv_001","points":25}
 */

#include "data_recorder.hpp"
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace agv_navigation
{

// 转义JSON字符串中的特殊字符（双引号、反斜杠、换行符等）
static std::string escapeJsonString(const std::string & input)
{
  std::string result;
  result.reserve(input.size() + 8);
  for (char c : input) {
    switch (c) {
      case '"':  result += "\\\""; break;
      case '\\': result += "\\\\"; break;
      case '\n': result += "\\n";  break;
      case '\r': result += "\\r";  break;
      case '\t': result += "\\t";  break;
      default:   result += c;      break;
    }
  }
  return result;
}

DataRecorder::DataRecorder(const rclcpp::NodeOptions & options)
: Node("data_recorder", options),
  record_count_(0)
{
  // 声明参数
  this->declare_parameter("log_dir", std::string("~/AGV/logs"));
  this->declare_parameter("agv_ids", std::vector<std::string>{"agv_001", "agv_002"});
  this->declare_parameter("record_status", true);
  this->declare_parameter("record_cmd_vel", true);
  this->declare_parameter("record_path", true);

  log_dir_ = this->get_parameter("log_dir").as_string();
  agv_ids_ = this->get_parameter("agv_ids").as_string_array();
  record_status_ = this->get_parameter("record_status").as_bool();
  record_cmd_vel_ = this->get_parameter("record_cmd_vel").as_bool();
  record_path_ = this->get_parameter("record_path").as_bool();

  // 展开 ~ 路径（HOME环境变量可能为空，需要检查）
  if (log_dir_.front() == '~') {
    const char * home = std::getenv("HOME");
    if (home != nullptr) {
      log_dir_ = std::string(home) + log_dir_.substr(1);
    } else {
      RCLCPP_WARN(this->get_logger(), "HOME环境变量未设置，使用原始路径: %s", log_dir_.c_str());
    }
  }

  // 创建日志目录
  std::filesystem::create_directories(log_dir_);

  // 生成日志文件名（带时间戳）
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << log_dir_ << "/agv_log_"
     << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
     << ".jsonl";
  log_filename_ = ss.str();

  log_file_.open(log_filename_, std::ios::out | std::ios::app);
  if (!log_file_.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "无法打开日志文件: %s", log_filename_.c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "数据记录器启动: %s", log_filename_.c_str());

  // 创建订阅器
  for (const auto & agv_id : agv_ids_) {
    if (record_status_) {
      status_subs_.push_back(
        this->create_subscription<agv_interfaces::msg::AGVStatus>(
          agv_id + "/status", rclcpp::QoS(10).reliable(),
          [this, agv_id](const agv_interfaces::msg::AGVStatus::SharedPtr msg) {
            agvStatusCallback(agv_id, msg);
          }));
    }

    if (record_cmd_vel_) {
      cmd_vel_subs_.push_back(
        this->create_subscription<geometry_msgs::msg::Twist>(
          agv_id + "/cmd_vel", rclcpp::QoS(10).reliable(),
          [this, agv_id](const geometry_msgs::msg::Twist::SharedPtr msg) {
            cmdVelCallback(agv_id, msg);
          }));
    }

    if (record_path_) {
      path_subs_.push_back(
        this->create_subscription<nav_msgs::msg::Path>(
          agv_id + "/planned_path", rclcpp::QoS(5).reliable(),
          [this, agv_id](const nav_msgs::msg::Path::SharedPtr msg) {
            pathCallback(agv_id, msg);
          }));
    }
  }
}

DataRecorder::~DataRecorder()
{
  closeLog();
}

// ============================================================
// agvStatusCallback - 记录AGV状态
// ============================================================
void DataRecorder::agvStatusCallback(
  const std::string & agv_id,
  const agv_interfaces::msg::AGVStatus::SharedPtr msg)
{
  std::stringstream data;
  data << "{\"x\":" << msg->pose.pose.pose.position.x
       << ",\"y\":" << msg->pose.pose.pose.position.y
       << ",\"v\":" << msg->linear_velocity
       << ",\"w\":" << msg->angular_velocity
       << ",\"battery\":" << msg->battery_level
       << ",\"status\":" << static_cast<int>(msg->status)
       << ",\"task\":\"" << escapeJsonString(msg->current_task_id) << "\"}";

  double ts = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  writeRecord("status", agv_id, data.str(), ts);
}

// ============================================================
// cmdVelCallback - 记录速度指令
// ============================================================
void DataRecorder::cmdVelCallback(
  const std::string & agv_id,
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::stringstream data;
  data << "{\"linear_x\":" << msg->linear.x
       << ",\"linear_y\":" << msg->linear.y
       << ",\"angular_z\":" << msg->angular.z << "}";

  writeRecord("cmd_vel", agv_id, data.str());
}

// ============================================================
// pathCallback - 记录路径规划结果
// ============================================================
void DataRecorder::pathCallback(
  const std::string & agv_id,
  const nav_msgs::msg::Path::SharedPtr msg)
{
  std::stringstream data;
  data << "{\"points\":" << msg->poses.size() << "}";

  double ts = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  writeRecord("path", agv_id, data.str(), ts);
}

// ============================================================
// writeRecord - 写入日志记录
// ============================================================
void DataRecorder::writeRecord(const std::string & type, const std::string & agv_id,
                                const std::string & data, double timestamp)
{
  std::lock_guard<std::mutex> lock(file_mutex_);

  if (!log_file_.is_open()) return;

  double ts = timestamp;
  if (ts <= 0) {
    ts = this->now().seconds();
  }

  log_file_ << "{\"ts\":" << std::fixed << std::setprecision(3) << ts
            << ",\"type\":\"" << type << "\""
            << ",\"agv\":\"" << agv_id << "\""
            << "," << data.substr(1);  // 去掉开头的 {，合并到外层

  log_file_ << std::endl;

  ++record_count_;
  if (record_count_ % 1000 == 0) {
    RCLCPP_INFO(this->get_logger(), "已记录 %d 条数据", record_count_);
  }
}

// ============================================================
// closeLog - 关闭日志文件
// ============================================================
void DataRecorder::closeLog()
{
  std::lock_guard<std::mutex> lock(file_mutex_);
  if (log_file_.is_open()) {
    log_file_.close();
    RCLCPP_INFO(this->get_logger(),
      "日志文件已关闭: %s (共 %d 条记录)", log_filename_.c_str(), record_count_);
  }
}

}  // namespace agv_navigation
