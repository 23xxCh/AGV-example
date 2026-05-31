/**
 * ============================================================
 * data_recorder.hpp - 数据记录器节点
 * ============================================================
 *
 * 【功能】
 * 记录AGV运行数据到JSON Lines文件，用于事后分析和回放：
 * - AGV状态（位置、速度、电池）
 * - 速度指令
 * - 任务事件（分配、完成、失败）
 * - 路径规划结果
 *
 * 【输出格式】
 * JSON Lines（每行一个JSON对象），包含时间戳和数据类型
 */

#ifndef AGV_NAVIGATION__DATA_RECORDER_HPP_
#define AGV_NAVIGATION__DATA_RECORDER_HPP_

#include <string>
#include <fstream>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "agv_interfaces/msg/agv_status.hpp"

namespace agv_navigation
{

class DataRecorder : public rclcpp::Node
{
public:
  explicit DataRecorder(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DataRecorder();

private:
  // AGV状态回调
  void agvStatusCallback(const std::string & agv_id,
                          const agv_interfaces::msg::AGVStatus::SharedPtr msg);

  // 速度指令回调
  void cmdVelCallback(const std::string & agv_id,
                       const geometry_msgs::msg::Twist::SharedPtr msg);

  // 路径回调
  void pathCallback(const std::string & agv_id,
                     const nav_msgs::msg::Path::SharedPtr msg);

  // 写入日志记录
  void writeRecord(const std::string & type, const std::string & agv_id,
                    const std::string & data, double timestamp = 0);

  // 关闭日志文件
  void closeLog();

  // 参数
  std::string log_dir_;
  std::vector<std::string> agv_ids_;
  bool record_status_;
  bool record_cmd_vel_;
  bool record_path_;

  // 日志文件
  std::ofstream log_file_;
  std::mutex file_mutex_;
  std::string log_filename_;

  // 订阅器
  std::vector<rclcpp::Subscription<agv_interfaces::msg::AGVStatus>::SharedPtr> status_subs_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> cmd_vel_subs_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> path_subs_;

  // 记录计数
  int record_count_;
};

}  // namespace agv_navigation

#endif  // AGV_NAVIGATION__DATA_RECORDER_HPP_
