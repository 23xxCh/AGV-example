/**
 * ============================================================
 * agv_sim_node.cpp - AGV 2D仿真器节点
 * ============================================================
 *
 * 【这是什么】
 * 简单的2D仿真器，模拟AGV运动，发布TF和可视化Marker。
 * 订阅 /cmd_vel 速度指令，使用差分运动学模型更新位姿。
 *
 * 【数据流】
 *   DWA → cmd_vel → 仿真器 → TF (map→base_link) → DWA
 *                      ↓
 *                   Marker → RViz2
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <agv_interfaces/msg/agv_status.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <array>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class AGVSimulator : public rclcpp::Node
{
public:
  AGVSimulator()
  : Node("agv_simulator"),
    x_(0.0), y_(0.0), theta_(0.0),
    has_cmd_(false)
  {
    this->declare_parameter("initial_x", 0.15);
    this->declare_parameter("initial_y", 0.15);
    this->declare_parameter("initial_theta", 0.0);
    this->declare_parameter("update_rate", 50.0);
    this->declare_parameter("base_frame", std::string("base_link"));
    this->declare_parameter("agv_id", std::string("agv_001"));

    // 电池参数
    this->declare_parameter("initial_battery", 100.0);      // 初始电量(%)
    this->declare_parameter("battery_drain_rate", 0.05);     // 移动时耗电(%/秒)
    this->declare_parameter("battery_charge_rate", 0.5);     // 充电速率(%/秒)
    this->declare_parameter("low_battery_threshold", 20.0);  // 低电量阈值(%)
    this->declare_parameter("charging_station_x", 0.15);     // 充电站X坐标
    this->declare_parameter("charging_station_y", 0.15);     // 充电站Y坐标

    x_ = this->get_parameter("initial_x").as_double();
    y_ = this->get_parameter("initial_y").as_double();
    theta_ = this->get_parameter("initial_theta").as_double();
    double rate = this->get_parameter("update_rate").as_double();
    base_frame_ = this->get_parameter("base_frame").as_string();
    agv_id_ = this->get_parameter("agv_id").as_string();

    // 电池参数
    battery_level_ = this->get_parameter("initial_battery").as_double();
    battery_drain_rate_ = this->get_parameter("battery_drain_rate").as_double();
    battery_charge_rate_ = this->get_parameter("battery_charge_rate").as_double();
    low_battery_threshold_ = this->get_parameter("low_battery_threshold").as_double();
    charging_station_x_ = this->get_parameter("charging_station_x").as_double();
    charging_station_y_ = this->get_parameter("charging_station_y").as_double();
    is_charging_ = false;

    // 根据agv_id选择不同颜色（用于多车可视化区分）
    // 预定义6种颜色：红、绿、蓝、黄、青、品红
    std::vector<std::array<float, 3>> colors = {
      {1.0, 0.0, 0.0},  // 红色
      {0.0, 1.0, 0.0},  // 绿色
      {0.0, 0.3, 1.0},  // 蓝色
      {1.0, 1.0, 0.0},  // 黄色
      {0.0, 1.0, 1.0},  // 青色
      {1.0, 0.0, 1.0},  // 品红
    };
    // 从agv_id中提取数字作为颜色索引
    int color_idx = 0;
    try {
      color_idx = std::stoi(agv_id_.substr(agv_id_.find_last_of('_') + 1)) % colors.size();
    } catch (...) {
      color_idx = 0;
    }
    color_r_ = colors[color_idx][0];
    color_g_ = colors[color_idx][1];
    color_b_ = colors[color_idx][2];

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "robot_marker", rclcpp::QoS(10).reliable());

    // AGV状态发布器
    status_pub_ = this->create_publisher<agv_interfaces::msg::AGVStatus>(
      "status", rclcpp::QoS(10).reliable());

    // 电池可视化Marker发布器
    battery_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "battery_markers", rclcpp::QoS(10).reliable());

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10).reliable(),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        current_vel_ = *msg;
        has_cmd_ = true;
      });

    auto period = std::chrono::duration<double>(1.0 / rate);
    sim_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&AGVSimulator::simUpdate, this));

    RCLCPP_INFO(this->get_logger(),
      "AGV仿真器启动: 位置(%.2f, %.2f, %.2f), 频率=%.0f Hz",
      x_, y_, theta_, rate);
  }

private:
  void simUpdate()
  {
    // 使用node时钟获取当前时间（避免SIM_TIME vs STEADY_TIME冲突）
    auto now = this->get_clock()->now();

    double dt = 0.02;  // 50Hz = 0.02s
    double v = current_vel_.linear.x;
    double omega = current_vel_.angular.z;

    // 差分运动学更新
    x_ += v * std::cos(theta_) * dt;
    y_ += v * std::sin(theta_) * dt;
    theta_ += omega * dt;

    while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
    while (theta_ < -M_PI) theta_ += 2.0 * M_PI;

    // 发布TF
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = "map";
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.w = std::cos(theta_ / 2.0);
    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = std::sin(theta_ / 2.0);
    tf_broadcaster_->sendTransform(tf);

    // 发布机器人箭头Marker（颜色根据agv_id区分）
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = "map";
    arrow.header.stamp = now;
    arrow.ns = "agv";
    arrow.id = 0;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.pose.position.x = x_;
    arrow.pose.position.y = y_;
    arrow.pose.position.z = 0.05;
    arrow.pose.orientation.w = std::cos(theta_ / 2.0);
    arrow.pose.orientation.z = std::sin(theta_ / 2.0);
    arrow.scale.x = 0.3;
    arrow.scale.y = 0.15;
    arrow.scale.z = 0.1;
    arrow.color.r = color_r_;
    arrow.color.g = color_g_;
    arrow.color.b = color_b_;
    arrow.color.a = 1.0;
    marker_pub_->publish(arrow);

    // 发布机器人底盘Marker（同色半透明）
    visualization_msgs::msg::Marker box;
    box.header = arrow.header;
    box.ns = "agv_body";
    box.id = 1;
    box.type = visualization_msgs::msg::Marker::CUBE;
    box.action = visualization_msgs::msg::Marker::ADD;
    box.pose.position.x = x_;
    box.pose.position.y = y_;
    box.pose.position.z = 0.02;
    box.pose.orientation = arrow.pose.orientation;
    box.scale.x = 0.35;
    box.scale.y = 0.25;
    box.scale.z = 0.04;
    box.color.r = color_r_;
    box.color.g = color_g_;
    box.color.b = color_b_;
    box.color.a = 0.5;
    marker_pub_->publish(box);

    // 电池仿真
    updateBattery(dt);

    // 发布AGV状态（每10个周期发布一次，约5Hz）
    status_counter_++;
    if (status_counter_ >= 10) {
      status_counter_ = 0;
      publishStatus();
      publishBatteryMarker();
    }
  }

  // ============================================================
  // updateBattery - 更新电池状态
  // ============================================================
  void updateBattery(double dt)
  {
    double speed = std::sqrt(
      current_vel_.linear.x * current_vel_.linear.x +
      current_vel_.linear.y * current_vel_.linear.y);

    if (is_charging_) {
      // 充电模式
      battery_level_ += battery_charge_rate_ * dt;
      if (battery_level_ >= 100.0) {
        battery_level_ = 100.0;
        is_charging_ = false;
        RCLCPP_INFO(this->get_logger(), "%s 充电完成！电量=%.0f%%", agv_id_.c_str(), battery_level_);
      }
    } else if (speed > 0.01) {
      // 移动时消耗电量
      battery_level_ -= battery_drain_rate_ * dt;
      if (battery_level_ < 0.0) battery_level_ = 0.0;
    }

    // 检查是否到达充电站
    double dx = x_ - charging_station_x_;
    double dy = y_ - charging_station_y_;
    double dist_to_charger = std::sqrt(dx * dx + dy * dy);
    if (dist_to_charger < 0.3 && battery_level_ < 100.0) {
      is_charging_ = true;
    }
  }

  // ============================================================
  // publishStatus - 发布AGV状态
  // ============================================================
  void publishStatus()
  {
    auto status_msg = agv_interfaces::msg::AGVStatus();
    status_msg.header.stamp = this->now();
    status_msg.header.frame_id = "map";
    status_msg.agv_id = agv_id_;
    status_msg.pose.header = status_msg.header;
    status_msg.pose.pose.pose.position.x = x_;
    status_msg.pose.pose.pose.position.y = y_;
    status_msg.pose.pose.pose.position.z = 0.0;
    status_msg.pose.pose.pose.orientation.w = std::cos(theta_ / 2.0);
    status_msg.pose.pose.pose.orientation.z = std::sin(theta_ / 2.0);
    status_msg.linear_velocity = current_vel_.linear.x;
    status_msg.angular_velocity = current_vel_.angular.z;
    status_msg.battery_level = battery_level_;

    if (is_charging_) {
      status_msg.status = agv_interfaces::msg::AGVStatus::STATUS_CHARGING;
    } else if (has_cmd_ && std::abs(current_vel_.linear.x) > 0.01) {
      status_msg.status = agv_interfaces::msg::AGVStatus::STATUS_EXECUTING;
    } else {
      status_msg.status = agv_interfaces::msg::AGVStatus::STATUS_IDLE;
    }

    status_pub_->publish(status_msg);
  }

  // ============================================================
  // publishBatteryMarker - 发布电池可视化Marker
  // ============================================================
  void publishBatteryMarker()
  {
    visualization_msgs::msg::MarkerArray markers;
    auto now = this->now();

    // 电池电量条
    visualization_msgs::msg::Marker bar;
    bar.header.frame_id = "map";
    bar.header.stamp = now;
    bar.ns = "battery_bar";
    bar.id = 0;
    bar.type = visualization_msgs::msg::Marker::CUBE;
    bar.action = visualization_msgs::msg::Marker::ADD;
    bar.pose.position.x = x_;
    bar.pose.position.y = y_;
    bar.pose.position.z = 0.4;
    bar.pose.orientation.w = 1.0;

    // 电量条长度与电量成正比
    double bar_width = 0.3 * (battery_level_ / 100.0);
    bar.scale.x = bar_width;
    bar.scale.y = 0.04;
    bar.scale.z = 0.03;

    // 颜色：绿色(>50%) → 黄色(20~50%) → 红色(<20%)
    if (battery_level_ > 50.0) {
      bar.color.r = 0.0;
      bar.color.g = 1.0;
      bar.color.b = 0.0;
    } else if (battery_level_ > 20.0) {
      bar.color.r = 1.0;
      bar.color.g = 1.0;
      bar.color.b = 0.0;
    } else {
      bar.color.r = 1.0;
      bar.color.g = 0.0;
      bar.color.b = 0.0;
    }
    bar.color.a = 0.8;
    bar.lifetime = rclcpp::Duration::from_seconds(0.3);
    markers.markers.push_back(bar);

    // 低电量警告闪烁
    if (battery_level_ < low_battery_threshold_) {
      visualization_msgs::msg::Marker warning;
      warning.header = bar.header;
      warning.ns = "battery_warning";
      warning.id = 0;
      warning.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      warning.action = visualization_msgs::msg::Marker::ADD;
      warning.pose.position.x = x_;
      warning.pose.position.y = y_;
      warning.pose.position.z = 0.5;
      warning.pose.orientation.w = 1.0;
      warning.scale.z = 0.15;
      warning.color.r = 1.0;
      warning.color.g = 0.5;
      warning.color.b = 0.0;
      warning.color.a = 1.0;
      warning.text = "LOW BATTERY " + std::to_string(static_cast<int>(battery_level_)) + "%";
      warning.lifetime = rclcpp::Duration::from_seconds(0.3);
      markers.markers.push_back(warning);
    }

    // 充电中标识
    if (is_charging_) {
      visualization_msgs::msg::Marker charging;
      charging.header = bar.header;
      charging.ns = "charging_indicator";
      charging.id = 0;
      charging.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      charging.action = visualization_msgs::msg::Marker::ADD;
      charging.pose.position.x = x_;
      charging.pose.position.y = y_;
      charging.pose.position.z = 0.55;
      charging.pose.orientation.w = 1.0;
      charging.scale.z = 0.12;
      charging.color.r = 0.0;
      charging.color.g = 1.0;
      charging.color.b = 1.0;
      charging.color.a = 1.0;
      charging.text = "CHARGING " + std::to_string(static_cast<int>(battery_level_)) + "%";
      charging.lifetime = rclcpp::Duration::from_seconds(0.3);
      markers.markers.push_back(charging);
    }

    battery_marker_pub_->publish(markers);
  }

  double x_, y_, theta_;
  bool has_cmd_;
  std::string base_frame_;
  std::string agv_id_;
  float color_r_, color_g_, color_b_;
  geometry_msgs::msg::Twist current_vel_;

  // 电池相关
  double battery_level_;        // 当前电量(%)
  double battery_drain_rate_;   // 耗电速率(%/秒)
  double battery_charge_rate_;  // 充电速率(%/秒)
  double low_battery_threshold_; // 低电量阈值(%)
  double charging_station_x_;   // 充电站X坐标
  double charging_station_y_;   // 充电站Y坐标
  bool is_charging_;            // 是否正在充电
  int status_counter_ = 0;     // 状态发布计数器

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<agv_interfaces::msg::AGVStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr battery_marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr sim_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AGVSimulator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
