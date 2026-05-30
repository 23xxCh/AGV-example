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
#include <tf2_ros/transform_broadcaster.h>
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

    x_ = this->get_parameter("initial_x").as_double();
    y_ = this->get_parameter("initial_y").as_double();
    theta_ = this->get_parameter("initial_theta").as_double();
    double rate = this->get_parameter("update_rate").as_double();
    base_frame_ = this->get_parameter("base_frame").as_string();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "robot_marker", rclcpp::QoS(10).reliable());

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

    // 发布机器人箭头Marker
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
    arrow.color.r = 0.0;
    arrow.color.g = 1.0;
    arrow.color.b = 0.0;
    arrow.color.a = 1.0;
    marker_pub_->publish(arrow);

    // 发布机器人底盘Marker（蓝色矩形）
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
    box.color.r = 0.0;
    box.color.g = 0.3;
    box.color.b = 1.0;
    box.color.a = 0.7;
    marker_pub_->publish(box);
  }

  double x_, y_, theta_;
  bool has_cmd_;
  std::string base_frame_;
  geometry_msgs::msg::Twist current_vel_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
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
