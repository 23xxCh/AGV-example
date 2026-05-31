/**
 * ============================================================
 * cooperative_transport.cpp - 多车协同搬运协调器
 * ============================================================
 *
 * 【功能】
 * 协调两台AGV协同搬运大件货物：
 * - Leader AGV：正常导航，规划路径
 * - Follower AGV：保持固定偏移，跟随Leader移动
 * - 两车同步运动，维持相对位置不变
 *
 * 【工作流程】
 * 1. 收到协同搬运任务（两台AGV ID + 目标位置）
 * 2. Leader规划路径并预约
 * 3. 控制循环（10Hz）：
 *    a. 查询Leader位姿
 *    b. 计算Follower目标位姿（Leader + 偏移）
 *    c. 发布Leader速度指令
 *    d. 计算Follower速度，发布给Follower
 * 4. 到达目标 → 释放预约
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <agv_interfaces/srv/path_plan.hpp>
#include <agv_interfaces/srv/reserve_path.hpp>

#include <string>
#include <cmath>
#include <memory>

namespace agv_navigation
{

class CooperativeTransport : public rclcpp::Node
{
public:
  CooperativeTransport()
  : Node("cooperative_transport")
  {
    // 参数声明
    this->declare_parameter("leader_id", std::string("agv_001"));
    this->declare_parameter("follower_id", std::string("agv_002"));
    this->declare_parameter("offset_x", -0.5);   // Follower在Leader后方0.5m
    this->declare_parameter("offset_y", 0.0);
    this->declare_parameter("leader_speed", 0.3);
    this->declare_parameter("formation_angle", 0.0);  // 编队角度（弧度）
    this->declare_parameter("goal_x", 2.0);
    this->declare_parameter("goal_y", 1.5);
    this->declare_parameter("goal_tolerance", 0.2);
    this->declare_parameter("control_frequency", 10.0);

    leader_id_ = this->get_parameter("leader_id").as_string();
    follower_id_ = this->get_parameter("follower_id").as_string();
    offset_x_ = this->get_parameter("offset_x").as_double();
    offset_y_ = this->get_parameter("offset_y").as_double();
    leader_speed_ = this->get_parameter("leader_speed").as_double();
    goal_x_ = this->get_parameter("goal_x").as_double();
    goal_y_ = this->get_parameter("goal_y").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    double freq = this->get_parameter("control_frequency").as_double();

    // TF监听器
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Leader和Follower的速度指令发布器
    leader_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/" + leader_id_ + "/cmd_vel", rclcpp::QoS(10).reliable());
    follower_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/" + follower_id_ + "/cmd_vel", rclcpp::QoS(10).reliable());

    // 全局规划器客户端
    plan_client_ = this->create_client<agv_interfaces::srv::PathPlan>("/plan_path");
    reserve_client_ = this->create_client<agv_interfaces::srv::ReservePath>("/reserve_path");
    release_client_ = this->create_client<agv_interfaces::srv::ReservePath>("/release_path");

    // 控制定时器
    auto period = std::chrono::duration<double>(1.0 / freq);
    control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CooperativeTransport::controlLoop, this));

    RCLCPP_INFO(this->get_logger(),
      "协同搬运启动: Leader=%s, Follower=%s, 偏移=(%.1f, %.1f), 目标=(%.1f, %.1f)",
      leader_id_.c_str(), follower_id_.c_str(),
      offset_x_, offset_y_, goal_x_, goal_y_);

    // 启动导航
    startNavigation();
  }

private:
  // ============================================================
  // startNavigation - 启动协同导航
  // ============================================================
  void startNavigation()
  {
    // 查询Leader当前位置作为起点
    double start_x = 0.15, start_y = 0.15;
    try {
      auto tf = tf_buffer_->lookupTransform(
        "map", leader_id_ + "_base_link", tf2::TimePointZero,
        std::chrono::milliseconds(1000));
      start_x = tf.transform.translation.x;
      start_y = tf.transform.translation.y;
    } catch (const tf2::TransformException &) {
      RCLCPP_WARN(this->get_logger(), "无法查询Leader位置，使用默认起点");
    }

    // 调用全局规划器
    if (!plan_client_->wait_for_service(std::chrono::seconds(3))) {
      RCLCPP_ERROR(this->get_logger(), "全局规划服务不可用");
      return;
    }

    auto req = std::make_shared<agv_interfaces::srv::PathPlan::Request>();
    req->start.x = start_x;
    req->start.y = start_y;
    req->start.z = 0.0;
    req->goal.x = goal_x_;
    req->goal.y = goal_y_;
    req->goal.z = 0.0;
    req->use_current_pose = true;
    req->planner_id = "astar";

    auto future = plan_client_->async_send_request(req);
    auto status = future.wait_for(std::chrono::seconds(10));
    if (status != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "路径规划超时");
      return;
    }

    auto resp = future.get();
    if (!resp->success) {
      RCLCPP_ERROR(this->get_logger(), "路径规划失败: %s", resp->error_msg.c_str());
      return;
    }

    leader_path_ = resp->path;
    path_index_ = 0;
    navigation_active_ = true;

    RCLCPP_INFO(this->get_logger(), "协同路径规划成功: %zu个路径点", leader_path_.poses.size());
  }

  // ============================================================
  // controlLoop - 控制循环
  // ============================================================
  void controlLoop()
  {
    if (!navigation_active_) return;

    // 查询Leader位姿
    double leader_x, leader_y, leader_theta;
    if (!getRobotPose(leader_id_, leader_x, leader_y, leader_theta)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "无法查询Leader位姿");
      return;
    }

    // 检查是否到达目标
    double dx_goal = goal_x_ - leader_x;
    double dy_goal = goal_y_ - leader_y;
    double dist_to_goal = std::sqrt(dx_goal * dx_goal + dy_goal * dy_goal);

    if (dist_to_goal < goal_tolerance_) {
      // 到达目标，停止两车
      publishStop(leader_vel_pub_);
      publishStop(follower_vel_pub_);
      navigation_active_ = false;
      RCLCPP_INFO(this->get_logger(), "协同搬运完成！到达目标(%.1f, %.1f)", goal_x_, goal_y_);
      return;
    }

    // 计算Leader速度（朝向目标方向）
    geometry_msgs::msg::Twist leader_vel;
    double angle_to_goal = std::atan2(dy_goal, dx_goal);
    double angle_diff = angle_to_goal - leader_theta;
    while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

    // 转向控制
    if (std::abs(angle_diff) > 0.1) {
      leader_vel.angular.z = std::clamp(angle_diff * 1.0, -1.0, 1.0);
      leader_vel.linear.x = 0.0;  // 转向时不前进
    } else {
      leader_vel.linear.x = leader_speed_;
      leader_vel.angular.z = angle_diff * 0.5;
    }

    leader_vel_pub_->publish(leader_vel);

    // 计算Follower目标位姿（Leader位置 + 偏移）
    // 偏移方向根据Leader朝向旋转
    double cos_theta = std::cos(leader_theta);
    double sin_theta = std::sin(leader_theta);
    double follower_target_x = leader_x + offset_x_ * cos_theta - offset_y_ * sin_theta;
    double follower_target_y = leader_y + offset_x_ * sin_theta + offset_y_ * cos_theta;
    double follower_target_theta = leader_theta;  // 与Leader同朝向

    // 查询Follower当前位姿
    double follower_x, follower_y, follower_theta;
    if (!getRobotPose(follower_id_, follower_x, follower_y, follower_theta)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "无法查询Follower位姿");
      return;
    }

    // 计算Follower速度（朝向目标位置）
    double fx_dx = follower_target_x - follower_x;
    double fx_dy = follower_target_y - follower_y;
    double fx_dist = std::sqrt(fx_dx * fx_dx + fx_dy * fx_dy);
    double fx_angle = std::atan2(fx_dy, fx_dx);
    double fx_angle_diff = fx_angle - follower_theta;
    while (fx_angle_diff > M_PI) fx_angle_diff -= 2.0 * M_PI;
    while (fx_angle_diff < -M_PI) fx_angle_diff += 2.0 * M_PI;

    geometry_msgs::msg::Twist follower_vel;
    if (fx_dist > 0.1) {
      // 需要移动到目标位置
      if (std::abs(fx_angle_diff) > 0.2) {
        follower_vel.angular.z = std::clamp(fx_angle_diff * 1.0, -1.0, 1.0);
        follower_vel.linear.x = 0.0;
      } else {
        follower_vel.linear.x = std::min(leader_speed_, fx_dist * 2.0);
        follower_vel.angular.z = fx_angle_diff * 0.5;
      }
    } else {
      // 已在目标位置，跟随Leader速度
      follower_vel.linear.x = leader_vel.linear.x;
      follower_vel.angular.z = leader_vel.angular.z;
    }

    follower_vel_pub_->publish(follower_vel);

    // 发布Follower虚拟TF（用于其他模块参考）
    geometry_msgs::msg::TransformStamped follower_tf;
    follower_tf.header.stamp = this->now();
    follower_tf.header.frame_id = "map";
    follower_tf.child_frame_id = follower_id_ + "_formation_target";
    follower_tf.transform.translation.x = follower_target_x;
    follower_tf.transform.translation.y = follower_target_y;
    follower_tf.transform.translation.z = 0.0;
    follower_tf.transform.rotation.w = std::cos(follower_target_theta / 2.0);
    follower_tf.transform.rotation.z = std::sin(follower_target_theta / 2.0);
    tf_broadcaster_->sendTransform(follower_tf);
  }

  // ============================================================
  // getRobotPose - 查询机器人位姿
  // ============================================================
  bool getRobotPose(const std::string & agv_id,
                     double & x, double & y, double & theta)
  {
    try {
      auto tf = tf_buffer_->lookupTransform(
        "map", agv_id + "_base_link", tf2::TimePointZero,
        std::chrono::milliseconds(100));
      x = tf.transform.translation.x;
      y = tf.transform.translation.y;
      auto q = tf.transform.rotation;
      theta = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  // ============================================================
  // publishStop - 发布停止指令
  // ============================================================
  void publishStop(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub)
  {
    geometry_msgs::msg::Twist stop;
    pub->publish(stop);
  }

  // 成员变量
  std::string leader_id_, follower_id_;
  double offset_x_, offset_y_;
  double leader_speed_;
  double goal_x_, goal_y_;
  double goal_tolerance_;
  bool navigation_active_ = false;

  nav_msgs::msg::Path leader_path_;
  size_t path_index_ = 0;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr leader_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr follower_vel_pub_;

  rclcpp::Client<agv_interfaces::srv::PathPlan>::SharedPtr plan_client_;
  rclcpp::Client<agv_interfaces::srv::ReservePath>::SharedPtr reserve_client_;
  rclcpp::Client<agv_interfaces::srv::ReservePath>::SharedPtr release_client_;

  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace agv_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<agv_navigation::CooperativeTransport>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
