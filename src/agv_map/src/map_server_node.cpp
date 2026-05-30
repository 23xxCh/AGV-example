/**
 * ============================================================
 * map_server_node.cpp - 地图服务器节点入口
 * ============================================================
 *
 * 【这是什么】
 * 这是地图服务器的main函数，负责：
 * 1. 初始化ROS2运行环境
 * 2. 创建MapServer节点实例
 * 3. 进入事件循环，等待回调
 *
 * 【技术要点 - ROS2节点启动流程】
 *
 *   rclcpp::init(argc, argv)
 *       │
 *       ▼  初始化ROS2底层通信层（DDS）
 *   make_shared<MapServer>()
 *       │
 *       ▼  构造节点：声明参数、创建发布器/订阅器/服务
 *   rclcpp::spin(node)
 *       │
 *       ▼  进入事件循环（永不返回，直到节点被关闭）
 *       │  循环中：检查是否有新的消息/服务请求到达
 *       │  如果有 → 调用对应的回调函数
 *       │  如果没有 → 短暂休眠后继续检查
 *       │
 *   rclcpp::shutdown()
 *       │
 *       ▼  清理ROS2资源
 *
 * 【为什么spin是必要的】
 * - ROS2基于回调机制：消息到达时触发回调函数
 * - spin()让节点进入等待状态，持续检查是否有回调需要执行
 * - 如果不调用spin()，回调永远不会被触发
 * - 类比：spin就像手机的主循环，保持开机等待来电
 */

#include "map_server.hpp"
#include <memory>

int main(int argc, char ** argv)
{
  // 初始化ROS2
  // argc/argv用于处理ROS2特定的命令行参数（如--ros-args）
  // 必须在任何ROS2操作之前调用
  rclcpp::init(argc, argv);

  // 创建MapServer节点
  // 使用make_shared而不是new，更高效（减少内存分配次数）
  auto node = std::make_shared<agv_map::MapServer>();

  // 进入事件循环
  // spin()会阻塞当前线程，持续处理回调
  // 直到节点被关闭（Ctrl+C 或 rclcpp::shutdown）
  rclcpp::spin(node);

  // 关闭ROS2
  // 释放DDS通信资源、线程池等
  rclcpp::shutdown();

  return 0;
}
