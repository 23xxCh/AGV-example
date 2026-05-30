/**
 * ============================================================
 * map_server.hpp - 地图服务器头文件
 * ============================================================
 *
 * 【模块概述】
 * 地图服务器是AGV路径规划系统的基础模块，负责：
 * 1. 从磁盘加载地图文件（PGM图像 + YAML元数据）
 * 2. 将地图数据发布为ROS2的OccupancyGrid消息
 * 3. 提供地图查询服务（GetMap）
 *
 * 【核心概念 - 栅格地图 (Occupancy Grid)】
 *
 *   什么是栅格地图？
 *   ┌──┬──┬──┬──┬──┬──┬──┬──┐
 *   │ 0│ 0│ 0│ 0│ 0│ 0│ 0│ 0│  ← 自由区域(代价值=0)
 *   ├──┼──┼──┼──┼──┼──┼──┼──┤
 *   │ 0│ 0│50│80│90│ 0│ 0│ 0│  ← 障碍物附近(代价值渐增)
 *   ├──┼──┼──┼──┼──┼──┼──┼──┤
 *   │ 0│50│100│100│100│80│ 0│ 0│  ← 障碍物(代价值=100)
 *   ├──┼──┼──┼──┼──┼──┼──┼──┤
 *   │ 0│ 0│60│100│100│50│ 0│ 0│
 *   ├──┼──┼──┼──┼──┼──┼──┼──┤
 *   │ 0│ 0│ 0│50│ 0│ 0│ 0│ 0│  ← 自由区域
 *   └──┴──┴──┴──┴──┴──┴──┴──┘
 *
 *   - 把连续的物理空间切分成等大小的格子(通常5cm×5cm)
 *   - 每个格子存储一个"占用概率"值：
 *     0   = 完全自由，AGV可以通行
 *     100 = 完全被占用(障碍物)，AGV不可通行
 *     -1  = 未知区域，没有传感器数据覆盖
 *   - 路径规划算法在这个网格上搜索路径
 *
 * 【地图坐标系】
 *
 *   ROS2地图坐标系约定（遵循REP-103标准）：
 *
 *        Y↑
 *         │
 *    (0,H)├──────────┐
 *         │          │  地图图像区域
 *         │  Origin  │  Origin是地图左下角在世界坐标系中的位置
 *         │(0,0)     │  图像像素(0,0)对应地图左下角
 *    (0,0)├──────────┘
 *        ─┼──────────→X
 *         0
 *
 *   - 注意：图像文件(如PGM)的像素原点在左上角
 *   - ROS2地图原点在左下角
 *   - 所以图像的第一行像素 = 地图的最上面一行
 *   - 存储时需要做行翻转：image[row] = map[(height-1-row) * width + col]
 *
 * 【PGM地图文件格式】
 *   PGM(Portable Gray Map)是一种简单的灰度图像格式：
 *   - P5 = 二进制PGM（最常用）
 *   - P2 = 文本PGM
 *   - 像素值0~255，0=黑(自由)，255=白(障碍)，205=未知
 *   - ROS2约定与显示直觉相反：0=黑=自由，254=白=障碍
 *
 * 【YAML元数据文件】
 *   image: warehouse.pgm          # PGM文件名
 *   resolution: 0.05              # 每像素对应多少米(5cm)
 *   origin: [0.0, 0.0, 0.0]       # 地图左下角在世界坐标的位置[x,y,yaw]
 *   negate: 0                      # 是否反转像素值(0=不反转)
 *   occupied_thresh: 0.65          # 像素值超过此阈值的格子视为障碍物
 *   free_thresh: 0.196             # 像素值低于此阈值的格子视为自由区域
 */

#ifndef AGV_MAP__MAP_SERVER_HPP_
#define AGV_MAP__MAP_SERVER_HPP_

#include <string>
#include <vector>
#include <memory>

// ROS2核心库
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"

// 自定义接口
#include "agv_interfaces/srv/get_map.hpp"

// YAML解析库
#include "yaml-cpp/yaml.h"

namespace agv_map
{

// ============================================================
// MapMetadata - 地图元数据结构
// ============================================================
// 从YAML文件解析出的地图配置信息
// 这些信息描述了地图如何从像素坐标转换到物理世界坐标
struct MapMetadata
{
  std::string image_path;     // PGM图像文件路径
  double resolution;          // 分辨率：每像素对应的米数（如0.05 = 5cm/像素）
  double origin_x;            // 地图左下角在世界坐标系的X坐标（米）
  double origin_y;            // 地图左下角在世界坐标系的Y坐标（米）
  double origin_yaw;          // 地图左下角的朝向角（弧度，通常为0）
  bool negate;                // 是否反转像素含义（true=白=自由，黑=障碍）
  double occupied_thresh;     // 障碍物阈值：像素归一化值 > 此值 = 障碍物
  double free_thresh;         // 自由区域阈值：像素归一化值 < 此值 = 自由
};

// ============================================================
// MapServer - 地图服务器类
// ============================================================
class MapServer : public rclcpp::Node
{
public:
  /**
   * 构造函数
   * @param options ROS2节点选项，包含节点名、参数等
   *
   * 工作流程：
   * 1. 读取YAML配置文件路径参数
   * 2. 解析YAML获取地图元数据
   * 3. 加载PGM图像数据
   * 4. 将图像数据转换为OccupancyGrid格式
   * 5. 创建话题发布器和服务服务器
   * 6. 发布地图数据
   */
  explicit MapServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * 获取加载的地图数据（供其他模块使用）
   * @return OccupancyGrid格式的地图数据
   */
  const nav_msgs::msg::OccupancyGrid & getMap() const { return map_; }

private:
  // ============================================================
  // 核心功能方法
  // ============================================================

  /**
   * 解析YAML地图配置文件
   * @param yaml_path YAML文件的完整路径
   * @return 解析出的地图元数据
   *
   * YAML文件示例：
   *   image: warehouse.pgm
   *   resolution: 0.05
   *   origin: [0.0, 0.0, 0.0]
   *   negate: 0
   *   occupied_thresh: 0.65
   *   free_thresh: 0.196
   */
  MapMetadata parseYaml(const std::string & yaml_path);

  /**
   * 加载PGM图像文件
   * @param pgm_path PGM文件路径
   * @param width 输出：图像宽度（像素数）
   * @param height 输出：图像高度（像素数）
   * @return 像素值数组（0~255），按行优先排列
   *
   * PGM文件格式解析：
   *   P5\n          ← 魔数，表示二进制PGM格式
   *   800 600\n     ← 宽度 高度
   *   255\n         ← 最大像素值
   *   [二进制像素数据]  ← width*height个字节
   */
  std::vector<uint8_t> loadPGM(const std::string & pgm_path, unsigned int & width, unsigned int & height);

  /**
   * 将PGM像素数据转换为ROS2 OccupancyGrid格式
   * @param pixels PGM像素数组
   * @param width 图像宽度
   * @param height 图像高度
   * @param meta 地图元数据
   *
   * 转换规则（ROS2标准）：
   *   像素归一化值 = pixel_value / 255.0
   *
   *   如果 negate = false（默认）：
   *     归一化值 > occupied_thresh → OccupancyGrid值 = 100（障碍物）
   *     归一化值 < free_thresh     → OccupancyGrid值 = 0（自由区域）
   *     其他                       → OccupancyGrid值 = 按比例计算 (1~99)
   *
   *   特殊像素值：
   *     205 (0xCD) → OccupancyGrid值 = -1（未知区域）
   *
   *   OccupancyGrid数据布局：
   *     data[row * width + col]
   *     (0,0)是地图左下角，但图像(0,0)是左上角
   *     所以需要翻转行顺序
   */
  void convertToOccupancyGrid(
    const std::vector<uint8_t> & pixels,
    unsigned int width,
    unsigned int height,
    const MapMetadata & meta);

  /**
   * 处理GetMap服务请求
   * @param request 请求（包含期望的坐标系）
   * @param response 响应（包含地图数据）
   *
   * 这是ROS2 Service的回调函数，当客户端调用GetMap服务时触发
   */
  void handleGetMapRequest(
    const std::shared_ptr<agv_interfaces::srv::GetMap::Request> request,
    std::shared_ptr<agv_interfaces::srv::GetMap::Response> response);

  /**
   * 发布地图数据到话题
   * 使用latched=True，确保新订阅者也能收到地图（即使只发布一次）
   */
  void publishMap();

  // ============================================================
  // 成员变量
  // ============================================================

  // 加载后的地图数据
  nav_msgs::msg::OccupancyGrid map_;

  // 话题发布器：持续发布地图数据
  // "latched"含义：发布一次后，新订阅者连接时自动收到最近一次的消息
  // 这对于不频繁变化的数据（如静态地图）非常有用
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

  // 服务服务器：响应地图查询请求
  rclcpp::Service<agv_interfaces::srv::GetMap>::SharedPtr map_service_;

  // 地图YAML配置文件路径
  std::string yaml_path_;
};

}  // namespace agv_map

#endif  // AGV_MAP__MAP_SERVER_HPP_
