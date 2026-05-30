/**
 * ============================================================
 * map_server.cpp - 地图服务器实现
 * ============================================================
 *
 * 【实现概述】
 * 本文件实现了MapServer类的所有方法，包括：
 * - YAML配置文件解析
 * - PGM图像文件加载
 * - 像素数据到OccupancyGrid的转换
 * - 地图发布与服务响应
 *
 * 【技术要点】
 * - ROS2节点继承自rclcpp::Node，通过构造函数初始化
 * - 使用parameters_declare声明参数，通过get_parameter读取
 * - Publisher的QoS (Quality of Service) 控制通信质量：
 *   - RELIABLE: 保证消息送达（适合重要数据如地图）
 *   - TRANSIENT_LOCAL: latched语义，新订阅者收到最近消息
 * - Service回调在单独线程中执行，不会阻塞主线程
 */

#include "map_server.hpp"
#include <fstream>
#include <cstring>
#include <filesystem>

namespace agv_map
{

// ============================================================
// 构造函数
// ============================================================
MapServer::MapServer(const rclcpp::NodeOptions & options)
: Node("map_server", options)  // 节点名称设为 "map_server"
{
  // ----------------------------------------------------------
  // 步骤1：声明和获取ROS2参数
  // ----------------------------------------------------------
  // ROS2参数类似于命令行参数，可以在启动时通过YAML文件或命令行覆盖
  // 声明参数格式：declare_parameter(名称, 默认值)
  // 读取参数格式：get_parameter(名称, 输出变量)

  this->declare_parameter("yaml_path", std::string(""));
  this->get_parameter("yaml_path", yaml_path_);

  // 检查YAML路径是否有效
  if (yaml_path_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "必须指定地图YAML配置文件路径！使用参数: yaml_path");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "加载地图配置文件: %s", yaml_path_.c_str());

  // ----------------------------------------------------------
  // 步骤2：解析YAML配置文件，获取地图元数据
  // ----------------------------------------------------------
  MapMetadata meta = parseYaml(yaml_path_);

  // ----------------------------------------------------------
  // 步骤3：加载PGM图像文件
  // ----------------------------------------------------------
  unsigned int width = 0, height = 0;
  std::vector<uint8_t> pixels = loadPGM(meta.image_path, width, height);

  if (pixels.empty()) {
    RCLCPP_ERROR(this->get_logger(), "地图图像加载失败: %s", meta.image_path.c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "地图图像加载成功: %dx%d 像素", width, height);

  // ----------------------------------------------------------
  // 步骤4：将像素数据转换为OccupancyGrid
  // ----------------------------------------------------------
  convertToOccupancyGrid(pixels, width, height, meta);

  // ----------------------------------------------------------
  // 步骤5：创建话题发布器
  // ----------------------------------------------------------
  // QoS配置说明：
  // - keep_last(1): 只保留最新1条消息（地图不频繁更新，保留1条足够）
  // - reliable: 保证消息可靠送达，不丢失
  // - transient_local: 类似ROS1的latched=True，新订阅者自动收到最新地图
  //
  // 为什么用transient_local？
  // 因为地图通常只在启动时发布一次，之后不再更新
  // 如果没有latched，后启动的节点订阅时会错过地图
  // 有了latched，新订阅者连接时自动收到缓存的最新地图
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
    .reliable()
    .transient_local();

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", qos);

  // ----------------------------------------------------------
  // 步骤6：创建服务服务器
  // ----------------------------------------------------------
  // Service的回调函数在独立的回调线程中执行
  // 不会阻塞主线程（spin线程）
  // 回调签名：void(RequestSharedPtr, ResponseSharedPtr)
  map_service_ = this->create_service<agv_interfaces::srv::GetMap>(
    "get_map",
    std::bind(&MapServer::handleGetMapRequest, this,
              std::placeholders::_1, std::placeholders::_2));

  // ----------------------------------------------------------
  // 步骤7：发布地图
  // ----------------------------------------------------------
  publishMap();

  RCLCPP_INFO(this->get_logger(),
    "地图服务器启动完成！发布地图: %dx%d, 分辨率: %.3f m/像素, 原点: (%.2f, %.2f)",
    map_.info.width, map_.info.height, map_.info.resolution,
    map_.info.origin.position.x, map_.info.origin.position.y);
}

// ============================================================
// parseYaml - 解析YAML地图配置文件
// ============================================================
MapMetadata MapServer::parseYaml(const std::string & yaml_path)
{
  MapMetadata meta;

  // YAML::LoadFile 读取并解析YAML文件
  // YAML文件本质是键值对的层级结构
  try {
    YAML::Node doc = YAML::LoadFile(yaml_path);

    // 解析图像文件路径
    // YAML中写的是相对路径（相对于YAML文件所在目录）
    // 需要拼接成绝对路径，这样无论在哪里运行程序都能找到图像
    meta.image_path = doc["image"].as<std::string>();

    // 如果image_path是相对路径，拼接YAML文件所在目录
    namespace fs = std::filesystem;
    fs::path image_fs_path(meta.image_path);
    if (image_fs_path.is_relative()) {
      fs::path yaml_dir = fs::path(yaml_path).parent_path();
      meta.image_path = (yaml_dir / image_fs_path).string();
    }

    // 解析分辨率：每像素对应的米数
    // 例如0.05表示每个格子5cm×5cm
    // 分辨率越小，地图越精细，但占用内存越大
    // 典型值：0.02(2cm)~0.1(10cm)
    meta.resolution = doc["resolution"].as<double>();

    // 解析原点：地图左下角在世界坐标系的位置[x, y, yaw]
    // origin是一个包含3个元素的数组
    // x, y: 世界坐标（米）
    // yaw: 旋转角（弧度），通常为0
    auto origin = doc["origin"];
    meta.origin_x = origin[0].as<double>();
    meta.origin_y = origin[1].as<double>();
    meta.origin_yaw = origin[2].as<double>();

    // 解析阈值参数
    // negate: 是否反转像素含义（0=不反转，1=反转）
    // 正常情况：像素值0=黑=自由，255=白=障碍
    // 反转后：像素值0=黑=障碍，255=白=自由
    meta.negate = doc["negate"].as<int>() != 0;

    // occupied_thresh: 像素归一化值超过此阈值 → 障碍物
    // 归一化值 = pixel_value / 255.0
    // 例如0.65表示像素值 > 165的格子视为障碍物
    meta.occupied_thresh = doc["occupied_thresh"].as<double>();

    // free_thresh: 像素归一化值低于此阈值 → 自由区域
    // 例如0.196表示像素值 < 50的格子视为可自由通行
    meta.free_thresh = doc["free_thresh"].as<double>();

    RCLCPP_INFO(this->get_logger(),
      "YAML解析完成: 图像=%s, 分辨率=%.3f, 原点=(%.2f,%.2f,%.2f)",
      meta.image_path.c_str(), meta.resolution,
      meta.origin_x, meta.origin_y, meta.origin_yaw);

  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "YAML解析错误: %s", e.what());
  }

  return meta;
}

// ============================================================
// loadPGM - 加载PGM图像文件
// ============================================================
std::vector<uint8_t> MapServer::loadPGM(
  const std::string & pgm_path,
  unsigned int & width,
  unsigned int & height)
{
  std::vector<uint8_t> pixels;

  // 以二进制模式打开文件
  // std::ios::binary 告诉C++不要对换行符做任何转换
  std::ifstream file(pgm_path, std::ios::binary);

  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "无法打开PGM文件: %s", pgm_path.c_str());
    return pixels;
  }

  // ----------------------------------------------------------
  // 解析PGM文件头
  // ----------------------------------------------------------
  // PGM文件格式：
  //   第一行: "P5" 或 "P2"（魔数，标识文件类型）
  //   P5 = 二进制格式（像素数据是原始字节）
  //   P2 = 文本格式（像素数据是ASCII数字）
  //   后续行: 宽度 高度
  //   再后: 最大像素值（通常255）
  //   最后: 像素数据

  std::string magic;  // 魔数
  file >> magic;

  if (magic != "P5" && magic != "P2") {
    RCLCPP_ERROR(this->get_logger(), "不支持的PGM格式: %s (仅支持P5和P2)", magic.c_str());
    return pixels;
  }

  bool is_binary = (magic == "P5");

  // 跳过注释行（以#开头）
  // PGM文件允许在头部插入注释
  std::string line;
  while (std::getline(file, line)) {
    // 找到第一个非注释行（宽度 高度）
    if (line.empty() || line[0] == '#') {
      continue;
    }
    // 尝试解析为宽度高度
    std::istringstream iss(line);
    if (!(iss >> width >> height)) {
      continue;  // 这行不是宽高，继续找
    }
    break;
  }

  // 读取最大像素值
  int max_val;
  file >> max_val;

  // 跳过头部后的一个空白字符（换行符）
  // PGM规范：最大值后面跟一个空白字符，然后是像素数据
  file.ignore(1);

  // ----------------------------------------------------------
  // 读取像素数据
  // ----------------------------------------------------------
  size_t pixel_count = static_cast<size_t>(width) * static_cast<size_t>(height);
  pixels.resize(pixel_count);

  if (is_binary) {
    // P5二进制格式：直接读取原始字节
    // 每个像素占1字节（当max_val<=255时）
    file.read(reinterpret_cast<char *>(pixels.data()), pixel_count);
  } else {
    // P2文本格式：像素值是ASCII数字，用空格或换行分隔
    for (size_t i = 0; i < pixel_count; ++i) {
      int val;
      file >> val;
      pixels[i] = static_cast<uint8_t>(val);
    }
  }

  // 检查是否读取了足够的像素
  if (pixels.size() != pixel_count) {
    RCLCPP_ERROR(this->get_logger(),
      "像素数据不完整: 期望 %zu 像素, 实际读取 %zu", pixel_count, pixels.size());
    pixels.clear();
  }

  return pixels;
}

// ============================================================
// convertToOccupancyGrid - 像素数据转OccupancyGrid
// ============================================================
void MapServer::convertToOccupancyGrid(
  const std::vector<uint8_t> & pixels,
  unsigned int width,
  unsigned int height,
  const MapMetadata & meta)
{
  // ----------------------------------------------------------
  // 填充地图元数据
  // ----------------------------------------------------------
  // map_.info 是 nav_msgs/MapMetaData 类型，包含：
  // - resolution: 分辨率（米/像素）
  // - width, height: 地图尺寸（像素数）
  // - origin: 地图左下角在世界坐标系的位姿

  // 设置时间戳为当前时间
  map_.header.stamp = this->now();
  // 坐标系名称，"map"是ROS2中全局固定坐标系的约定名称
  map_.header.frame_id = "map";

  map_.info.resolution = meta.resolution;
  map_.info.width = width;
  map_.info.height = height;

  // 设置地图原点（左下角在世界坐标系的位置）
  map_.info.origin.position.x = meta.origin_x;
  map_.info.origin.position.y = meta.origin_y;
  map_.info.origin.position.z = 0.0;  // 2D地图，Z坐标始终为0

  // 用四元数表示朝向角（yaw角转四元数）
  // 四元数是3D旋转的数学表示，避免万向锁问题
  // 对于2D地图，只有绕Z轴的旋转(yaw)有意义
  // 四元数与yaw的转换公式：
  //   w = cos(yaw/2), x = 0, y = 0, z = sin(yaw/2)
  map_.info.origin.orientation.w = std::cos(meta.origin_yaw / 2.0);
  map_.info.origin.orientation.x = 0.0;
  map_.info.origin.orientation.y = 0.0;
  map_.info.origin.orientation.z = std::sin(meta.origin_yaw / 2.0);

  // ----------------------------------------------------------
  // 转换像素值为OccupancyGrid值
  // ----------------------------------------------------------
  // OccupancyGrid.data 是 int8[] 数组
  // 每个格子的值含义：
  //   -1  = 未知 (unknown)
  //   0   = 完全自由 (free)
  //   100 = 完全被占用/障碍物 (occupied)
  //   1~99 = 不确定程度

  map_.data.resize(width * height);

  for (unsigned int row = 0; row < height; ++row) {
    for (unsigned int col = 0; col < width; ++col) {
      // PGM图像的像素索引（行优先，左上角为原点）
      size_t pixel_idx = static_cast<size_t>(row) * width + col;

      // OccupancyGrid的索引（行优先，左下角为原点）
      // 关键：图像行0在顶部，地图行0在底部
      // 所以图像的 row 行 = 地图的 (height-1-row) 行
      size_t map_idx = static_cast<size_t>(height - 1 - row) * width + col;

      uint8_t pixel = pixels[pixel_idx];

      // 特殊值：205(0xCD) 表示未知区域
      // 这是ROS2地图工具（如gmapping, cartographer）的约定
      if (pixel == 205) {
        map_.data[map_idx] = -1;  // -1 = 未知
        continue;
      }

      // 将像素值归一化到 [0.0, 1.0]
      // pixel值0~255，归一化后0=黑，1=白
      double normalized = static_cast<double>(pixel) / 255.0;

      // 如果negate=true，反转归一化值
      // 某些地图工具生成的图像白=自由，黑=障碍，需要反转
      if (meta.negate) {
        normalized = 1.0 - normalized;
      }

      // 根据阈值分类
      int8_t occupancy_val;
      if (normalized > meta.occupied_thresh) {
        // 归一化值超过障碍阈值 → 障碍物
        occupancy_val = 100;
      } else if (normalized < meta.free_thresh) {
        // 归一化值低于自由阈值 → 自由区域
        occupancy_val = 0;
      } else {
        // 在两个阈值之间 → 按比例映射到1~99
        // 线性插值：
        //   free_thresh → 0
        //   occupied_thresh → 100
        //   中间值按比例
        double ratio = (normalized - meta.free_thresh) /
                       (meta.occupied_thresh - meta.free_thresh);
        occupancy_val = static_cast<int8_t>(ratio * 100.0);

        // 钳位到 [1, 99] 范围
        // 避免与0(自由)和100(障碍)混淆
        occupancy_val = std::max<int8_t>(1, std::min<int8_t>(99, occupancy_val));
      }

      map_.data[map_idx] = occupancy_val;
    }
  }

  // 统计地图各类型格子数量，方便调试
  int free_count = 0, occupied_count = 0, unknown_count = 0;
  for (const auto & val : map_.data) {
    if (val == 0) free_count++;
    else if (val == 100) occupied_count++;
    else if (val == -1) unknown_count++;
  }

  RCLCPP_INFO(this->get_logger(),
    "地图转换完成: 自由=%d, 障碍=%d, 未知=%d (总格子=%zu)",
    free_count, occupied_count, unknown_count, map_.data.size());
}

// ============================================================
// handleGetMapRequest - 处理GetMap服务请求
// ============================================================
void MapServer::handleGetMapRequest(
  const std::shared_ptr<agv_interfaces::srv::GetMap::Request> request,
  std::shared_ptr<agv_interfaces::srv::GetMap::Response> response)
{
  (void)request;  // 暂时不使用请求参数

  // 直接返回缓存的地图数据
  // 地图在启动时加载一次，后续查询直接返回
  response->map = map_;

  RCLCPP_DEBUG(this->get_logger(), "响应GetMap服务请求");
}

// ============================================================
// publishMap - 发布地图数据
// ============================================================
void MapServer::publishMap()
{
  // 更新时间戳
  map_.header.stamp = this->now();

  // 发布地图
  // 由于QoS设为transient_local，这条消息会被缓存
  // 后来的订阅者也能收到
  map_pub_->publish(map_);
}

}  // namespace agv_map
