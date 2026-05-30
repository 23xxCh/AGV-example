/**
 * ============================================================
 * dwa_search.hpp - DWA（动态窗口法）算法头文件
 * ============================================================
 *
 * 【DWA算法原理 - 最通俗易懂的解释】
 *
 * 想象你在一个拥挤的走廊里走路：
 * - 你知道目的地在哪里（全局路径）
 * - 你能看到周围的人和障碍物（代价地图）
 * - 你的腿有速度限制（最大速度）
 * - 你不能瞬间加速或减速（加速度限制）
 *
 * DWA的思路是：
 * 1. 想象你当前能采取的所有可能的行走方式（速度组合）
 * 2. 对每种方式，模拟你往前走几步会走到哪里
 * 3. 评估哪种方式最好（离目标近、不撞到人、走得快）
 * 4. 选择最好的那种方式走一步
 * 5. 下一秒再重新想一遍
 *
 * 【核心概念】
 *
 * 动态窗口 (Dynamic Window):
 *   你当前能选择的速度范围，受限于：
 *   - 机器人硬件的物理限制（最大/最小速度）
 *   - 当前速度和加速度（不能瞬间变速）
 *   ┌─────────────────────────────────┐
 *   │ Vs: 硬件限速 [v_min, v_max]     │ ← 机器人能跑多快
 *   │ Vd: 加速度可达 [v-a*dt, v+a*dt] │ ← 当前能变到多快
 *   │ Dw: 两者的交集                   │ ← 实际可选范围
 *   └─────────────────────────────────┘
 *
 * 轨迹模拟:
 *   对每个候选速度(v,ω)，假设保持这个速度走一段时间，
 *   计算机器人会走出什么样的路径
 *
 *   差分运动学模型（简化版）:
 *   x_new     = x     + v * cos(θ) * dt
 *   y_new     = y     + v * sin(θ) * dt
 *   theta_new = theta + ω * dt
 *
 * 评分函数:
 *   对每条候选轨迹打分，公式为：
 *   G = α * heading(朝向目标) + β * clearance(远离障碍) + γ * velocity(速度)
 *
 *   α (path_distance_bias): 路径跟踪权重 - 越大越紧贴全局路径
 *   β (goal_distance_bias): 目标朝向权重 - 越大越朝向目标走
 *   γ (occdist_scale):      避障权重     - 越大越远离障碍物
 *
 * 【与A*的区别】
 * A*在"地图空间"搜索，找到全局最优路径（静态规划）
 * DWA在"速度空间"搜索，找到局部最优速度（实时避障）
 * 两者配合使用：A*提供全局路径，DWA沿路径行驶并避开动态障碍
 */

#ifndef AGV_LOCAL_PLANNER__DWA_SEARCH_HPP_
#define AGV_LOCAL_PLANNER__DWA_SEARCH_HPP_

#include <vector>
#include <cmath>
#include <cstdint>
#include <limits>
#include <algorithm>

namespace agv_local_planner
{

// ============================================================
// 数据结构定义
// ============================================================

/**
 * 二维位姿 (x, y, theta)
 * 用于表示机器人在地图中的位置和朝向
 */
struct Pose2D
{
  double x;      // X坐标（米）
  double y;      // Y坐标（米）
  double theta;  // 朝向角（弧度，0=朝右，逆时针为正）

  Pose2D() : x(0.0), y(0.0), theta(0.0) {}
  Pose2D(double x, double y, double theta) : x(x), y(y), theta(theta) {}
};

/**
 * 速度 (v, omega)
 * v: 线速度（米/秒），正值前进，负值后退
 * omega: 角速度（弧度/秒），正值逆时针，负值顺时针
 */
struct Velocity
{
  double v;      // 线速度（米/秒）
  double omega;  // 角速度（弧度/秒）

  Velocity() : v(0.0), omega(0.0) {}
  Velocity(double v, double omega) : v(v), omega(omega) {}
};

/**
 * 动态窗口
 * 描述当前可选的速度范围
 */
struct DynamicWindow
{
  double v_min;      // 最小线速度
  double v_max;      // 最大线速度
  double omega_min;  // 最小角速度
  double omega_max;  // 最大角速度
};

/**
 * 模拟轨迹
 * 机器人以某个速度行驶一段时间后的位置序列
 */
struct Trajectory
{
  std::vector<Pose2D> poses;  // 轨迹上的所有位姿点
  double v;                    // 该轨迹使用的线速度
  double omega;                // 该轨迹使用的角速度

  // 轨迹末端位姿（用于评分）
  Pose2D endPose() const
  {
    return poses.empty() ? Pose2D() : poses.back();
  }
};

/**
 * DWA算法参数
 * 每个参数都有详细注释说明其含义和调参建议
 */
struct DWAParams
{
  // ---- 速度限制 ----
  double max_vel_x;       // 最大前进速度（m/s），仓储AGV典型值：0.3~1.0
  double min_vel_x;       // 最小前进速度（m/s），负值允许后退
  double max_vel_theta;   // 最大角速度（rad/s），仓储AGV典型值：0.5~2.0
  double min_vel_theta;   // 最小角速度（rad/s），通常为-max_vel_theta

  // ---- 加速度限制 ----
  double acc_lim_x;       // 最大线加速度（m/s²），决定加速多快
  double acc_lim_theta;   // 最大角加速度（rad/s²），决定转向多快

  // ---- 模拟参数 ----
  double sim_time;        // 前向模拟时长（秒），典型值1.0~3.0
  double sim_granularity; // 模拟时间步长（秒），越小越精确但越慢
  int vx_samples;         // 线速度采样数，典型值5~20
  int vtheta_samples;     // 角速度采样数，典型值10~40

  // ---- 评分权重 ----
  double path_distance_bias;  // α：路径跟踪权重，越大越紧贴全局路径
  double goal_distance_bias;  // β：目标朝向权重，越大越朝向目标
  double occdist_scale;       // γ：避障权重，越大越远离障碍物

  // ---- 目标到达判定 ----
  double goal_tolerance_xy;   // 到达目标的位置容差（米）
  double goal_tolerance_yaw;  // 到达目标的角度容差（弧度）

  // ---- 停止条件 ----
  double stop_time_buffer;    // 停止前的缓冲时间（秒）
  double oscillation_dist;    // 振荡检测距离（米）
};

// ============================================================
// DWASearch - DWA搜索算法类
// ============================================================
class DWASearch
{
public:
  /**
   * 构造函数
   * @param params DWA算法参数
   */
  explicit DWASearch(const DWAParams & params = DWAParams());

  /**
   * 更新参数（运行时调参用）
   */
  void setParams(const DWAParams & params) { params_ = params; }

  /**
   * DWA主入口：计算最优速度指令
   *
   * @param robot_pose   机器人当前位姿 (x, y, theta)
   * @param robot_vel    机器人当前速度 (v, omega)
   * @param goal_pose    目标位姿（全局路径上的局部目标点）
   * @param costmap      代价地图数据（一维数组，row-major排列）
   * @param map_width    地图宽度（格子数）
   * @param map_height   地图高度（格子数）
   * @param map_resolution 地图分辨率（米/像素）
   * @param map_origin_x 地图原点X坐标
   * @param map_origin_y 地图原点Y坐标
   * @return 最优速度指令 (v, omega)
   *
   * 【调用流程】
   * 每个控制周期（如10Hz）调用一次，返回当前应该执行的速度
   */
  Velocity computeVelocity(
    const Pose2D & robot_pose,
    const Velocity & robot_vel,
    const Pose2D & goal_pose,
    const std::vector<uint8_t> & costmap,
    unsigned int map_width,
    unsigned int map_height,
    double map_resolution,
    double map_origin_x,
    double map_origin_y);

  /**
   * 检查是否已到达目标
   */
  bool isGoalReached(
    const Pose2D & robot_pose,
    const Pose2D & goal_pose) const;

  /**
   * 获取最近的候选轨迹（用于可视化）
   */
  const std::vector<Trajectory> & getRecentTrajectories() const { return recent_trajectories_; }

private:
  // ============================================================
  // 内部算法方法
  // ============================================================

  /**
   * 步骤1：计算动态窗口
   *
   * 动态窗口 = 硬件限速 ∩ 当前速度可达
   *
   * 硬件限速 Vs: [min_vel_x, max_vel_x] × [min_vel_theta, max_vel_theta]
   * 当前可达 Vd: [v-acc*dt, v+acc*dt] × [ω-αcc_θ*dt, ω+αcc_θ*dt]
   *
   * 取交集确保选出的速度在物理上可行
   */
  DynamicWindow calculateDynamicWindow(const Velocity & current_vel) const;

  /**
   * 步骤2：前向模拟轨迹
   *
   * 使用差分运动学模型，从当前位置开始，以给定速度前进sim_time秒：
   *   for t = 0 to sim_time:
   *     x     += v * cos(θ) * dt
   *     y     += v * sin(θ) * dt
   *     θ     += ω * dt
   *     记录 (x, y, θ)
   *
   * @param start_pose 起始位姿
   * @param v          线速度
   * @param omega      角速度
   * @return 模拟出的轨迹
   */
  Trajectory simulateTrajectory(
    const Pose2D & start_pose,
    double v,
    double omega) const;

  /**
   * 步骤3：评分函数
   *
   * G = α * heading + β * clearance + γ * velocity
   *
   * 三个分量都归一化到[0,1]后加权求和
   *
   * @param trajectory 候选轨迹
   * @param goal       目标位姿
   * @param costmap    代价地图
   * @param map_*      地图参数
   * @return 评分（越高越好）
   */
  double scoreTrajectory(
    const Trajectory & trajectory,
    const Pose2D & goal,
    const std::vector<uint8_t> & costmap,
    unsigned int map_width,
    unsigned int map_height,
    double map_resolution,
    double map_origin_x,
    double map_origin_y) const;

  /**
   * 计算heading分量：轨迹末端朝向与目标方向的对齐程度
   *
   * heading = π - |angle_to_goal - trajectory_heading|
   * 归一化到[0,1]，1表示完美朝向目标
   */
  double computeHeadingScore(
    const Pose2D & end_pose,
    const Pose2D & goal) const;

  /**
   * 计算clearance分量：轨迹与最近障碍物的距离
   *
   * 遍历轨迹上每个点，查询代价地图，找到最近障碍物距离
   * 如果任何点在障碍物上，返回-1（碰撞，该轨迹不可选）
   *
   * @return clearance值（距离/最大可能距离），-1表示碰撞
   */
  double computeClearanceScore(
    const Trajectory & trajectory,
    const std::vector<uint8_t> & costmap,
    unsigned int map_width,
    unsigned int map_height,
    double map_resolution,
    double map_origin_x,
    double map_origin_y) const;

  /**
   * 计算velocity分量：前进速度的归一化值
   *
   * velocity = |v| / max_vel_x
   * 鼓励机器人尽可能快地移动（在安全前提下）
   */
  double computeVelocityScore(double v) const;

  /**
   * 世界坐标转格子坐标（与agv_global_planner中相同）
   */
  bool worldToGrid(
    double wx, double wy,
    unsigned int map_width, unsigned int map_height,
    double map_resolution, double map_origin_x, double map_origin_y,
    int & grid_x, int & grid_y) const;

  /**
   * 检查某个世界坐标点是否在障碍物上
   * @return true=可通行，false=障碍物
   */
  bool isTraversable(
    double wx, double wy,
    const std::vector<uint8_t> & costmap,
    unsigned int map_width,
    unsigned int map_height,
    double map_resolution,
    double map_origin_x,
    double map_origin_y) const;

  // ============================================================
  // 成员变量
  // ============================================================
  DWAParams params_;
  std::vector<Trajectory> recent_trajectories_;  // 最近的候选轨迹（可视化用）
};

}  // namespace agv_local_planner

#endif  // AGV_LOCAL_PLANNER__DWA_SEARCH_HPP_
