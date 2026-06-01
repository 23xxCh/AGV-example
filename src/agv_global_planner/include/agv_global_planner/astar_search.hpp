/**
 * ============================================================
 * astar_search.hpp - A*搜索算法头文件
 * ============================================================
 *
 * 【A*算法原理 - 最通俗易懂的解释】
 *
 * 想象你在迷宫中找最短路径：
 *
 *   起点 S ──→ ┌──┬──┬──┬──┐    你站在S点，想要去G点
 *              │  │██│  │  │    ██ 是墙壁（障碍物）
 *              ├──┼──┼──┼──┤
 *              │  │  │██│  │    你可以往上下左右走
 *              ├──┼──┼──┼──┤    每走一步代价为1
 *              │  │  │  │ G│    目标：找到S到G的最短路径
 *              └──┴──┴──┴──┘
 *
 * A*的核心思想：
 * ─────────────────────────────────────────────
 * 对每个待探索的格子，计算两个值：
 *
 *   g(n) = 从起点走到当前格子n的实际代价（已经走过的路）
 *   h(n) = 从当前格子n到终点的预估代价（还没走的路，用启发函数估算）
 *   f(n) = g(n) + h(n) = 总预估代价
 *
 * A*每次选择 f(n) 最小的格子继续探索
 * ─────────────────────────────────────────────
 *
 * 为什么A*能找到最短路径？
 * - g(n) 保证了"已经走过的路是最优的"（类似Dijkstra）
 * - h(n) 保证了"优先探索有希望的方向"（不像Dijkstra盲目搜索）
 * - 当h(n)是"可采纳的"（即永远不会高估实际代价）时，A*保证找到最优路径
 *
 * 常用启发函数：
 * ─────────────────────────────────────────────
 * 1. 曼哈顿距离 (Manhattan Distance):
 *    h = |x1-x2| + |y1-y2|
 *    适用：只能上下左右移动（4方向移动）
 *
 * 2. 欧几里得距离 (Euclidean Distance):
 *    h = sqrt((x1-x2)² + (y1-y2)²)
 *    适用：可以任意方向移动（8方向或连续移动）
 *
 * 3. 对角线距离 (Chebyshev/Octile Distance):
 *    h = max(|x1-x2|, |y1-y2|) + (√2-1) * min(|x1-x2|, |y1-y2|)
 *    适用：8方向移动（上下左右+对角线）
 * ─────────────────────────────────────────────
 *
 * 【A*算法执行过程（图解）】
 *
 *   第1步：把起点加入"待探索列表"（OpenList）
 *
 *   第2步：从OpenList中取出f值最小的节点N
 *          │
 *          ├── 如果N是终点 → 找到了！回溯路径
 *          │
 *          └── 如果N不是终点 →
 *              │
 *              ├── 把N移到"已探索列表"（ClosedList）
 *              │
 *              └── 遍历N的每个邻居M：
 *                  │
 *                  ├── 如果M在ClosedList中 → 跳过
 *                  │
 *                  ├── 如果M是障碍物 → 跳过
 *                  │
 *                  ├── 计算新的g值：new_g = N.g + N到M的移动代价
 *                  │
 *                  ├── 如果M不在OpenList中 → 加入OpenList
 *                  │
 *                  └── 如果M已在OpenList且new_g < M.g → 更新M的g和父节点
 *
 *   第3步：重复第2步，直到OpenList为空（无解）或找到终点
 *
 * 【数据结构选择】
 * - OpenList：优先队列（小顶堆），快速取出f值最小的节点
 * - ClosedList：哈希集合，快速判断节点是否已探索
 * - 路径回溯：每个节点记录父节点，从终点沿父节点链回溯到起点
 */

#ifndef AGV_GLOBAL_PLANNER__ASTAR_SEARCH_HPP_
#define AGV_GLOBAL_PLANNER__ASTAR_SEARCH_HPP_

#include <vector>
#include <cstdint>      // uint8_t 等固定宽度整数类型
#include <cmath>
#include <queue>       // priority_queue 优先队列
#include <unordered_set>  // 哈希集合
#include <unordered_map>  // 哈希映射
#include <algorithm>   // reverse 反转
#include <limits>      // numeric_limits

namespace agv_global_planner
{

// ============================================================
// GridCell - 栅格坐标（地图上的一个格子）
// ============================================================
// 用整数坐标表示地图上的位置
// 对应OccupancyGrid中的 data[row * width + col]
struct GridCell
{
  int x;  // 列号（对应地图的X方向）
  int y;  // 行号（对应地图的Y方向）

  // 相等运算符，用于比较两个格子是否相同
  bool operator==(const GridCell & other) const
  {
    return x == other.x && y == other.y;
  }

  bool operator!=(const GridCell & other) const
  {
    return !(*this == other);
  }

  // 小于运算符，priority_queue的pair<double,GridCell>需要
  // 先按x排序，x相同再按y排序（顺序不重要，只要稳定即可）
  bool operator<(const GridCell & other) const
  {
    if (x != other.x) return x < other.x;
    return y < other.y;
  }
};

// ============================================================
// GridCellHash - GridCell的哈希函数
// ============================================================
// unordered_set/unordered_map 需要哈希函数来快速查找
// 哈希函数将GridCell映射为一个size_t整数
// 好的哈希函数应该均匀分布，减少冲突
struct GridCellHash
{
  size_t operator()(const GridCell & cell) const
  {
    // 简单但有效的哈希：把x和y组合成一个数
    // 原理：每个x值乘以一个大质数，加上y值
    // 73856093和19349663是常用的大质数，减少哈希冲突
    return static_cast<size_t>(cell.x) * 73856093UL ^
           static_cast<size_t>(cell.y) * 19349663UL;
  }
};

// ============================================================
// SearchNode - A*搜索节点
// ============================================================
// 存储A*搜索过程中每个格子的搜索状态
struct SearchNode
{
  GridCell cell;        // 当前格子坐标
  double g_cost;        // g值：从起点到当前格子的实际代价
  double h_cost;        // h值：当前格子到终点的启发式预估代价
  double f_cost;        // f值：f = g + h，总预估代价
  GridCell parent;      // 父节点：用于回溯路径
  bool has_parent;      // 是否有父节点（起点没有父节点）

  SearchNode()
  : g_cost(0.0), h_cost(0.0), f_cost(0.0), has_parent(false) {}
};

// ============================================================
// 启发函数类型枚举
// ============================================================
enum class HeuristicType
{
  MANHATTAN = 0,   // 曼哈顿距离（4方向移动适用）
  EUCLIDEAN = 1,   // 欧几里得距离（连续移动适用）
  OCTILE = 2       // 对角线距离（8方向移动适用，最常用）
};

// ============================================================
// AstarSearch - A*搜索算法类
// ============================================================
class AstarSearch
{
public:
  /**
   * 构造函数
   * @param heuristic_type 启发函数类型
   * @param allow_diagonal 是否允许对角线移动
   * @param cost_factor 代价缩放因子（默认1.0）
   *
   * cost_factor的作用：
   * - 代价地图中，障碍物附近格子代价较高（如50、80）
   * - cost_factor放大这些代价，使A*更倾向于远离障碍物
   * - cost_factor=1.0: 正常代价
   * - cost_factor=2.0: 障碍物附近代价翻倍，路径更远离障碍物
   * - cost_factor越大，路径越安全但越长
   */
  AstarSearch(
    HeuristicType heuristic_type = HeuristicType::OCTILE,
    bool allow_diagonal = true,
    double cost_factor = 1.0);

  /**
   * 执行A*搜索
   * @param costmap 代价地图数据（0~254的代价值，255=致命障碍）
   * @param width 地图宽度（格子数）
   * @param height 地图高度（格子数）
   * @param start 起点（格子坐标）
   * @param goal 终点（格子坐标）
   * @param path 输出：找到的路径（格子坐标序列）
   * @return true=找到路径，false=未找到路径
   *
   * costmap说明：
   * - 这是nav2_costmap_2d生成的代价地图
   * - 与OccupancyGrid的区别：
   *   OccupancyGrid: 0=自由, 100=障碍, -1=未知
   *   Costmap2D:      0=自由, 1~252=代价递增, 253=内切障碍, 254=外切障碍, 255=致命
   * - Costmap2D还包含"膨胀层"：障碍物周围一圈格子代价逐渐递增
   *   这使得AGV不会贴着障碍物走
   */
  bool search(
    const std::vector<uint8_t> & costmap,
    unsigned int width,
    unsigned int height,
    const GridCell & start,
    const GridCell & goal,
    std::vector<GridCell> & path);

  /**
   * 设置致命障碍阈值
   * 代价值 >= 此阈值的格子视为不可通行
   * 默认254（Costmap2D的外切障碍值）
   */
  void setLethalCost(uint8_t lethal_cost) { lethal_cost_ = lethal_cost; }

private:
  // ============================================================
  // 内部方法
  // ============================================================

  /**
   * 计算启发函数值
   * @param from 起始格子
   * @param to 目标格子
   * @return 启发式预估代价
   */
  double calculateHeuristic(const GridCell & from, const GridCell & to) const;

  /**
   * 计算两个相邻格子之间的移动代价
   * @param from 起始格子
   * @param to 目标格子
   * @param costmap 代价地图
   * @param width 地图宽度
   * @return 移动代价
   *
   * 代价计算公式：
   *   水平/垂直移动: cost = 1.0 * cost_factor + costmap_value * 0.01
   *   对角线移动:    cost = √2 * cost_factor + costmap_value * 0.01
   *
   * 公式解释：
   * - 基础移动代价：水平1.0，对角√2≈1.414（距离更远）
   * - costmap_value * 0.01：代价值越高，经过此格子的代价越大
   *   例如costmap_value=100时，额外增加1.0的代价
   * - 这使得A*倾向于选择代价值低的路径（远离障碍物）
   */
  double getMoveCost(
    const GridCell & from,
    const GridCell & to,
    const std::vector<uint8_t> & costmap,
    unsigned int width) const;

  /**
   * 检查格子是否在地图范围内
   */
  bool isValid(int x, int y, unsigned int width, unsigned int height) const;

  /**
   * 检查格子是否可通行（不是致命障碍）
   */
  bool isTraversable(
    const GridCell & cell,
    const std::vector<uint8_t> & costmap,
    unsigned int width,
    unsigned int height) const;

  /**
   * 获取格子的8个（或4个）邻居
   * @param cell 当前格子
   * @param neighbors 输出：邻居列表
   *
   * 8方向邻居布局：
   *   ┌───┬───┬───┐
   *   │NW │ N │NE │   NW=西北  N=北  NE=东北
   *   ├───┼───┼───┤   W =西   ●=当前  E =东
   *   │ W │ ● │ E │   SW=西南  S=南  SE=东南
   *   ├───┼───┼───┤
   *   │SW │ S │SE │
   *   └───┴───┴───┘
   *
   * 对角线移动的额外检查：
   * 从(0,0)对角走到(1,1)时，必须确保(1,0)和(0,1)都可通行
   * 否则AGV可能"穿过"障碍物的角
   */
  void getNeighbors(
    const GridCell & cell,
    std::vector<GridCell> & neighbors) const;

  // ============================================================
  // 成员变量
  // ============================================================

  HeuristicType heuristic_type_;   // 启发函数类型
  bool allow_diagonal_;             // 是否允许对角线移动
  double cost_factor_;              // 代价缩放因子
  uint8_t lethal_cost_;             // 致命障碍阈值

  // 方向偏移数组：8个方向的dx和dy
  // 顺序：右、左、下、上、右下、左下、右上、左上
  // 前4个是水平/垂直方向，后4个是对角线方向
  static constexpr int dx[8] = {1, -1, 0, 0, 1, -1, 1, -1};
  static constexpr int dy[8] = {0, 0, 1, -1, 1, 1, -1, -1};
  // 对应移动代价：水平/垂直=1.0，对角线=√2
  static constexpr double move_cost_base[8] = {
    1.0, 1.0, 1.0, 1.0,       // 水平/垂直
    M_SQRT2, M_SQRT2, M_SQRT2, M_SQRT2  // 对角线 (√2≈1.414)
  };
};

}  // namespace agv_global_planner

#endif  // AGV_GLOBAL_PLANNER__ASTAR_SEARCH_HPP_
