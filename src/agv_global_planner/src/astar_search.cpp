/**
 * ============================================================
 * astar_search.cpp - A*搜索算法实现
 * ============================================================
 *
 * 本文件实现了A*算法的核心搜索逻辑
 * 仔细阅读每个函数的注释，理解算法的每一步
 */

#include "agv_global_planner/astar_search.hpp"
#include <chrono>  // 用于计时

namespace agv_global_planner
{

// 静态成员初始化：8个方向的偏移量和基础移动代价
constexpr int AstarSearch::dx[8];
constexpr int AstarSearch::dy[8];
constexpr double AstarSearch::move_cost_base[8];

// ============================================================
// 构造函数
// ============================================================
AstarSearch::AstarSearch(
  HeuristicType heuristic_type,
  bool allow_diagonal,
  double cost_factor)
: heuristic_type_(heuristic_type),
  allow_diagonal_(allow_diagonal),
  cost_factor_(cost_factor),
  lethal_cost_(254)  // Costmap2D中外切障碍的代价值
{
}

// ============================================================
// search - 执行A*搜索（核心算法）
// ============================================================
bool AstarSearch::search(
  const std::vector<uint8_t> & costmap,
  unsigned int width,
  unsigned int height,
  const GridCell & start,
  const GridCell & goal,
  std::vector<GridCell> & path)
{
  // ----------------------------------------------------------
  // 前置检查
  // ----------------------------------------------------------
  // 检查起终点是否在地图范围内
  if (!isValid(start.x, start.y, width, height)) {
    return false;  // 起点不在地图内
  }
  if (!isValid(goal.x, goal.y, width, height)) {
    return false;  // 终点不在地图内
  }

  // 检查起终点是否可通行
  if (!isTraversable(start, costmap, width, height)) {
    return false;  // 起点在障碍物上
  }
  if (!isTraversable(goal, costmap, width, height)) {
    return false;  // 终点在障碍物上
  }

  // 特殊情况：起点就是终点
  if (start == goal) {
    path.push_back(start);
    return true;
  }

  // ----------------------------------------------------------
  // 初始化数据结构
  // ----------------------------------------------------------

  // OpenList：待探索的节点，按f值从小到大排序
  // C++的priority_queue默认是大顶堆（最大值在顶部）
  // 我们需要小顶堆（最小值在顶部），所以用greater比较器
  //
  // pair的第一个元素是f值，第二个元素是格子坐标
  // priority_queue按第一个元素排序，f值最小的在顶部
  //
  // 为什么用pair而不是自定义结构？
  // 因为pair已经有默认的比较器，简单高效
  using OpenListEntry = std::pair<double, GridCell>;
  std::priority_queue<
    OpenListEntry,
    std::vector<OpenListEntry>,
    std::greater<OpenListEntry>> open_list;

  // ClosedList：已探索的格子集合
  // 用unordered_set实现，查找时间复杂度O(1)
  std::unordered_set<GridCell, GridCellHash> closed_list;

  // 搜索节点映射表：记录每个格子的搜索信息（g值、父节点等）
  // key=格子坐标，value=搜索节点信息
  std::unordered_map<GridCell, SearchNode, GridCellHash> node_map;

  // ----------------------------------------------------------
  // 将起点加入OpenList
  // ----------------------------------------------------------
  SearchNode start_node;
  start_node.cell = start;
  start_node.g_cost = 0.0;  // 起点到起点的代价为0
  start_node.h_cost = calculateHeuristic(start, goal);
  start_node.f_cost = start_node.g_cost + start_node.h_cost;
  start_node.has_parent = false;

  node_map[start] = start_node;
  open_list.push({start_node.f_cost, start});

  // ----------------------------------------------------------
  // A*主循环
  // ----------------------------------------------------------
  // 循环条件：OpenList不为空
  // 如果OpenList空了还没找到终点，说明不存在可行路径
  while (!open_list.empty()) {

    // 步骤1：从OpenList取出f值最小的节点
    // 这是A*最关键的一步——每次都扩展最有希望的节点
    GridCell current = open_list.top().second;
    open_list.pop();

    // 步骤2：检查是否到达终点
    if (current == goal) {
      // ----------------------------------------------------------
      // 回溯路径
      // ----------------------------------------------------------
      // 从终点沿着parent指针一步步回溯到起点
      // 然后反转路径（因为回溯顺序是终点→起点，但我们需要起点→终点）
      //
      // 路径回溯图示：
      //   S → A → B → C → G（搜索方向）
      //   G.parent=C, C.parent=B, B.parent=A, A.parent=S
      //   回溯：G→C→B→A→S
      //   反转：S→A→B→C→G（这就是最终路径）
      path.clear();
      GridCell trace = goal;
      while (node_map[trace].has_parent) {
        path.push_back(trace);
        trace = node_map[trace].parent;
      }
      path.push_back(start);  // 加入起点
      // 反转路径，使其从起点到终点
      std::reverse(path.begin(), path.end());
      return true;
    }

    // 步骤3：如果当前节点已在ClosedList中，跳过
    // （可能因为更新了已有节点的g值而重复加入OpenList）
    if (closed_list.count(current) > 0) {
      continue;
    }

    // 步骤4：将当前节点加入ClosedList
    closed_list.insert(current);

    // 步骤5：扩展当前节点的邻居
    std::vector<GridCell> neighbors;
    getNeighbors(current, neighbors);

    for (const GridCell & neighbor : neighbors) {
      // 跳过已在ClosedList中的邻居（已经探索过）
      if (closed_list.count(neighbor) > 0) {
        continue;
      }

      // 跳过不在地图范围内的格子
      if (!isValid(neighbor.x, neighbor.y, width, height)) {
        continue;
      }

      // 跳过不可通行的格子（障碍物）
      if (!isTraversable(neighbor, costmap, width, height)) {
        continue;
      }

      // 对角移动检查：必须确保两个相邻正交格都可通行，防止"穿墙角"
      // 从current到neighbor对角移动时，经过(current.x,neighbor.y)和(neighbor.x,current.y)
      int dx = neighbor.x - current.x;
      int dy = neighbor.y - current.y;
      if (dx != 0 && dy != 0) {
        GridCell orth1 = {neighbor.x, current.y};
        GridCell orth2 = {current.x, neighbor.y};
        if (!isTraversable(orth1, costmap, width, height) ||
            !isTraversable(orth2, costmap, width, height)) {
          continue;  // 正交格有障碍物，跳过此对角移动
        }
      }

      // 计算从起点经过current到neighbor的新g值
      // new_g = current的g值 + current到neighbor的移动代价
      double move_cost = getMoveCost(current, neighbor, costmap, width);
      double new_g = node_map[current].g_cost + move_cost;

      // 检查neighbor是否已经在node_map中
      bool in_open = (node_map.count(neighbor) > 0);

      if (!in_open) {
        // 情况1：neighbor从未被探索过
        // 创建新的搜索节点，加入OpenList
        SearchNode neighbor_node;
        neighbor_node.cell = neighbor;
        neighbor_node.g_cost = new_g;
        neighbor_node.h_cost = calculateHeuristic(neighbor, goal);
        neighbor_node.f_cost = new_g + neighbor_node.h_cost;
        neighbor_node.parent = current;
        neighbor_node.has_parent = true;

        node_map[neighbor] = neighbor_node;
        open_list.push({neighbor_node.f_cost, neighbor});

      } else if (new_g < node_map[neighbor].g_cost) {
        // 情况2：neighbor已在OpenList中，但新路径更短
        // 更新neighbor的g值和父节点
        // （不需要从OpenList中删除旧条目，因为closed_list检查会过滤掉旧的）
        //
        // 什么情况下会出现更短路径？
        // 例：
        //   S → A → C（代价10）
        //   S → B → C（代价8）← 发现了更短的路径！
        // 此时更新C的g值和parent

        node_map[neighbor].g_cost = new_g;
        node_map[neighbor].f_cost = new_g + node_map[neighbor].h_cost;
        node_map[neighbor].parent = current;
        node_map[neighbor].has_parent = true;

        // 加入新的OpenList条目（f值已更新）
        // 旧的条目会在被取出时被closed_list检查跳过
        open_list.push({node_map[neighbor].f_cost, neighbor});
      }
      // 情况3：neighbor已在OpenList中，且新路径不更短 → 不做任何操作
    }
  }

  // OpenList为空，说明没有可行路径
  return false;
}

// ============================================================
// calculateHeuristic - 计算启发函数值
// ============================================================
double AstarSearch::calculateHeuristic(
  const GridCell & from,
  const GridCell & to) const
{
  // dx和dy：两点在X和Y方向上的距离（格子数）
  int dx = std::abs(from.x - to.x);
  int dy = std::abs(from.y - to.y);

  switch (heuristic_type_) {
    // --------------------------------------------------------
    // 曼哈顿距离
    // --------------------------------------------------------
    // 原理：假设只能水平/垂直移动，最短距离=水平距离+垂直距离
    // 得名：因为曼哈顿的街道是网格状的，只能横着走或竖着走
    //
    //   S ──→ 3格
    //   │         总距离 = 3 + 2 = 5
    //   ↓ 2格
    //   G
    //
    // 可采纳性：当只允许4方向移动时，h(n)永远不会高估实际最短距离
    // 允许8方向移动时不可采纳（会高估），但仍能使用
    case HeuristicType::MANHATTAN:
      return static_cast<double>(dx + dy);

    // --------------------------------------------------------
    // 欧几里得距离
    // --------------------------------------------------------
    // 原理：两点之间的直线距离（勾股定理）
    //
    //   S ──── 3 ────┐
    //   │            │ 总距离 = √(3²+2²) = √13 ≈ 3.61
    //   │  2         G
    //   └────────────┘
    //
    // 可采纳性：始终可采纳（直线距离是最短可能的距离）
    // 但启发力较弱，搜索范围较大，效率较低
    case HeuristicType::EUCLIDEAN:
      return std::sqrt(static_cast<double>(dx * dx + dy * dy));

    // --------------------------------------------------------
    // 对角线距离（Octile Distance）
    // --------------------------------------------------------
    // 原理：允许8方向移动时的最优启发函数
    // 先走对角线到对齐位置，再走直线
    //
    //   S ╲           对角走min(dx,dy)=2步
    //     ╲ 2步       然后直线走 |dx-dy|=1步
    //       ╲         总距离 = √2×2 + 1×1 ≈ 3.83
    //         → 1步
    //           G
    //
    // 公式：h = √2 × min(dx,dy) + 1 × (max(dx,dy) - min(dx,dy))
    //       = √2 × min(dx,dy) + max(dx,dy) - min(dx,dy)
    //       = (√2-1) × min(dx,dy) + max(dx,dy)
    //
    // 可采纳性：8方向移动时完全可采纳，是最精确的启发函数
    // 推荐用于AGV路径规划（8方向移动是标准配置）
    case HeuristicType::OCTILE:
    default:
    {
      double min_d = static_cast<double>(std::min(dx, dy));
      double max_d = static_cast<double>(std::max(dx, dy));
      return (M_SQRT2 - 1.0) * min_d + max_d;
    }
  }
}

// ============================================================
// getMoveCost - 计算两个相邻格子之间的移动代价
// ============================================================
double AstarSearch::getMoveCost(
  const GridCell & from,
  const GridCell & to,
  const std::vector<uint8_t> & costmap,
  unsigned int width) const
{
  // 判断是直线移动还是对角线移动
  bool is_diagonal = (from.x != to.x && from.y != to.y);

  // 基础移动代价：
  // 水平/垂直 = 1.0
  // 对角线 = √2 ≈ 1.414（对角线距离更远）
  double base_cost = is_diagonal ? M_SQRT2 : 1.0;

  // 获取目标格子的代价地图值
  // costmap是一维数组，通过 row*width+col 访问
  // 注意：这里y对应行号(row)，x对应列号(col)
  size_t idx = static_cast<size_t>(to.y) * width + static_cast<size_t>(to.x);
  uint8_t cell_cost = costmap[idx];

  // 代价地图值的影响
  // costmap值范围0~254，值越高表示越不安全
  // 乘以0.01将其映射到0~2.54的额外代价
  // 例如：costmap=100 → 额外代价1.0
  //       costmap=200 → 额外代价2.0
  //       costmap=0   → 额外代价0.0（自由区域）
  double costmap_penalty = static_cast<double>(cell_cost) * 0.01 * cost_factor_;

  return base_cost + costmap_penalty;
}

// ============================================================
// isValid - 检查格子是否在地图范围内
// ============================================================
bool AstarSearch::isValid(int x, int y, unsigned int width, unsigned int height) const
{
  return x >= 0 && y >= 0 &&
         static_cast<unsigned int>(x) < width &&
         static_cast<unsigned int>(y) < height;
}

// ============================================================
// isTraversable - 检查格子是否可通行
// ============================================================
bool AstarSearch::isTraversable(
  const GridCell & cell,
  const std::vector<uint8_t> & costmap,
  unsigned int width,
  unsigned int height) const
{
  // 格子必须在地图范围内
  if (!isValid(cell.x, cell.y, width, height)) {
    return false;
  }

  // 代价值 >= lethal_cost_ 的格子视为不可通行
  // lethal_cost_ 默认254（Costmap2D的外切障碍）
  // 255是致命障碍（绝对不可通行）
  // 253是内切障碍（AGV底盘会碰到）
  // 254是外切障碍（AGV外轮廓会碰到）
  size_t idx = static_cast<size_t>(cell.y) * width + static_cast<size_t>(cell.x);
  return costmap[idx] < lethal_cost_;
}

// ============================================================
// getNeighbors - 获取邻居格子
// ============================================================
void AstarSearch::getNeighbors(
  const GridCell & cell,
  std::vector<GridCell> & neighbors) const
{
  neighbors.clear();

  // 先加入4个水平/垂直方向邻居
  for (int i = 0; i < 4; ++i) {
    GridCell neighbor;
    neighbor.x = cell.x + dx[i];
    neighbor.y = cell.y + dy[i];
    neighbors.push_back(neighbor);
  }

  // 如果允许对角线移动，加入4个对角方向邻居
  if (allow_diagonal_) {
    for (int i = 4; i < 8; ++i) {
      GridCell neighbor;
      neighbor.x = cell.x + dx[i];
      neighbor.y = cell.y + dy[i];
      neighbors.push_back(neighbor);
    }
  }
}

}  // namespace agv_global_planner
