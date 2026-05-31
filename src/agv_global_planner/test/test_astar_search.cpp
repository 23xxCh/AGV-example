/**
 * test_astar_search.cpp - A*搜索算法单元测试
 *
 * 测试用例：
 * 1. 空旷地图 - 直线路径
 * 2. 障碍物绕行
 * 3. 无解情况（完全封锁）
 * 4. 对角线移动
 * 5. 大地图性能
 */

#include <gtest/gtest.h>
#include "agv_global_planner/astar_search.hpp"

using namespace agv_global_planner;

class AstarSearchTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    astar_ = std::make_unique<AstarSearch>(HeuristicType::OCTILE, true, 1.0);
  }

  // 创建空旷地图（全部可通行）
  std::vector<uint8_t> createEmptyMap(unsigned int w, unsigned int h)
  {
    return std::vector<uint8_t>(w * h, 0);
  }

  // 在地图上设置障碍物
  void setObstacle(std::vector<uint8_t> & map, unsigned int w,
                    int x, int y, uint8_t cost = 254)
  {
    if (x >= 0 && x < static_cast<int>(w) && y >= 0) {
      map[y * w + x] = cost;
    }
  }

  std::unique_ptr<AstarSearch> astar_;
};

// 测试1：空旷地图上的直线路径
TEST_F(AstarSearchTest, StraightLinePath)
{
  auto map = createEmptyMap(10, 10);
  GridCell start = {1, 1};
  GridCell goal = {8, 1};
  std::vector<GridCell> path;

  bool found = astar_->search(map, 10, 10, start, goal, path);

  EXPECT_TRUE(found);
  ASSERT_FALSE(path.empty());
  EXPECT_EQ(path.front(), start);
  EXPECT_EQ(path.back(), goal);
  // 直线路径长度应该接近7（8-1）
  EXPECT_GE(path.size(), 7);
  EXPECT_LE(path.size(), 9);
}

// 测试2：绕过障碍物
TEST_F(AstarSearchTest, ObstacleAvoidance)
{
  auto map = createEmptyMap(20, 20);

  // 在x=10设置墙壁，但y=15处留一个缺口
  for (int y = 0; y < 20; ++y) {
    if (y < 14 || y > 16) {
      setObstacle(map, 20, 10, y);
    }
  }

  GridCell start = {5, 10};
  GridCell goal = {15, 10};
  std::vector<GridCell> path;

  bool found = astar_->search(map, 20, 20, start, goal, path);

  EXPECT_TRUE(found);
  ASSERT_FALSE(path.empty());
  EXPECT_EQ(path.front(), start);
  EXPECT_EQ(path.back(), goal);

  // 路径长度应该比直线长（需要绕行）
  int manhattan = std::abs(goal.x - start.x) + std::abs(goal.y - start.y);
  EXPECT_GT(static_cast<int>(path.size()), manhattan);
}

// 测试3：无解情况
TEST_F(AstarSearchTest, NoPathFound)
{
  auto map = createEmptyMap(10, 10);

  // 完全封锁目标区域
  for (int x = 6; x < 10; ++x) {
    setObstacle(map, 10, x, 5);
    setObstacle(map, 10, x, 7);
  }
  for (int y = 5; y < 8; ++y) {
    setObstacle(map, 10, 6, y);
    setObstacle(map, 10, 9, y);
  }

  GridCell start = {1, 1};
  GridCell goal = {8, 6};
  std::vector<GridCell> path;

  bool found = astar_->search(map, 10, 10, start, goal, path);

  EXPECT_FALSE(found);
  EXPECT_TRUE(path.empty());
}

// 测试4：起点等于终点
TEST_F(AstarSearchTest, StartEqualsGoal)
{
  auto map = createEmptyMap(10, 10);
  GridCell start = {5, 5};
  GridCell goal = {5, 5};
  std::vector<GridCell> path;

  bool found = astar_->search(map, 10, 10, start, goal, path);

  EXPECT_TRUE(found);
  ASSERT_EQ(path.size(), 1);
  EXPECT_EQ(path[0], start);
}

// 测试5：对角线移动
TEST_F(AstarSearchTest, DiagonalMovement)
{
  auto map = createEmptyMap(10, 10);
  GridCell start = {1, 1};
  GridCell goal = {8, 8};
  std::vector<GridCell> path;

  bool found = astar_->search(map, 10, 10, start, goal, path);

  EXPECT_TRUE(found);
  ASSERT_FALSE(path.empty());

  // 对角线路径应该比曼哈顿路径短
  // 曼哈顿距离 = |8-1| + |8-1| = 14
  // 对角线路径约10步
  EXPECT_LE(path.size(), 12);
}

// 测试6：高代价区域
TEST_F(AstarSearchTest, HighCostRegion)
{
  auto map = createEmptyMap(10, 10);

  // 在直线路径上设置高代价区域（不是障碍物）
  for (int x = 3; x < 7; ++x) {
    for (int y = 0; y < 3; ++y) {
      setObstacle(map, 10, x, y, 200);
    }
  }

  GridCell start = {1, 1};
  GridCell goal = {8, 1};
  std::vector<GridCell> path;

  bool found = astar_->search(map, 10, 10, start, goal, path);

  EXPECT_TRUE(found);
  ASSERT_FALSE(path.empty());

  // 路径可能会绕过高代价区域
  // 至少应该找到路径
  EXPECT_EQ(path.front(), start);
  EXPECT_EQ(path.back(), goal);
}

// 测试7：路径连续性
TEST_F(AstarSearchTest, PathContinuity)
{
  auto map = createEmptyMap(20, 20);

  // 添加一些障碍物使路径弯曲
  for (int y = 5; y < 15; ++y) {
    setObstacle(map, 20, 10, y);
  }

  GridCell start = {1, 10};
  GridCell goal = {18, 10};
  std::vector<GridCell> path;

  bool found = astar_->search(map, 20, 20, start, goal, path);

  EXPECT_TRUE(found);
  ASSERT_GE(path.size(), 2);

  // 检查路径连续性：相邻点距离不超过sqrt(2)
  for (size_t i = 1; i < path.size(); ++i) {
    int dx = std::abs(path[i].x - path[i-1].x);
    int dy = std::abs(path[i].y - path[i-1].y);
    EXPECT_LE(dx, 1) << "Path discontinuity at step " << i;
    EXPECT_LE(dy, 1) << "Path discontinuity at step " << i;
    EXPECT_TRUE(dx + dy >= 1) << "Path has zero-length step at " << i;
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
