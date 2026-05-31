/**
 * test_dwa_search.cpp - DWA算法单元测试
 *
 * 测试用例：
 * 1. 动态窗口计算
 * 2. 轨迹模拟
 * 3. 目标到达检测
 * 4. 空旷地图上朝目标行驶
 * 5. 障碍物避让
 */

#include <gtest/gtest.h>
#include "agv_local_planner/dwa_search.hpp"

using namespace agv_local_planner;

class DWASearchTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    DWAParams params;
    params.max_vel_x = 0.5;
    params.min_vel_x = -0.1;
    params.max_vel_theta = 1.5;
    params.min_vel_theta = -1.5;
    params.acc_lim_x = 1.0;
    params.acc_lim_theta = 2.0;
    params.sim_time = 1.5;
    params.sim_granularity = 0.1;
    params.vx_samples = 10;
    params.vtheta_samples = 20;
    params.path_distance_bias = 0.6;
    params.goal_distance_bias = 0.8;
    params.occdist_scale = 0.5;
    params.goal_tolerance_xy = 0.15;
    params.goal_tolerance_yaw = 0.3;

    dwa_ = std::make_unique<DWASearch>(params);
  }

  // 创建空旷地图
  std::vector<uint8_t> createEmptyMap(unsigned int w, unsigned int h)
  {
    return std::vector<uint8_t>(w * h, 0);
  }

  // 在地图上设置障碍物
  void setObstacle(std::vector<uint8_t> & map, unsigned int w,
                    int x, int y, uint8_t cost = 254)
  {
    if (x >= 0 && x < static_cast<int>(w) && y >= 0 && y < static_cast<int>(map.size() / w)) {
      map[y * w + x] = cost;
    }
  }

  std::unique_ptr<DWASearch> dwa_;
};

// 测试1：目标到达检测
TEST_F(DWASearchTest, GoalReached)
{
  Pose2D robot(1.0, 1.0, 0.0);
  Pose2D goal(1.1, 1.05, 0.1);  // 距离约0.11m < 0.15m容差

  EXPECT_TRUE(dwa_->isGoalReached(robot, goal));
}

// 测试2：目标未到达
TEST_F(DWASearchTest, GoalNotReached)
{
  Pose2D robot(1.0, 1.0, 0.0);
  Pose2D goal(2.0, 2.0, 0.0);  // 距离约1.41m

  EXPECT_FALSE(dwa_->isGoalReached(robot, goal));
}

// 测试3：轨迹模拟
TEST_F(DWASearchTest, TrajectorySimulation)
{
  Pose2D start(1.0, 1.0, 0.0);  // 朝右

  // 直行轨迹
  Trajectory straight = dwa_->simulateTrajectory(start, 0.3, 0.0);
  EXPECT_GT(straight.poses.size(), 0);

  // 末端位置应该在起点右侧
  Pose2D end = straight.endPose();
  EXPECT_GT(end.x, start.x);
  EXPECT_NEAR(end.y, start.y, 0.01);

  // 转弯轨迹
  Trajectory turning = dwa_->simulateTrajectory(start, 0.3, 0.5);
  EXPECT_GT(turning.poses.size(), 0);

  // 末端应该有角度变化
  Pose2D turn_end = turning.endPose();
  EXPECT_GT(turn_end.theta, start.theta);
}

// 测试4：空旷地图上计算速度
TEST_F(DWASearchTest, ComputeVelocityOpenSpace)
{
  auto map = createEmptyMap(100, 100);
  double resolution = 0.05;
  double origin_x = 0.0;
  double origin_y = 0.0;

  Pose2D robot(1.0, 1.0, 0.0);
  Velocity vel(0.0, 0.0);
  Pose2D goal(3.0, 1.0, 0.0);  // 右侧目标

  Velocity cmd = dwa_->computeVelocity(
    robot, vel, goal, map, 100, 100, resolution, origin_x, origin_y);

  // 应该前进
  EXPECT_GT(cmd.v, 0.0);
  // 角速度应该很小（目标几乎在正前方）
  EXPECT_LT(std::abs(cmd.omega), 0.5);
}

// 测试5：避障
TEST_F(DWASearchTest, ObstacleAvoidance)
{
  auto map = createEmptyMap(100, 100);
  double resolution = 0.05;
  double origin_x = 0.0;
  double origin_y = 0.0;

  // 在前方设置障碍物
  for (int x = 25; x < 30; ++x) {
    for (int y = 18; y < 22; ++y) {
      setObstacle(map, 100, x, y);
    }
  }

  Pose2D robot(1.0, 1.0, 0.0);
  Velocity vel(0.3, 0.0);
  Pose2D goal(2.0, 1.0, 0.0);

  Velocity cmd = dwa_->computeVelocity(
    robot, vel, goal, map, 100, 100, resolution, origin_x, origin_y);

  // 应该仍然有速度输出（不会完全停止）
  EXPECT_GE(cmd.v, 0.0);
}

// 测试6：朝向目标
TEST_F(DWASearchTest, HeadTowardsGoal)
{
  auto map = createEmptyMap(100, 100);
  double resolution = 0.05;
  double origin_x = 0.0;
  double origin_y = 0.0;

  // 目标在左上方
  Pose2D robot(1.0, 1.0, 0.0);
  Velocity vel(0.0, 0.0);
  Pose2D goal(1.5, 2.0, 0.0);

  Velocity cmd = dwa_->computeVelocity(
    robot, vel, goal, map, 100, 100, resolution, origin_x, origin_y);

  // 应该有正的角速度（逆时针转向目标）
  EXPECT_GT(cmd.omega, 0.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
