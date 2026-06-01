#!/usr/bin/env python3
"""
工厂场景10轮测试脚本
=====================
测试AGV在实际工厂环境中的导航能力。

用法:
  1. 启动仿真: ros2 launch agv_navigation multi_agv.launch.py
  2. 运行测试: python3 ~/AGV/test/factory_test.py

测试用例设计原则:
  - 从简单到复杂
  - 覆盖直线、转弯、窄通道、绕障碍、多车协同
  - 模拟真实工厂任务：取货、送货、充电、避让
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from agv_interfaces.srv import AssignTask
from agv_interfaces.msg import AGVStatus
from agv_interfaces.action import Navigate
import time
import sys
import json
import math
from dataclasses import dataclass, asdict
from typing import Optional


@dataclass
class TestCase:
    name: str
    description: str
    goal_x: float
    goal_y: float
    goal_theta: float
    priority: int
    target_agv: Optional[str] = None  # None=auto assign
    timeout: float = 60.0
    expected_success: bool = True


# ============================================================
# 10轮测试用例定义
# ============================================================
TEST_CASES = [
    # ---- 基础测试 (1-3) ----
    TestCase(
        name="T01_直线导航",
        description="从起点直线前进到货架A1前方，验证基本导航能力",
        goal_x=-1.5, goal_y=1.5, goal_theta=0.0,
        priority=1, timeout=60.0
    ),
    TestCase(
        name="T02_转弯导航",
        description="从起点导航到右侧货架B3，需要转弯和横向移动",
        goal_x=1.5, goal_y=-2.5, goal_theta=0.0,
        priority=1, timeout=60.0
    ),
    TestCase(
        name="T03_对角穿越",
        description="从左下到右上对角穿越仓库，经过中央堆垛区",
        goal_x=3.0, goal_y=2.0, goal_theta=0.0,
        priority=1, timeout=80.0
    ),

    # ---- 窄通道测试 (4-5) ----
    TestCase(
        name="T04_货架间穿行",
        description="在两列货架之间横向穿行，通道宽约1.2m",
        goal_x=-2.5, goal_y=-0.5, goal_theta=1.57,
        priority=2, timeout=60.0
    ),
    TestCase(
        name="T05_绕行堆垛",
        description="绕过中央堆垛区到达对面，验证绕行能力",
        goal_x=-0.5, goal_y=-2.0, goal_theta=0.0,
        priority=2, timeout=80.0
    ),

    # ---- 真实工厂任务 (6-8) ----
    TestCase(
        name="T06_取货任务",
        description="从起点导航到货架A2取货点（模拟取货流程）",
        goal_x=-1.5, goal_y=-0.5, goal_theta=0.0,
        priority=3, timeout=60.0
    ),
    TestCase(
        name="T07_送货到工作站",
        description="从货架区域送货到工作站1（模拟送货流程）",
        goal_x=-3.0, goal_y=2.0, goal_theta=0.0,
        priority=3, timeout=80.0
    ),
    TestCase(
        name="T08_充电导航",
        description="导航到充电桩位置（模拟低电量回充）",
        goal_x=3.8, goal_y=-2.0, goal_theta=0.0,
        priority=1, timeout=80.0
    ),

    # ---- 多车协同测试 (9-10) ----
    TestCase(
        name="T09_多车同向",
        description="两台AGV同时导航到不同货架，路径部分重叠",
        goal_x=-1.5, goal_y=1.5, goal_theta=0.0,
        priority=2, timeout=150.0,
        target_agv="agv_001"
    ),
    TestCase(
        name="T10_多车对向",
        description="两台AGV相向而行，验证避让和交通管理",
        goal_x=1.5, goal_y=-2.5, goal_theta=0.0,
        priority=3, timeout=150.0,
        target_agv="agv_002"
    ),
]


class FactoryTestRunner(Node):
    """工厂测试运行器"""

    def __init__(self):
        super().__init__('factory_test_runner')

        # 服务客户端
        self.assign_client = self.create_client(AssignTask, '/assign_task')

        # Action客户端（用于直接导航测试）
        self.nav_clients = {
            'agv_001': ActionClient(self, Navigate, '/agv_001/navigate'),
            'agv_002': ActionClient(self, Navigate, '/agv_002/navigate'),
        }

        # AGV状态订阅（用于检测到达）
        self.agv_positions = {}
        self.agv_statuses = {}
        for agv_id in ['agv_001', 'agv_002']:
            self.create_subscription(
                AGVStatus, f'/{agv_id}/status',
                lambda msg, aid=agv_id: self._status_callback(msg, aid),
                10
            )

        # 测试结果
        self.results = []

        self.get_logger().info("工厂测试运行器已启动")

    def _status_callback(self, msg: AGVStatus, agv_id: str):
        """记录AGV位置和状态"""
        self.agv_positions[agv_id] = (
            msg.pose.pose.pose.position.x,
            msg.pose.pose.pose.position.y
        )
        self.agv_statuses[agv_id] = msg.status

    def wait_for_services(self, timeout=10.0):
        """等待服务可用"""
        if not self.assign_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("assign_task 服务不可用")
            return False
        self.get_logger().info("所有服务已就绪")
        return True

    def run_single_test(self, test: TestCase, test_index: int) -> dict:
        """运行单个测试用例"""
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"测试 {test_index+1}/10: {test.name}")
        self.get_logger().info(f"描述: {test.description}")
        self.get_logger().info(f"目标: ({test.goal_x}, {test.goal_y}, {test.goal_theta})")
        self.get_logger().info(f"{'='*60}")

        start_time = time.time()
        result = {
            'name': test.name,
            'description': test.description,
            'passed': False,
            'duration': 0.0,
            'error': ''
        }

        try:
            if test.target_agv:
                # 直接导航到指定AGV
                success = self.send_direct_navigation(test)
            else:
                # 通过调度器分配
                success = self.send_assign_task(test)

            if not success:
                result['error'] = '任务发送失败'
                return result

            # 等待任务完成（轮询检查）
            completed = self.wait_for_completion(test)
            elapsed = time.time() - start_time
            result['duration'] = round(elapsed, 1)

            if completed:
                result['passed'] = True
                self.get_logger().info(f"✓ 测试通过 ({elapsed:.1f}s)")
            else:
                result['error'] = f'超时 ({test.timeout}s)'
                self.get_logger().warn(f"✗ 测试失败: 超时")

        except Exception as e:
            result['error'] = str(e)
            self.get_logger().error(f"✗ 测试异常: {e}")

        return result

    def send_assign_task(self, test: TestCase) -> bool:
        """通过调度器分配任务"""
        req = AssignTask.Request()
        req.task.task_id = test.name
        req.task.goal_x = test.goal_x
        req.task.goal_y = test.goal_y
        req.task.goal_theta = test.goal_theta
        req.task.priority = test.priority

        future = self.assign_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            resp = future.result()
            if resp.success:
                agv = resp.assigned_agv_id or "(队列中)"
                self.get_logger().info(f"任务已分配: AGV={agv}")
                return True
            else:
                self.get_logger().error(f"分配失败: {resp.error_msg}")
                return False
        return False

    def send_direct_navigation(self, test: TestCase) -> bool:
        """直接发送导航目标到指定AGV"""
        client = self.nav_clients.get(test.target_agv)
        if not client:
            return False

        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"{test.target_agv} Action服务器不可用")
            return False

        goal = Navigate.Goal()
        goal.goal_position.x = test.goal_x
        goal.goal_position.y = test.goal_y
        goal.goal_position.z = 0.0
        goal.goal_theta = test.goal_theta
        goal.use_current_pose = True
        goal.max_planning_time = 10.0

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)

        if send_future.result() is not None:
            goal_handle = send_future.result()
            if goal_handle.accepted:
                self.get_logger().info(f"目标已被 {test.target_agv} 接受")
                return True
            else:
                self.get_logger().error(f"{test.target_agv} 拒绝了目标")
        return False

    def wait_for_completion(self, test: TestCase) -> bool:
        """等待任务完成，通过检测AGV位置判断是否到达目标"""
        start = time.time()
        check_interval = 1.0
        goal_tolerance = 0.35  # 到达容差（米），略大于导航协调器的0.3m
        stable_count = 0  # 连续到达计数
        stable_required = 2  # 连续2次检测到达才确认

        # 确定要检查的AGV
        check_agv = test.target_agv
        if not check_agv:
            # 自动分配模式，检查所有AGV
            check_agv = None

        while time.time() - start < test.timeout:
            # 使用spin_once代替time.sleep，允许回调处理状态消息
            wait_end = time.time() + check_interval
            while time.time() < wait_end:
                rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = time.time() - start
            self.get_logger().info(f"  等待中... ({elapsed:.0f}s/{test.timeout:.0f}s)")

            # 检查是否有AGV到达目标附近
            agvs_to_check = [check_agv] if check_agv else ['agv_001', 'agv_002']
            reached = False

            for agv_id in agvs_to_check:
                pos = self.agv_positions.get(agv_id)
                if pos is None:
                    continue
                dx = pos[0] - test.goal_x
                dy = pos[1] - test.goal_y
                dist = math.sqrt(dx * dx + dy * dy)

                if dist < goal_tolerance:
                    stable_count += 1
                    self.get_logger().info(f"  {agv_id} 接近目标 (距离={dist:.2f}m, 稳定={stable_count}/{stable_required})")
                    if stable_count >= stable_required:
                        self.get_logger().info(f"  {agv_id} 已到达目标！")
                        return True
                    reached = True
                    break

            if not reached:
                stable_count = 0  # 重置稳定计数

        return False

    def run_all_tests(self):
        """运行全部10轮测试"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("  工厂场景10轮测试开始")
        self.get_logger().info("="*60)

        i = 0
        while i < len(TEST_CASES):
            test = TEST_CASES[i]

            # T09和T10是多车测试，需要特殊处理
            if test.name in ["T09_多车同向", "T10_多车对向"]:
                self.get_logger().info(f"\n[多车测试] 同时发送两个任务...")
                test1 = TEST_CASES[8]  # T09
                test2 = TEST_CASES[9]  # T10

                start_time = time.time()
                self.send_direct_navigation(test1)
                rclpy.spin_once(self, timeout_sec=0.5)
                self.send_direct_navigation(test2)

                # 等待两个任务都完成（使用spin_once允许回调）
                wait_end = time.time() + 5
                while time.time() < wait_end:
                    rclpy.spin_once(self, timeout_sec=0.1)
                elapsed = time.time() - start_time

                # 检查两个AGV是否到达
                pos1 = self.agv_positions.get('agv_001')
                pos2 = self.agv_positions.get('agv_002')
                tol = 0.5

                r1_ok = pos1 and math.sqrt((pos1[0]-test1.goal_x)**2 + (pos1[1]-test1.goal_y)**2) < tol
                r2_ok = pos2 and math.sqrt((pos2[0]-test2.goal_x)**2 + (pos2[1]-test2.goal_y)**2) < tol

                # 如果还没到，继续等待
                timeout = max(test1.timeout, test2.timeout)
                while elapsed < timeout and not (r1_ok and r2_ok):
                    wait_end2 = time.time() + 2
                    while time.time() < wait_end2:
                        rclpy.spin_once(self, timeout_sec=0.1)
                    elapsed = time.time() - start_time
                    pos1 = self.agv_positions.get('agv_001')
                    pos2 = self.agv_positions.get('agv_002')
                    r1_ok = pos1 and math.sqrt((pos1[0]-test1.goal_x)**2 + (pos1[1]-test1.goal_y)**2) < tol
                    r2_ok = pos2 and math.sqrt((pos2[0]-test2.goal_x)**2 + (pos2[1]-test2.goal_y)**2) < tol
                    self.get_logger().info(f"  等待中... ({elapsed:.0f}s) agv_001={'OK' if r1_ok else '...'}, agv_002={'OK' if r2_ok else '...'}")

                self.results.append({
                    'name': test1.name, 'passed': r1_ok, 'duration': round(elapsed, 1),
                    'error': '' if r1_ok else '未到达目标'
                })
                self.results.append({
                    'name': test2.name, 'passed': r2_ok, 'duration': round(elapsed, 1),
                    'error': '' if r2_ok else '未到达目标'
                })
                i += 2
                continue

            result = self.run_single_test(test, i)
            self.results.append(result)
            i += 1

            # 测试间隔（使用spin_once允许回调处理）
            wait_end = time.time() + 3
            while time.time() < wait_end:
                rclpy.spin_once(self, timeout_sec=0.1)

        self.print_summary()

    def print_summary(self):
        """打印测试汇总"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("  测试结果汇总")
        self.get_logger().info("="*60)

        passed = sum(1 for r in self.results if r['passed'])
        total = len(self.results)

        for i, r in enumerate(self.results):
            status = "✓ PASS" if r['passed'] else "✗ FAIL"
            duration = f"{r['duration']:.1f}s" if r['duration'] else "N/A"
            error = f" ({r['error']})" if r['error'] else ""
            self.get_logger().info(f"  {i+1:2d}. {status}  {r['name']:<20s}  {duration}{error}")

        self.get_logger().info(f"\n  通过: {passed}/{total}")
        self.get_logger().info("="*60)

        # 保存结果到JSON
        with open('/home/user/AGV/test/test_results.json', 'w') as f:
            json.dump(self.results, f, ensure_ascii=False, indent=2)
        self.get_logger().info("结果已保存到 test/test_results.json")


def main(args=None):
    rclpy.init(args=args)
    runner = FactoryTestRunner()

    if not runner.wait_for_services():
        runner.get_logger().error("服务未就绪，请先启动仿真系统")
        rclpy.shutdown()
        return

    try:
        runner.run_all_tests()
    except KeyboardInterrupt:
        runner.get_logger().info("测试被中断")
    finally:
        runner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
