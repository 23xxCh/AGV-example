# AGV路径规划系统 / AGV Path Planning System

基于ROS2 Humble的仓储AGV自主导航系统，支持多车调度、路径规划、局部避障和死锁检测。

A warehouse AGV autonomous navigation system based on ROS2 Humble, supporting multi-vehicle scheduling, path planning, local obstacle avoidance, and deadlock detection.

## 系统架构 / System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      全局节点 / Global Nodes                      │
│  地图服务器    │  A*规划器    │  交通管理器    │  任务调度器        │
│  Map Server   │  A* Planner  │  Traffic Mgr   │  Task Scheduler   │
└─────────────────────────────────────────────────────────────────┘
        │              │              │                │
   /map │    /plan_path│   /reserve   │   /assign_task │
        ▼              ▼              ▼                ▼
┌─────────────────────────────────────────────────────────────────┐
│                每台AGV（独立命名空间） / Per AGV (Namespace)       │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐             │
│  │  仿真器      │  │  DWA避障器  │  │  导航协调器  │             │
│  │  Simulator   │  │  DWA Planner│  │  Coordinator│             │
│  └─────────────┘  └─────────────┘  └─────────────┘             │
└─────────────────────────────────────────────────────────────────┘
```

## 功能特性 / Features

| 功能 / Feature | 说明 / Description | 状态 / Status |
|----------------|-------------------|---------------|
| 地图服务器 / Map Server | 加载PGM/YAML地图，发布OccupancyGrid | ✅ 完成 |
| A*全局规划 / A* Global Planner | 8方向Octile启发式搜索 | ✅ 完成 |
| DWA局部避障 / DWA Local Planner | 动态窗口法实时避障 | ✅ 完成 |
| 2D仿真环境 / 2D Simulation | 差分运动学模型+TF发布 | ✅ 完成 |
| 多AGV调度 / Multi-AGV Scheduling | 最近空闲优先分配算法 | ✅ 完成 |
| 交通管理器 / Traffic Manager | 时间-空间预约表，路径冲突检测 | ✅ 完成 |
| 路径重规划 / Path Replanning | 卡住检测，自动重新规划路径 | ✅ 完成 |
| 死锁检测 / Deadlock Detection | 等待图循环检测，强制解除死锁 | ✅ 完成 |

## 包结构 / Package Structure

| 包名 / Package | 功能 / Purpose |
|----------------|---------------|
| `agv_interfaces` | 自定义消息/服务/动作接口 / Custom msg/srv/action interfaces |
| `agv_map` | 地图服务器 / Map server |
| `agv_global_planner` | A*全局路径规划器 / A* global path planner |
| `agv_local_planner` | DWA局部避障器 / DWA local obstacle avoidance |
| `agv_simulator` | 2D AGV仿真器 / 2D AGV simulator |
| `agv_navigation` | 导航协调器 / Navigation coordinator |
| `agv_scheduler` | 多车调度（任务调度器+交通管理器）/ Multi-AGV scheduling |

## 快速开始 / Quick Start

### 编译 / Build

```bash
cd ~/AGV
colcon build
source install/setup.bash
```

### 启动多AGV系统 / Launch Multi-AGV System

```bash
ros2 launch agv_navigation multi_agv.launch.py
```

### 分配任务 / Assign Task

```bash
# 自动分配给最近的空闲AGV
# Automatically assign to the nearest idle AGV
ros2 service call /assign_task agv_interfaces/srv/AssignTask \
  '{task: {goal_x: 2.0, goal_y: 1.5, goal_theta: 0.0, priority: 1}}'
```

### 直接导航（单AGV）/ Direct Navigation (Single AGV)

```bash
ros2 action send_goal /agv_001/navigate agv_interfaces/action/Navigate \
  '{goal_position: {x: 2.0, y: 1.5, z: 0.0}, goal_theta: 0.0, \
    use_current_pose: true, max_planning_time: 10.0}'
```

## 参数配置 / Configuration

### 任务调度器参数 / Task Scheduler Parameters

| 参数 / Parameter | 默认值 / Default | 说明 / Description |
|-----------------|-----------------|-------------------|
| `agv_ids` | [agv_001, agv_002] | AGV编号列表 / AGV ID list |
| `assignment_interval` | 1.0 | 任务分配间隔(秒) / Assignment interval(s) |
| `task_timeout` | 300.0 | 任务超时(秒) / Task timeout(s) |

### 交通管理器参数 / Traffic Manager Parameters

| 参数 / Parameter | 默认值 / Default | 说明 / Description |
|-----------------|-----------------|-------------------|
| `resolution` | 0.05 | 地图分辨率(米/格) / Map resolution(m/cell) |
| `safety_distance` | 2 | 安全距离(格) / Safety distance(cells) |
| `reservation_timeout` | 60.0 | 预约超时(秒) / Reservation timeout(s) |
| `deadlock_timeout` | 30.0 | 死锁超时(秒) / Deadlock timeout(s) |

### 导航协调器参数 / Navigation Coordinator Parameters

| 参数 / Parameter | 默认值 / Default | 说明 / Description |
|-----------------|-----------------|-------------------|
| `goal_tolerance_xy` | 0.3 | 到达容差(米) / Goal tolerance(m) |
| `stuck_timeout` | 10.0 | 卡住超时(秒) / Stuck timeout(s) |
| `max_replans` | 3 | 最大重规划次数 / Max replan count |
| `avg_speed` | 0.3 | 平均速度(米/秒) / Average speed(m/s) |

## 导航流程 / Navigation Flow

```
收到任务 → A*规划 → 预约路径 → DWA避障行驶 → 到达目标
Receive    A* Plan   Reserve    DWA Navigate   Arrive
Task       Path      Path       & Avoid        at Goal
                    ↑              ↓
               重预约新路径    检测卡住(10秒)
               Re-reserve     Detect Stuck
                    ↑              ↓
                    └── 释放旧预约 ←┘
                        Release & Replan

                    ↑              ↓
                 检测死锁      强制释放
                 Detect        Force
                 Deadlock      Release
```

## 接口定义 / Interface Definitions

### 消息 / Messages

- `TaskRequest.msg` - 任务请求 / Task request
- `AGVStatus.msg` - AGV状态 / AGV status
- `FleetStatus.msg` - 车队状态 / Fleet status

### 服务 / Services

- `AssignTask.srv` - 任务分配 / Task assignment
- `PathPlan.srv` - 路径规划 / Path planning
- `ReservePath.srv` - 路径预约/释放 / Path reservation/release

### 动作 / Actions

- `Navigate.action` - 导航动作 / Navigation action

## TF坐标系 / TF Frames

```
map
 └── agv_001_base_link
 └── agv_002_base_link
```

## 话题列表 / Topics

| 话题 / Topic | 类型 / Type | 说明 / Description |
|-------------|------------|-------------------|
| `/map` | OccupancyGrid | 栅格地图 / Occupancy grid map |
| `/agv_XXX/planned_path` | Path | 规划路径 / Planned path |
| `/agv_XXX/cmd_vel` | Twist | 速度指令 / Velocity command |
| `/agv_XXX/robot_marker` | Marker | 机器人可视化 / Robot visualization |
| `/agv_XXX/dwa_markers` | MarkerArray | DWA轨迹可视化 / DWA trajectory visualization |

## 开发环境 / Development Environment

- **OS**: Ubuntu 22.04
- **ROS2**: Humble Hawksbill
- **Language**: C++17
- **Build**: CMake + colcon

## 许可证 / License

MIT License
