# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2 Humble warehouse AGV autonomous navigation system in C++. Supports multi-vehicle scheduling, A* global planning, DWA local obstacle avoidance, path replanning, deadlock detection, cooperative transport, and battery management.

## Build & Test Commands

```bash
# Build all packages
cd ~/AGV && colcon build

# Build single package
colcon build --packages-select agv_global_planner

# Run tests (A* and DWA have unit tests)
colcon test --packages-select agv_global_planner agv_local_planner

# Run single test binary directly
./build/agv_global_planner/test_astar_search
./build/agv_local_planner/test_dwa_search

# View test results
colcon test-result --verbose

# Source workspace before running
source install/setup.bash
```

## Launch System

```bash
# Full multi-AGV 2D simulation (default)
ros2 launch agv_navigation multi_agv.launch.py

# Gazebo 3D simulation
ros2 launch agv_gazebo gazebo_simulation.launch.py

# Web dashboard (separate terminal)
ros2 launch agv_web_monitor web_monitor.launch.py
# Then open http://localhost:8080
```

## Architecture

```
Global Nodes (shared):
  agv_map/map_server_node          → /map (OccupancyGrid)
  agv_global_planner/global_planner_node → /plan_path (PathPlan srv)
  agv_scheduler/traffic_manager_node → /reserve_path, /release_path
  agv_scheduler/task_scheduler_node → /assign_task, /fleet_status

Per AGV (namespaced under /agv_001, /agv_002, ...):
  agv_simulator/agv_sim_node       → TF (map→base_link), /status, /cmd_vel subscription
  agv_local_planner/dwa_planner_node → /cmd_vel (Twist), subscribes /map + /planned_path
  agv_navigation/navigation_coordinator_node → orchestrates plan→reserve→follow flow
```

**Data flow**: Task Scheduler → assigns task to AGV → Navigation Coordinator calls `/plan_path` → publishes path to `/planned_path` → DWA subscribes path + costmap → publishes `/cmd_vel` → Simulator moves robot + publishes TF.

**Multi-AGV isolation**: Each AGV runs in its own ROS2 namespace. Global services (`/plan_path`, `/reserve_path`, `/assign_task`) are absolute names, not namespace-relative.

## Key Packages

- **agv_interfaces**: Custom msg/srv/action (AGVStatus, TaskRequest, FleetStatus, PathPlan.srv, ReservePath.srv, AssignTask.srv, Navigate.action)
- **agv_global_planner**: Pure algorithm class `AstarSearch` (no ROS deps) + ROS2 wrapper `AstarPlanner`. 8-direction Octile heuristic. Chaikin path smoothing.
- **agv_local_planner**: Pure algorithm class `DWASearch` + ROS2 wrapper `DWAPlanner`. Dynamic Window Approach with heading/clearance/velocity scoring.
- **agv_simulator**: 2D diff-drive simulator. Publishes TF, AGVStatus, battery markers. Also has `ObstacleManager` for dynamic obstacles (pedestrians, forklifts, carts).
- **agv_navigation**: Navigation coordinator (Action server), cooperative transport (leader-follower), data recorder/replayer, floor manager.
- **agv_scheduler**: Task scheduler with priority queue + aging + preemption. Traffic manager with time-space reservation table.
- **agv_web_monitor**: Python HTTP server + HTML/JS dashboard. Polls `/api/data` for fleet status.

## Code Patterns

- **Pure algorithm + ROS2 wrapper**: Core algorithms (`AstarSearch`, `DWASearch`) are standalone C++ classes with no ROS dependencies. Testable independently. ROS2 nodes wrap them.
- **Header location**: Headers are in `include/` (flat, not `include/<pkg_name>/`). Include as `#include "astar_search.hpp"`.
- **Chinese comments**: All source files have detailed Chinese comments explaining algorithms and design decisions.
- **Parameter config**: YAML files in `config/` directories. Launch files pass params to nodes.

## Map Coordinate System

- Map origin at (0,0), resolution 0.05m/cell, size ~400x400 cells (4x3.5m warehouse)
- OccupancyGrid: 0=free, 100=obstacle, -1=unknown
- Costmap2D: 0=free, 1-252=inflated, 253=inscribed, 254=lethal, 255=unknown

## Adding a New AGV

Edit `multi_agv.launch.py`: add `create_agv_group('agv_003', initial_x, initial_y)` and update `agv_ids` parameter in scheduler config.

## Known Issues

- DWA goal tolerance: the `goal_tolerance_xy` parameter controls when AGV stops. Too small causes oscillation at goal.
- ROS process cleanup: kill stale ROS processes with `pkill -f ros2` or `kill -9 <pid>` before relaunching.
