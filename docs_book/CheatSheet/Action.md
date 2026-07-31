# 速查卡：Action 与通信

> 口径：Nav2 1.5.0（main 分支）。详解见 [第 19 章](../19_Action与通信机制.md)。

## 什么时候打开本速查

- 写 Action 客户端时想不起某个 Action 的名称或消息类型全名；
- 排查"目标发出去没人接"，需要确认这个 Action 到底由哪个服务器托管；
- 自己写服务器要用 `SimpleActionServer`，想不起构造参数与抢占/取消 API；
- 行为树节点里 `server_name` 填什么拿不准；
- 想核对 cmd_vel 链路上各环节的话题名。

## 1. Nav2 常用 Action 清单

Action 定义在 `nav2_msgs/action/`（DockRobot 等 4 个在 `nav2_msgs` 中定义、由 nav2_docking/nav2_following 托管）。

| Action 名称 | 类型（nav2_msgs/action/） | 托管服务器 | 用途 |
|---|---|---|---|
| `navigate_to_pose` | NavigateToPose | bt_navigator | 单点导航总入口（跑行为树） |
| `navigate_through_poses` | NavigateThroughPoses | bt_navigator | 多途经点导航总入口 |
| `compute_path_to_pose` | ComputePathToPose | planner_server | 计算到单目标的全局路径 |
| `compute_path_through_poses` | ComputePathThroughPoses | planner_server | 计算途经多点的全局路径 |
| `follow_path` | FollowPath | controller_server | 跟踪路径输出速度指令 |
| `smooth_path` | SmoothPath | smoother_server | 平滑一条路径 |
| `spin` | Spin | behavior_server | 原地旋转恢复行为 |
| `backup` | BackUp | behavior_server | 倒退恢复行为 |
| `drive_on_heading` | DriveOnHeading | behavior_server | 沿指定朝向直行 |
| `wait` | Wait | behavior_server | 原地等待 |
| `assisted_teleop` | AssistedTeleop | behavior_server | 辅助遥操作（碰撞过滤） |
| `follow_waypoints` | FollowWaypoints | waypoint_follower | 顺序巡检路点 |
| `follow_gps_waypoints` | FollowGPSWaypoints | waypoint_follower | GPS 路点巡检 |
| `compute_route` | ComputeRoute | route_server | 路线图上算路线 |
| `compute_and_track_route` | ComputeAndTrackRoute | route_server | 算路线并跟踪进度 |
| `dock_robot` | DockRobot | docking_server | 自动靠泊 |
| `undock_robot` | UndockRobot | docking_server | 脱离泊位 |
| `follow_object` | FollowObject | following_server | 跟随动态目标 |

注：behavior_server 托管的 Action 名即 YAML 中 `behavior_plugins` 里的插件别名（默认 `spin`/`backup`/`drive_on_heading`/`wait`）。各 Action 的 result 携带 `error_code`（规划器 4xx、控制器 5xx，对照表见 [第 04 章](../04_nav2_core插件接口.md)）。行为树如何消费这些 Action 见 [第 11 章](../11_行为树与BTNavigator.md)。

## 2. SimpleActionServer 用法要点

头文件：`nav2_ros_common/include/nav2_ros_common/simple_action_server.hpp`（全文件约 680 行）。核心模型：**单目标执行 + 新目标抢占**，`work()` 循环驱动用户回调。

构造（通常经 `nav2::LifecycleNode::create_action_server<ActionT>()` 创建）：

```cpp
action_server_ = create_action_server<ActionT>(
  "follow_path",                       // action 名
  std::bind(&Self::computeControl, this),  // ExecuteCallback（工作线程内执行）
  nullptr,                             // CompletionCallback，可选
  std::chrono::milliseconds(500),      // server_timeout
  false,                               // spin_thread
  false);                              // realtime（软实时优先级）
```

执行回调内的惯用 API（均已在源码核实）：

| 方法 | 用途 |
|---|---|
| `activate()` / `deactivate()` | 随宿主 on_activate/on_deactivate 开关接单 |
| `is_server_active()` | 服务器是否激活 |
| `is_running()` | 是否有目标正在执行 |
| `get_current_goal()` / `get_pending_goal()` | 取当前/待决目标 |
| `is_preempt_requested()` | 是否有新目标请求抢占 |
| `accept_pending_goal()` | 接受抢占目标（在执行循环内切换） |
| `is_cancel_requested()` | 当前目标是否被取消 |
| `publish_feedback(feedback)` | 发布反馈 |
| `succeeded_current(result)` | 以成功结束当前目标 |
| `terminate_current(result)` | 以中止结束当前目标 |
| `terminate_all(result)` | 结束当前与待决的全部目标 |

典型执行循环骨架：

```cpp
while (rclcpp::ok()) {
  if (action_server_->is_cancel_requested()) {
    action_server_->terminate_all();
    return;
  }
  if (action_server_->is_preempt_requested()) {
    goal = action_server_->accept_pending_goal();  // 无缝切换新目标
  }
  // ... 干活、publish_feedback() ...
}
action_server_->succeeded_current(result);
```

线程模型（执行回调跑在独立工作线程、软实时选项）详见 [第 20 章](../20_线程模型.md)。

## 3. cmd_vel 链路速记

```
controller_server / behavior_server ──(并联)──> cmd_vel_nav
  └─> velocity_smoother ──> cmd_vel_smoothed ──> collision_monitor ──> cmd_vel
```

behavior_server 与控制器**并联**输出 `cmd_vel_nav`；velocity_smoother 与 collision_monitor 串在下游。话题名可在 `nav2_bringup/params/nav2_params.yaml` 的 `collision_monitor.cmd_vel_in_topic/cmd_vel_out_topic` 核对。详见 [第 16 章](../16_CollisionMonitor与VelocitySmoother.md)。
