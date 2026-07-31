# 速查卡：nav2_core 插件接口

> 口径：Nav2 1.5.0（main 分支）。全部签名核实自 `nav2_core/include/nav2_core/` 头文件。详解见 [第 04 章](../04_nav2_core插件接口.md)。

## 什么时候打开本速查

- 动手写自定义插件，想不起要实现哪些纯虚函数、签名长什么样；
- 改 YAML 时想不起某类插件挂在哪个键下（`planner_plugins` 还是 `plugins`？）；
- 拿不准某接口有没有默认实现、最小实现要写几个方法；
- 忘了 `PLUGINLIB_EXPORT_CLASS` 宏和 plugins.xml 的写法；
- 想确认某接口的 configure/initialize 注入了哪些依赖。

## 1. 九大接口总表

| 接口 | 头文件（nav2_core/include/nav2_core/） | 纯虚函数 | 默认实现 | YAML 挂载键（所在服务器） |
|---|---|---|---|---|
| GlobalPlanner | global_planner.hpp | `configure` `cleanup` `activate` `deactivate` `createPlan` | 无 | `planner_plugins`（planner_server） |
| Controller | controller.hpp | `configure` `cleanup` `activate` `deactivate` `newPathReceived` `computeVelocityCommands` `setSpeedLimit` | `cancel()` 返 true；`reset()` 空 | `controller_plugins`（controller_server） |
| Smoother | smoother.hpp | `configure` `cleanup` `activate` `deactivate` `smooth` | 无 | `smoother_plugins`（smoother_server） |
| GoalChecker | goal_checker.hpp | `initialize` `reset` `isGoalReached` `getTolerances` `isGoalXYReached` | 无 | `goal_checker_plugins`（controller_server） |
| ProgressChecker | progress_checker.hpp | `initialize` `check` `reset` | 无 | `progress_checker_plugins`（controller_server） |
| PathHandler | path_handler.hpp | `initialize` `setPlan` `findPlanSegment` `transformLocalPlan` `getTransformedGoal` | 无 | `path_handler_plugins`（controller_server） |
| Behavior | behavior.hpp | `configure` `cleanup` `activate` `deactivate` `getResourceInfo` | 无 | `behavior_plugins`（behavior_server） |
| NavigatorBase / BehaviorTreeNavigator | behavior_tree_navigator.hpp | 基类：`on_configure` `on_activate` `on_deactivate` `on_cleanup` `preempt`；模板层留给子类：`getName` `getDefaultBTFilepath` `goalReceived` `onLoop` `onPreempt` `goalCompleted` | BehaviorTreeNavigator 将基类五方法实现为 `final`；`configure()` 有空默认 | `navigators`（bt_navigator） |
| WaypointTaskExecutor | waypoint_task_executor.hpp | `initialize` `processAtWaypoint` | 无 | `waypoint_task_executor_plugin`（waypoint_follower） |

分野速记：**重资产接口**（GlobalPlanner/Controller/Smoother/Behavior）走四阶段生命周期；**轻量判据接口**（GoalChecker/ProgressChecker/PathHandler/WaypointTaskExecutor）只有 `initialize`（部分带 `reset`）。逐接口精讲见 [第 04 章](../04_nav2_core插件接口.md)。

## 2. 核心方法签名（现场核实）

```cpp
// GlobalPlanner（global_planner.hpp，createPlan 约 78 行处）
virtual nav_msgs::msg::Path createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::vector<geometry_msgs::msg::PoseStamped> & viapoints,
  std::function<bool()> cancel_checker) = 0;

// Controller（controller.hpp，computeVelocityCommands 约 118 行处）
virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker,
  const nav_msgs::msg::Path & transformed_global_plan,
  const geometry_msgs::msg::PoseStamped & global_goal) = 0;

// Smoother（smoother.hpp，smooth 约 74 行处）：原地修改，超时返 false
virtual bool smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & max_time) = 0;

// Behavior（behavior.hpp）：声明所需代价地图，NONE/LOCAL/GLOBAL/BOTH
virtual CostmapInfoType getResourceInfo() = 0;

// WaypointTaskExecutor（waypoint_task_executor.hpp）
virtual bool processAtWaypoint(
  const geometry_msgs::msg::PoseStamped & curr_pose, const int & curr_waypoint_index) = 0;
```

configure/initialize 首参一律是 `nav2::LifecycleNode::WeakPtr`（防循环引用，用前须 `lock()`）；注入差异：GlobalPlanner/Controller 拿 `Costmap2DROS`，Smoother 拿 `CostmapSubscriber`+`FootprintSubscriber`，Behavior 拿局部/全局两个 `CostmapTopicCollisionChecker`。设计动机见 [第 04 章](../04_nav2_core插件接口.md)。

## 3. pluginlib 导出四件套

以 NavFn 为例（`nav2_navfn_planner/src/navfn_planner.cpp` 末尾约 530 行处）：

```cpp
// ① 源文件末尾注册宏
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_navfn_planner::NavfnPlanner, nav2_core::GlobalPlanner)
```

```xml
<!-- ② plugins.xml：声明库与基类 -->
<class_libraries>
  <library path="my_planner_lib">
    <class type="my_pkg::MyPlanner" base_class_type="nav2_core::GlobalPlanner">
      <description>My planner</description>
    </class>
  </library>
</class_libraries>

<!-- ③ package.xml：导出声明 -->
<export>
  <nav2_core plugin="${prefix}/plugins.xml"/>
</export>
```

```yaml
# ④ YAML：别名 → 实现类型字符串
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "my_pkg::MyPlanner"
```

CMake 侧还需 `pluginlib_export_plugin_description_file(nav2_core plugins.xml)`。完整实战（含调试）见 [第 24 章](../24_扩展开发指南.md)。

## 4. 高频踩坑清单

- 缓存 `parent.lock()` 结果为成员 shared_ptr —— 重新制造引用环；
- deactivate 忘停线程 —— 节点 INACTIVE 仍在发消息；
- plugins.xml 的 `base_class_type` 拼错 / package.xml 漏 export —— 运行时 "class not found"，编译期无提示；
- 长耗时规划不轮询 `cancel_checker` —— 取消请求被拖到整次搜索结束。

详见 [第 04 章](../04_nav2_core插件接口.md) "最佳实践与容易踩坑"。
