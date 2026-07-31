# 实战示例：从零实现一个直线规划器插件（GlobalPlanner）

> 测试口径：Nav2 1.5.0（main 分支，`nav2_core/package.xml` 中 `<version>1.5.0</version>`），C++20。
> 相关章节：[07 PlannerServer 与全局规划](../07_PlannerServer与全局规划.md) · [04 nav2_core 插件接口](../04_nav2_core插件接口.md) · [24 扩展开发指南](../24_扩展开发指南.md)

## 1. 目标

做什么：实现一个最小可用的全局规划器插件 `my_nav2_examples::StraightLinePlanner`——从起点到终点（途经 viapoints）按固定分辨率做线性插值生成路径，不做任何避障，专注展示插件骨架。

学到什么：

- `nav2_core::GlobalPlanner` 接口的 5 个纯虚函数及其在 PlannerServer 生命周期中的调用时机；
- pluginlib 三件套：`PLUGINLIB_EXPORT_CLASS` 导出宏、plugins.xml 描述文件、`pluginlib_export_plugin_description_file` CMake 导出；
- 通过 `planner_plugins` YAML 把自定义规划器挂载进 `planner_server`。

## 2. 最小实现面：必须 override 的纯虚函数

以下签名逐字核实自 `nav2_core/include/nav2_core/global_planner.hpp`（约 88 行）：

| 纯虚函数 | 调用时机 |
|---|---|
| `virtual void configure(const nav2::LifecycleNode::WeakPtr & parent, std::string name, nav2::TransformBuffer::SharedPtr tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;` | PlannerServer `on_configure` 时，逐插件调用 |
| `virtual void cleanup() = 0;` | PlannerServer `on_cleanup` 时 |
| `virtual void activate() = 0;` | PlannerServer `on_activate` 时 |
| `virtual void deactivate() = 0;` | PlannerServer `on_deactivate` 时 |
| `virtual nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal, const std::vector<geometry_msgs::msg::PoseStamped> & viapoints, std::function<bool()> cancel_checker) = 0;` | 每次收到 ComputePathToPose/ThroughPoses 请求时 |

注意两个 1.5.0 口径的关键点（与老教程不同，勿凭记忆照抄旧签名）：

- 节点与 TF 类型是 `nav2::LifecycleNode::WeakPtr` 与 `nav2::TransformBuffer::SharedPtr`（来自 `nav2_ros_common/lifecycle_node.hpp`、`nav2_ros_common/tf2_factories.hpp`，其中 `nav2::TransformBuffer` 是 `tf2_ros::Buffer` 的别名），不再是 `rclcpp_lifecycle::LifecycleNode` / 裸 `tf2_ros::Buffer`；
- `createPlan` 带 `viapoints`（途经点）与 `cancel_checker`（取消检查）两个参数。

## 3. 完整代码

工程布局（独立 ament 包 `my_nav2_examples_planner`）：

```
my_nav2_examples_planner/
├── include/my_nav2_examples_planner/straight_line_planner.hpp
├── src/straight_line_planner.cpp
├── straight_line_planner_plugin.xml
├── CMakeLists.txt
└── package.xml
```

### 3.1 straight_line_planner.hpp

```cpp
#ifndef MY_NAV2_EXAMPLES_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
#define MY_NAV2_EXAMPLES_PLANNER__STRAIGHT_LINE_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/global_planner.hpp"          // 基类接口（纯虚）
#include "nav2_core/planner_exceptions.hpp"      // PlannerException 异常族
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"    // nav2::LifecycleNode
#include "nav2_ros_common/tf2_factories.hpp"     // nav2::TransformBuffer
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace my_nav2_examples
{

/**
 * @class StraightLinePlanner
 * @brief 忽略障碍物的直线规划器：start -> viapoints... -> goal 逐段线性插值。
 *        仅用于教学演示插件骨架，请勿在真实机器人上单独使用。
 */
class StraightLinePlanner : public nav2_core::GlobalPlanner
{
public:
  StraightLinePlanner() = default;
  ~StraightLinePlanner() override = default;

  // ---- 以下 5 个 override 与 global_planner.hpp 中的纯虚函数一一对应 ----
  void configure(
    const nav2::LifecycleNode::WeakPtr & parent,
    std::string name, nav2::TransformBuffer::SharedPtr tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::vector<geometry_msgs::msg::PoseStamped> & viapoints,
    std::function<bool()> cancel_checker) override;

private:
  /// 向 path 追加 from->to 之间的插值点（不含 to 本身）
  void appendSegment(
    const geometry_msgs::msg::PoseStamped & from,
    const geometry_msgs::msg::PoseStamped & to,
    nav_msgs::msg::Path & path,
    const std::function<bool()> & cancel_checker) const;

  nav2::LifecycleNode::WeakPtr node_;                       // 宿主节点（弱引用，防循环持有）
  nav2::TransformBuffer::SharedPtr tf_;                     // TF 缓冲（本例未使用，保留成员示范）
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("StraightLinePlanner")};
  std::string name_;           // YAML 中的插件实例名，如 "GridBased"
  std::string global_frame_;   // 全局代价地图坐标系，通常为 "map"
  double interpolation_resolution_{0.1};  // 插值步长（米）
};

}  // namespace my_nav2_examples

#endif  // MY_NAV2_EXAMPLES_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
```

### 3.2 straight_line_planner.cpp

```cpp
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "my_nav2_examples_planner/straight_line_planner.hpp"

namespace my_nav2_examples
{

void StraightLinePlanner::configure(
  const nav2::LifecycleNode::WeakPtr & parent,
  std::string name, nav2::TransformBuffer::SharedPtr tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();   // WeakPtr -> SharedPtr，configure 阶段一定成功
  if (!node) {
    throw std::runtime_error{"StraightLinePlanner: 无法锁定宿主节点"};
  }

  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  global_frame_ = costmap_ros_->getGlobalFrameID();
  logger_ = node->get_logger();

  // 1.5.0 推荐用 declare_or_get_parameter：已声明则取值，未声明则用默认值声明
  // 参数全名形如 planner_server.GridBased.interpolation_resolution
  interpolation_resolution_ = node->declare_or_get_parameter(
    name_ + ".interpolation_resolution", 0.1);

  RCLCPP_INFO(
    logger_, "已配置插件 %s（类型 StraightLinePlanner），插值分辨率 %.3f m",
    name_.c_str(), interpolation_resolution_);
}

void StraightLinePlanner::cleanup()
{
  RCLCPP_INFO(logger_, "清理插件 %s", name_.c_str());
  costmap_ros_.reset();
  tf_.reset();
}

void StraightLinePlanner::activate()
{
  RCLCPP_INFO(logger_, "激活插件 %s", name_.c_str());
}

void StraightLinePlanner::deactivate()
{
  RCLCPP_INFO(logger_, "停用插件 %s", name_.c_str());
}

nav_msgs::msg::Path StraightLinePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::vector<geometry_msgs::msg::PoseStamped> & viapoints,
  std::function<bool()> cancel_checker)
{
  nav_msgs::msg::Path path;
  auto node = node_.lock();
  path.header.stamp = node ? node->now() : rclcpp::Time();
  path.header.frame_id = global_frame_;

  // PlannerServer 已把 start/goal 变换到全局坐标系；这里做防御性校验，
  // 坐标系不一致直接抛 nav2_core 异常族，由服务端转成 action 错误码
  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    throw nav2_core::PlannerTFError(
      "StraightLinePlanner 只接受 " + global_frame_ + " 坐标系下的位姿");
  }

  // 途经点串联：start -> via[0] -> ... -> via[n-1] -> goal
  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  waypoints.push_back(start);
  waypoints.insert(waypoints.end(), viapoints.begin(), viapoints.end());
  waypoints.push_back(goal);

  for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
    appendSegment(waypoints[i], waypoints[i + 1], path, cancel_checker);
  }
  path.poses.push_back(goal);  // 终点姿态精确入列（保留 goal 朝向）

  RCLCPP_DEBUG(logger_, "生成直线路径，共 %zu 个位姿", path.poses.size());
  return path;
}

void StraightLinePlanner::appendSegment(
  const geometry_msgs::msg::PoseStamped & from,
  const geometry_msgs::msg::PoseStamped & to,
  nav_msgs::msg::Path & path,
  const std::function<bool()> & cancel_checker) const
{
  const double dx = to.pose.position.x - from.pose.position.x;
  const double dy = to.pose.position.y - from.pose.position.y;
  const int total = std::max(
    1, static_cast<int>(std::hypot(dx, dy) / interpolation_resolution_));

  for (int i = 0; i < total; ++i) {
    // 长路径逐点检查取消请求：官方 NavfnPlanner 也是把 cancel_checker
    // 一路传进搜索循环，收到取消即抛 PlannerCancelled
    if (cancel_checker()) {
      throw nav2_core::PlannerCancelled("StraightLinePlanner 规划被取消");
    }
    const double ratio = static_cast<double>(i) / total;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = from.pose.position.x + dx * ratio;
    pose.pose.position.y = from.pose.position.y + dy * ratio;
    pose.pose.orientation.w = 1.0;  // 演示用单位四元数；实际可按段方向算 yaw
    path.poses.push_back(pose);
  }
}

}  // namespace my_nav2_examples

// pluginlib 导出宏：参数一=实现类，参数二=接口基类（与 plugins.xml 严格一致）
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_nav2_examples::StraightLinePlanner, nav2_core::GlobalPlanner)
```

## 4. 插件导出：plugins.xml + CMakeLists.txt + package.xml

### 4.1 straight_line_planner_plugin.xml

写法对照官方 `nav2_navfn_planner/global_planner_plugin.xml`：

```xml
<library path="my_nav2_examples_planner">
  <class type="my_nav2_examples::StraightLinePlanner"
         base_class_type="nav2_core::GlobalPlanner">
    <description>教学用直线规划器（无避障）</description>
  </class>
</library>
```

### 4.2 CMakeLists.txt 关键段（对照 nav2_navfn_planner/CMakeLists.txt）

```cmake
cmake_minimum_required(VERSION 3.20)
project(my_nav2_examples_planner)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package()   # 来自 nav2_common：统一编译选项（C++ 标准、告警等）

add_library(${PROJECT_NAME} SHARED src/straight_line_planner.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC
  nav2_core::nav2_core
  nav2_ros_common::nav2_ros_common
  nav2_costmap_2d::nav2_costmap_2d_core
  nav_msgs::nav_msgs
  geometry_msgs::geometry_msgs
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros)
target_link_libraries(${PROJECT_NAME} PRIVATE pluginlib::pluginlib)

# 关键：向 nav2_core 包注册插件描述文件，planner_server 才能发现本插件
pluginlib_export_plugin_description_file(nav2_core straight_line_planner_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(FILES straight_line_planner_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(nav2_core nav2_costmap_2d nav2_ros_common nav_msgs
  geometry_msgs rclcpp rclcpp_lifecycle tf2_ros)
ament_export_targets(${PROJECT_NAME})
ament_package()
```

### 4.3 package.xml 关键段

```xml
<name>my_nav2_examples_planner</name>
<buildtool_depend>ament_cmake</buildtool_depend>
<build_depend>nav2_common</build_depend>
<depend>nav2_core</depend>
<depend>nav2_costmap_2d</depend>
<depend>nav2_ros_common</depend>
<depend>nav_msgs</depend>
<depend>geometry_msgs</depend>
<depend>pluginlib</depend>
<depend>rclcpp</depend>
<depend>rclcpp_lifecycle</depend>
<depend>tf2_ros</depend>
<export><build_type>ament_cmake</build_type></export>
```

## 5. YAML 挂载配置

替换 `nav2_bringup/params/nav2_params.yaml` 中 `planner_server` 段的 `GridBased` 插件（官方默认是 `nav2_navfn_planner::NavfnPlanner`）：

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "my_nav2_examples::StraightLinePlanner"   # plugins.xml 中的 class type
      interpolation_resolution: 0.1                     # 本插件自定义参数
```

## 6. 运行验证

```bash
# 1. 编译（工作空间根目录）
colcon build --packages-select my_nav2_examples_planner --symlink-install
source install/setup.bash

# 2. 确认 pluginlib 能发现插件（应输出 my_nav2_examples/straight_line_planner_plugin.xml 路径）
ros2 pkg xml my_nav2_examples_planner | grep -A1 nav2_core

# 3. 带自定义参数文件启动仿真
ros2 launch nav2_bringup tb3_simulation_launch.py params_file:=/path/to/my_params.yaml

# 4. RViz 里点 "Nav2 Goal" 下发目标；或命令行直接触发规划动作：
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0}}}, use_start: false}"

# 5. 观察 /plan 话题应是一条直线路径
ros2 topic echo /plan --once
```

## 7. 常见错误排查

1. `Failed to create global planner. Exception: According to the loaded plugin descriptions the class ... does not exist`——YAML 的 `plugin:` 值与 plugins.xml 的 `class type` 不一致，或 CMake 忘了 `pluginlib_export_plugin_description_file(nav2_core ...)`，或编译后没有重新 `source install/setup.bash`。
2. 编译报错 `no matching function ... configure`——照抄了旧版教程签名（`rclcpp_lifecycle::LifecycleNode::WeakPtr` / 无 viapoints 的 `createPlan`）。1.5.0 签名以第 2 节表格为准，`override` 关键字能让编译器帮你尽早发现不匹配。
3. 链接错误 `undefined reference to typeinfo for nav2_core::GlobalPlanner`——`target_link_libraries` 缺 `nav2_core::nav2_core`。
4. 运行时路径不显示——终点/起点 frame 不是 `map`，本例直接抛 `PlannerTFError`；查 planner_server 日志中的 error_msg。
5. 目标取消后 planner_server 卡住——长循环里没有调用 `cancel_checker()`；参考本例在插值循环里逐点检查。

## 8. 本示例对应源码（映射链）

| 环节 | 位置 |
|---|---|
| 示例类 | `my_nav2_examples::StraightLinePlanner`（本文） |
| 对应官方实现 | `nav2_navfn_planner/src/navfn_planner.cpp`（约 530 行，锚点 `NavfnPlanner::createPlan`、`NavfnPlanner::makePlan`） |
| 接口头文件 | `nav2_core/include/nav2_core/global_planner.hpp`（约 88 行，5 个纯虚函数） |
| pluginlib 注册处 | `navfn_planner.cpp` 文件末尾 `PLUGINLIB_EXPORT_CLASS(nav2_navfn_planner::NavfnPlanner, nav2_core::GlobalPlanner)` + `nav2_navfn_planner/global_planner_plugin.xml` + `nav2_navfn_planner/CMakeLists.txt` 中 `pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)` |
| 官方 YAML 例子 | `nav2_bringup/params/nav2_params.yaml` 中 `planner_server` 段（锚点 `planner_plugins: ["GridBased"]`，`GridBased.plugin: "nav2_navfn_planner::NavfnPlanner"`） |

闭环路线：先读接口头文件确认契约 → 对照 NavfnPlanner 官方实现看工业级写法（代价地图搜索、容差、异常族）→ 用本示例骨架替换算法内核实现自己的规划器。进阶算法可继续研读 [08 SmacPlanner](../08_SmacPlanner.md)。
