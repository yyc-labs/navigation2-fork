# 实战示例：从零实现一个纯追踪简化控制器插件（Controller）

> 测试口径：Nav2 1.5.0（main 分支），C++20。
> 相关章节：[09 ControllerServer 与局部控制](../09_ControllerServer与局部控制.md) · [10 MPPI 与 RPP 控制器](../10_MPPI与RPP控制器.md) · [04 nav2_core 插件接口](../04_nav2_core插件接口.md)

## 1. 目标

做什么：实现一个极简纯追踪（Pure Pursuit）控制器插件 `my_nav2_examples::TinyPursuitController`——在已裁剪变换的局部路径上取一个固定前视距离的目标点，按曲率公式输出 `(v, w)`。不做碰撞检查、不做速度调节（那是官方 RPP 的 "Regulated" 部分）。

学到什么：

- `nav2_core::Controller` 接口的 7 个纯虚函数与 2 个可选虚函数（`cancel()`、`reset()`）；
- 1.5.0 控制器接口的关键变化：新增 `newPathReceived()` 回调，`computeVelocityCommands()` 直接收到**已变换裁剪的局部路径**与全局终点（路径处理由 ControllerServer 的 path_handler 插件完成）；
- 限速接口 `setSpeedLimit` 的绝对值/百分比双语义。

## 2. 最小实现面：必须 override 的纯虚函数

以下签名逐字核实自 `nav2_core/include/nav2_core/controller.hpp`（约 153 行）：

| 纯虚函数 | 调用时机 |
|---|---|
| `virtual void configure(const nav2::LifecycleNode::WeakPtr &, std::string name, nav2::TransformBuffer::SharedPtr, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) = 0;` | ControllerServer `on_configure` |
| `virtual void cleanup() = 0;` | `on_cleanup` |
| `virtual void activate() = 0;` | `on_activate` |
| `virtual void deactivate() = 0;` | `on_deactivate` |
| `virtual void newPathReceived(const nav_msgs::msg::Path & raw_global_path) = 0;` | 每次收到新全局路径（只做轻量工作，如复位内部状态） |
| `virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * goal_checker, const nav_msgs::msg::Path & transformed_global_plan, const geometry_msgs::msg::PoseStamped & global_goal) = 0;` | 每个控制周期（`controller_frequency`） |
| `virtual void setSpeedLimit(const double & speed_limit, const bool & percentage) = 0;` | 收到 `/speed_limit` 话题限速时 |

非纯虚、可选 override（同头文件）：`virtual bool cancel()`（默认返回 `true`，返回 `false` 表示"还没停稳，继续调我"）、`virtual void reset()`（任务退出时复位状态）。

## 3. 完整代码

工程布局（独立 ament 包 `my_nav2_examples_controller`）：

```
my_nav2_examples_controller/
├── include/my_nav2_examples_controller/tiny_pursuit_controller.hpp
├── src/tiny_pursuit_controller.cpp
├── tiny_pursuit_controller_plugin.xml
├── CMakeLists.txt
└── package.xml
```

### 3.1 tiny_pursuit_controller.hpp

```cpp
#ifndef MY_NAV2_EXAMPLES_CONTROLLER__TINY_PURSUIT_CONTROLLER_HPP_
#define MY_NAV2_EXAMPLES_CONTROLLER__TINY_PURSUIT_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "nav2_core/controller.hpp"              // 基类接口（纯虚）
#include "nav2_core/controller_exceptions.hpp"   // NoValidControl 等异常族
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/tf2_factories.hpp"

namespace my_nav2_examples
{

/**
 * @class TinyPursuitController
 * @brief 教学用极简纯追踪控制器：固定前视距离 + 曲率跟踪，
 *        无碰撞检查、无速度调节。工业级实现见官方 RPP 控制器。
 */
class TinyPursuitController : public nav2_core::Controller
{
public:
  TinyPursuitController() = default;
  ~TinyPursuitController() override = default;

  // ---- 7 个纯虚函数 ----
  void configure(
    const nav2::LifecycleNode::WeakPtr & parent,
    std::string name, nav2::TransformBuffer::SharedPtr tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void newPathReceived(const nav_msgs::msg::Path & raw_global_path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker,
    const nav_msgs::msg::Path & transformed_global_plan,
    const geometry_msgs::msg::PoseStamped & global_goal) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  // ---- 可选虚函数 ----
  void reset() override;

private:
  nav2::LifecycleNode::WeakPtr node_;
  nav2::TransformBuffer::SharedPtr tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("TinyPursuitController")};
  std::string name_;

  double desired_linear_vel_{0.3};   // 巡航线速度 (m/s)
  double lookahead_dist_{0.6};       // 前视距离 (m)
  double max_angular_vel_{1.0};      // 角速度限幅 (rad/s)
  double speed_scale_{1.0};          // setSpeedLimit 产生的缩放系数
};

}  // namespace my_nav2_examples

#endif  // MY_NAV2_EXAMPLES_CONTROLLER__TINY_PURSUIT_CONTROLLER_HPP_
```

### 3.2 tiny_pursuit_controller.cpp

```cpp
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "my_nav2_examples_controller/tiny_pursuit_controller.hpp"
#include "tf2/utils.hpp"   // tf2::getYaw

namespace my_nav2_examples
{

void TinyPursuitController::configure(
  const nav2::LifecycleNode::WeakPtr & parent,
  std::string name, nav2::TransformBuffer::SharedPtr tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"TinyPursuitController: 无法锁定宿主节点"};
  }

  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node->get_logger();

  // 参数全名形如 controller_server.FollowPath.desired_linear_vel
  desired_linear_vel_ = node->declare_or_get_parameter(
    name_ + ".desired_linear_vel", 0.3);
  lookahead_dist_ = node->declare_or_get_parameter(
    name_ + ".lookahead_dist", 0.6);
  max_angular_vel_ = node->declare_or_get_parameter(
    name_ + ".max_angular_vel", 1.0);

  RCLCPP_INFO(logger_, "已配置控制器 %s：v=%.2f, Ld=%.2f",
    name_.c_str(), desired_linear_vel_, lookahead_dist_);
}

void TinyPursuitController::cleanup()
{
  RCLCPP_INFO(logger_, "清理控制器 %s", name_.c_str());
  costmap_ros_.reset();
  tf_.reset();
}

void TinyPursuitController::activate()
{
  RCLCPP_INFO(logger_, "激活控制器 %s", name_.c_str());
}

void TinyPursuitController::deactivate()
{
  RCLCPP_INFO(logger_, "停用控制器 %s", name_.c_str());
}

void TinyPursuitController::newPathReceived(const nav_msgs::msg::Path & raw_global_path)
{
  // 接口约定：这里只做轻量工作。局部裁剪/变换由 path_handler 插件负责，
  // computeVelocityCommands 会直接拿到 transformed_global_plan
  RCLCPP_DEBUG(logger_, "收到新全局路径，%zu 个位姿", raw_global_path.poses.size());
  speed_scale_ = 1.0;  // 新任务复位限速缩放之外的内部状态（本例无更多状态）
}

geometry_msgs::msg::TwistStamped TinyPursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/,
  const nav_msgs::msg::Path & transformed_global_plan,
  const geometry_msgs::msg::PoseStamped & /*global_goal*/)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header = pose.header;

  if (transformed_global_plan.poses.empty()) {
    // 无可跟踪路径：抛控制器异常族，ControllerServer 会转成 action 错误码
    throw nav2_core::NoValidControl("TinyPursuitController: 局部路径为空");
  }

  // 1) 在局部路径上找第一个距离机器人 >= 前视距离的点（找不到则取最后一点）
  const auto & poses = transformed_global_plan.poses;
  auto target = poses.back();
  for (const auto & p : poses) {
    const double dx = p.pose.position.x - pose.pose.position.x;
    const double dy = p.pose.position.y - pose.pose.position.y;
    if (std::hypot(dx, dy) >= lookahead_dist_) {
      target = p;
      break;
    }
  }

  // 2) 把前视点变换到机器人本体坐标系（pose 与路径同坐标系，手工旋转即可）
  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double dx = target.pose.position.x - pose.pose.position.x;
  const double dy = target.pose.position.y - pose.pose.position.y;
  const double local_x = std::cos(-yaw) * dx - std::sin(-yaw) * dy;
  const double local_y = std::sin(-yaw) * dx + std::cos(-yaw) * dy;

  // 3) 纯追踪曲率公式：kappa = 2*y / Ld^2，w = v * kappa
  const double ld2 = std::max(1e-6, local_x * local_x + local_y * local_y);
  const double curvature = 2.0 * local_y / ld2;

  const double v = desired_linear_vel_ * speed_scale_;
  cmd.twist.linear.x = v;
  cmd.twist.angular.z = std::clamp(v * curvature, -max_angular_vel_, max_angular_vel_);
  return cmd;
}

void TinyPursuitController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit <= 0.0) {
    speed_scale_ = 1.0;                                   // 非正值 = 解除限速
  } else if (percentage) {
    speed_scale_ = std::clamp(speed_limit / 100.0, 0.0, 1.0);   // 百分比语义
  } else {
    speed_scale_ = std::clamp(speed_limit / desired_linear_vel_, 0.0, 1.0);  // 绝对值语义
  }
}

void TinyPursuitController::reset()
{
  speed_scale_ = 1.0;
}

}  // namespace my_nav2_examples

// pluginlib 导出宏：与官方 RPP 文件末尾写法一致
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  my_nav2_examples::TinyPursuitController,
  nav2_core::Controller)
```

## 4. 插件导出：plugins.xml + CMakeLists.txt + package.xml

### 4.1 tiny_pursuit_controller_plugin.xml

写法对照官方 `nav2_regulated_pure_pursuit_controller/nav2_regulated_pure_pursuit_controller.xml`：

```xml
<class_libraries>
  <library path="my_nav2_examples_controller">
    <class type="my_nav2_examples::TinyPursuitController"
           base_class_type="nav2_core::Controller">
      <description>教学用极简纯追踪控制器</description>
    </class>
  </library>
</class_libraries>
```

### 4.2 CMakeLists.txt 关键段

```cmake
cmake_minimum_required(VERSION 3.20)
project(my_nav2_examples_controller)

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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package()

add_library(${PROJECT_NAME} SHARED src/tiny_pursuit_controller.cpp)
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
  tf2::tf2
  tf2_ros::tf2_ros)
target_link_libraries(${PROJECT_NAME} PRIVATE pluginlib::pluginlib)

# 关键：控制器插件同样注册到 nav2_core 包名下
pluginlib_export_plugin_description_file(nav2_core tiny_pursuit_controller_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(FILES tiny_pursuit_controller_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(nav2_core nav2_costmap_2d nav2_ros_common nav_msgs
  geometry_msgs rclcpp rclcpp_lifecycle tf2 tf2_ros)
ament_export_targets(${PROJECT_NAME})
ament_package()
```

### 4.3 package.xml 关键段

```xml
<name>my_nav2_examples_controller</name>
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
<depend>tf2</depend>
<depend>tf2_ros</depend>
<export><build_type>ament_cmake</build_type></export>
```

## 5. YAML 挂载配置

替换 `controller_server` 段的 `FollowPath` 插件（官方默认是 `nav2_mppi_controller::MPPIController`）；注意 1.5.0 中 `path_handler_plugins` 是独立插件链，负责给控制器供给局部路径：

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
    path_handler_plugins: ["PathHandler"]
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
    PathHandler:
      plugin: "nav2_controller::FeasiblePathHandler"
    FollowPath:
      plugin: "my_nav2_examples::TinyPursuitController"  # plugins.xml 中的 class type
      desired_linear_vel: 0.3
      lookahead_dist: 0.6
      max_angular_vel: 1.0
```

## 6. 运行验证

```bash
# 1. 编译
colcon build --packages-select my_nav2_examples_controller --symlink-install
source install/setup.bash

# 2. 带自定义参数启动仿真
ros2 launch nav2_bringup tb3_simulation_launch.py params_file:=/path/to/my_params.yaml

# 3. RViz 下发 Nav2 Goal，观察机器人沿全局路径追踪行驶
# 4. 观察控制量输出
ros2 topic echo /cmd_vel

# 5. 验证限速接口（百分比语义，降到 50%）
ros2 topic pub --once /speed_limit nav2_msgs/msg/SpeedLimit \
  "{percentage: true, speed_limit: 50.0}"
```

## 7. 常见错误排查

1. `Failed to create controller ... does not exist`——YAML 的 `plugin:` 与 plugins.xml 的 `class type` 不一致，或漏了 `pluginlib_export_plugin_description_file(nav2_core ...)`。
2. 编译报 `cannot instantiate abstract class`——漏实现纯虚函数。1.5.0 新增了 `newPathReceived()`，且 `computeVelocityCommands` 是 5 参数版本（多了 `transformed_global_plan`、`global_goal`），旧教程的 3 参数版本编译不过。
3. 机器人原地打转——前视点变换符号写反（`local_y` 的符号决定转向），或 `lookahead_dist` 小于路径点间距导致始终取到身后点。
4. 到点后不停车——停车判定不归控制器管：由 `goal_checker` 判定到达、ControllerServer 停止调用控制器。检查 `general_goal_checker` 的容差参数而不是改控制器。
5. 收到新路径后行为异常——把重活（如全路径搜索）放进了 `newPathReceived()`。接口注释明确要求它只做轻量状态复位，重活放在 `computeVelocityCommands()`。

## 8. 本示例对应源码（映射链）

| 环节 | 位置 |
|---|---|
| 示例类 | `my_nav2_examples::TinyPursuitController`（本文） |
| 对应官方实现 | `nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp`（约 440 行，锚点 `RegulatedPurePursuitController::computeVelocityCommands`、`getLookAheadPoint`） |
| 接口头文件 | `nav2_core/include/nav2_core/controller.hpp`（约 153 行，7 纯虚 + `cancel()`/`reset()` 可选） |
| pluginlib 注册处 | `regulated_pure_pursuit_controller.cpp` 文件末尾 `PLUGINLIB_EXPORT_CLASS(nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController, nav2_core::Controller)` + `nav2_regulated_pure_pursuit_controller/nav2_regulated_pure_pursuit_controller.xml` |
| 官方 YAML 例子 | `nav2_bringup/params/nav2_params.yaml` 中 `controller_server` 段（锚点 `controller_plugins: ["FollowPath"]`、`path_handler_plugins: ["PathHandler"]`） |

闭环路线：接口头文件确认契约 → 对照官方 RPP 看完整版（速度调节、碰撞前瞻、动态参数）→ 用本示例骨架实现自己的控制律。MPPI 等优化型控制器见 [10 MPPI 与 RPP 控制器](../10_MPPI与RPP控制器.md)。
