# 实战示例：用 SimpleActionServer 写一个自定义 Action 服务器

> 测试口径：Nav2 1.5.0（main 分支），C++20。
> 相关章节：[19 Action 与通信机制](../19_Action与通信机制.md) · [20 线程模型](../20_线程模型.md) · [05 生命周期管理](../05_生命周期管理.md)

## 1. 目标

做什么：用 Nav2 的 `SimpleActionServer` 实现一个独立可运行的等待服务器 `ExampleWaitServer`——接收 `nav2_msgs/action/Wait` 目标，倒计时期间发布剩余时间反馈，支持取消与抢占（preemption），行为等价于官方 Wait 恢复行为的"单文件版"。

学到什么：

- `SimpleActionServer` 对 `rclcpp_action` 的封装价值：单目标 + 待决槽（pending slot）抢占模型、独立工作线程执行、异常兜底终止；
- 1.5.0 重要迁移事实：该类**已从 `nav2_util` 迁到 `nav2_ros_common` 包，命名空间从 `nav2_util::` 变为 `nav2::`**（头文件 `nav2_ros_common/simple_action_server.hpp`，现场核实；`nav2_util/include/nav2_util/` 下已无此文件）；
- 通过 `nav2::LifecycleNode::create_action_server<ActionT>()` 工厂创建并绑定生命周期。

## 2. 最小实现面：核心 API（现场核实签名）

`SimpleActionServer` 是模板类而非插件接口，**没有纯虚函数**——你的契约是提供 `ExecuteCallback` 回调并在其中驱动下述 API。以下签名逐字核实自 `nav2_ros_common/include/nav2_ros_common/simple_action_server.hpp`（约 683 行）：

| 成员 | 签名（节选） | 用途 |
|---|---|---|
| 类型 | `template<typename ActionT> class SimpleActionServer`，`typedef std::function<void ()> ExecuteCallback;`，`typedef std::function<void ()> CompletionCallback;` | 执行/完成回调均为无参 `void()` |
| 构造 | `template<typename NodeT> explicit SimpleActionServer(NodeT node, const std::string & action_name, ExecuteCallback execute_callback, CompletionCallback completion_callback = nullptr, std::chrono::milliseconds server_timeout = std::chrono::milliseconds(500), bool spin_thread = false, const bool realtime = false)` | 通常不直接调用，走 LifecycleNode 工厂 |
| 生命周期 | `void activate()` / `void deactivate()` | 挂到宿主节点 `on_activate`/`on_deactivate` |
| 取目标 | `const std::shared_ptr<const typename ActionT::Goal> get_current_goal() const` | 执行回调里第一件事 |
| 取消 | `bool is_cancel_requested() const` | 循环内轮询 |
| 抢占 | `bool is_preempt_requested() const` + `const std::shared_ptr<const typename ActionT::Goal> accept_pending_goal()` | 新目标到来时无缝切换 |
| 收尾 | `void succeeded_current(...result)` / `void terminate_current(...result)` / `void terminate_all(...result)` | 三种终态；result 参数默认 `std::make_shared<typename ActionT::Result>()` |
| 反馈 | `void publish_feedback(typename std::shared_ptr<typename ActionT::Feedback> feedback)` | 循环内周期发布 |

工厂方法（核实自 `nav2_ros_common/include/nav2_ros_common/lifecycle_node.hpp`，约 476 行）：

```cpp
template<typename ActionT>
typename nav2::SimpleActionServer<ActionT>::SharedPtr
create_action_server(
  const std::string & action_name,
  typename nav2::SimpleActionServer<ActionT>::ExecuteCallback execute_callback,
  typename nav2::SimpleActionServer<ActionT>::CompletionCallback compl_cb = nullptr,
  std::chrono::milliseconds server_timeout = std::chrono::milliseconds(500),
  bool spin_thread = false,
  const bool realtime = false);
```

所用 Action 定义（`nav2_msgs/action/Wait.action`，现场核实）：goal 为 `builtin_interfaces/Duration time`；result 为 `builtin_interfaces/Duration total_elapsed_time` + `uint16 error_code`（`NONE=0`、`UNKNOWN=740`、`TIMEOUT=741`）+ `string error_msg`；feedback 为 `builtin_interfaces/Duration time_left`。

## 3. 完整代码

工程布局（独立 ament 包 `my_nav2_examples_action`，产物是可执行节点，不是插件）：

```
my_nav2_examples_action/
├── include/my_nav2_examples_action/example_wait_server.hpp
├── src/example_wait_server.cpp
├── src/main.cpp
├── CMakeLists.txt
└── package.xml
```

### 3.1 example_wait_server.hpp

```cpp
#ifndef MY_NAV2_EXAMPLES_ACTION__EXAMPLE_WAIT_SERVER_HPP_
#define MY_NAV2_EXAMPLES_ACTION__EXAMPLE_WAIT_SERVER_HPP_

#include <memory>

#include "nav2_ros_common/lifecycle_node.hpp"         // nav2::LifecycleNode
#include "nav2_ros_common/simple_action_server.hpp"   // nav2::SimpleActionServer
#include "nav2_msgs/action/wait.hpp"

namespace my_nav2_examples
{

/**
 * @class ExampleWaitServer
 * @brief 教学用等待服务器：Wait action 的最小完整实现，
 *        覆盖 SimpleActionServer 的取消 / 抢占 / 反馈 / 三种终态。
 */
class ExampleWaitServer : public nav2::LifecycleNode
{
public:
  using WaitAction = nav2_msgs::action::Wait;

  explicit ExampleWaitServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ExampleWaitServer() override = default;

protected:
  // ---- 生命周期回调（签名与官方 VelocitySmoother 等服务器一致）----
  nav2::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /// ExecuteCallback：SimpleActionServer 在独立工作线程中调用（std::async）
  void execute();

  nav2::SimpleActionServer<WaitAction>::SharedPtr action_server_;
};

}  // namespace my_nav2_examples

#endif  // MY_NAV2_EXAMPLES_ACTION__EXAMPLE_WAIT_SERVER_HPP_
```

### 3.2 example_wait_server.cpp

```cpp
#include <memory>

#include "my_nav2_examples_action/example_wait_server.hpp"

namespace my_nav2_examples
{

ExampleWaitServer::ExampleWaitServer(const rclcpp::NodeOptions & options)
: nav2::LifecycleNode("example_wait_server", "", options)
{
  RCLCPP_INFO(get_logger(), "创建 ExampleWaitServer（未配置）");
}

nav2::CallbackReturn
ExampleWaitServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "on_configure：创建 action server");

  // 工厂方法自动串好节点接口；execute 回调将在工作线程中被调用
  action_server_ = create_action_server<WaitAction>(
    "example_wait",
    std::bind(&ExampleWaitServer::execute, this));

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
ExampleWaitServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "on_activate：开始接单");
  action_server_->activate();   // 未激活时 handle_goal 会直接 REJECT
  createBond();                 // 与 lifecycle_manager 建立 bond 心跳
  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
ExampleWaitServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "on_deactivate：停止接单");
  action_server_->deactivate();  // 等待工作线程收尾（server_timeout 兜底）
  destroyBond();
  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
ExampleWaitServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  action_server_.reset();
  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
ExampleWaitServer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  return nav2::CallbackReturn::SUCCESS;
}

void ExampleWaitServer::execute()
{
  auto goal = action_server_->get_current_goal();
  if (!goal) {
    return;  // 目标已失效（极小概率的竞态防御）
  }

  auto result = std::make_shared<WaitAction::Result>();
  rclcpp::Time start_time = now();
  rclcpp::Duration wait_duration(goal->time);
  rclcpp::Rate loop_rate(10.0);   // 10 Hz 轮询取消/抢占并发反馈

  RCLCPP_INFO(get_logger(), "开始等待 %.1f 秒", wait_duration.seconds());

  while (rclcpp::ok()) {
    // 1) 取消：置 canceled 终态（terminate_* 内部区分 cancel/abort）
    if (action_server_->is_cancel_requested()) {
      result->total_elapsed_time = now() - start_time;
      result->error_code = WaitAction::Result::NONE;
      action_server_->terminate_all(result);
      RCLCPP_INFO(get_logger(), "等待被取消");
      return;
    }

    // 2) 抢占：新目标顶替旧目标，重新计时（旧目标被 abort）
    if (action_server_->is_preempt_requested()) {
      goal = action_server_->accept_pending_goal();
      start_time = now();
      wait_duration = rclcpp::Duration(goal->time);
      RCLCPP_INFO(get_logger(), "被新目标抢占，重新等待 %.1f 秒", wait_duration.seconds());
    }

    // 3) 完成判定
    const rclcpp::Duration elapsed = now() - start_time;
    if (elapsed >= wait_duration) {
      result->total_elapsed_time = elapsed;
      result->error_code = WaitAction::Result::NONE;
      action_server_->succeeded_current(result);
      RCLCPP_INFO(get_logger(), "等待完成");
      return;
    }

    // 4) 周期反馈：剩余时间
    auto feedback = std::make_shared<WaitAction::Feedback>();
    feedback->time_left = wait_duration - elapsed;
    action_server_->publish_feedback(feedback);

    loop_rate.sleep();
  }

  // rclcpp 关闭：按失败收尾（execute 内抛异常也会被 SimpleActionServer 捕获并 terminate_all）
  result->error_code = WaitAction::Result::UNKNOWN;
  result->error_msg = "rclcpp 已关闭";
  action_server_->terminate_all(result);
}

}  // namespace my_nav2_examples
```

### 3.3 main.cpp

```cpp
#include <memory>
#include "my_nav2_examples_action/example_wait_server.hpp"
#include "rclcpp/rclcpp.hpp"

// 写法对照官方 nav2_velocity_smoother/src/main.cpp
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<my_nav2_examples::ExampleWaitServer>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
```

## 4. 工程关键段（无需 plugins.xml：action server 不是 pluginlib 插件）

### 4.1 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.20)
project(my_nav2_examples_action)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

nav2_package()

add_executable(example_wait_server
  src/example_wait_server.cpp
  src/main.cpp)
target_include_directories(example_wait_server PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(example_wait_server
  nav2_ros_common::nav2_ros_common
  nav2_msgs::nav2_msgs
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle)

install(TARGETS example_wait_server RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_package()
```

### 4.2 package.xml 关键段

```xml
<name>my_nav2_examples_action</name>
<buildtool_depend>ament_cmake</buildtool_depend>
<build_depend>nav2_common</build_depend>
<depend>nav2_msgs</depend>
<depend>nav2_ros_common</depend>
<depend>rclcpp</depend>
<depend>rclcpp_action</depend>
<depend>rclcpp_lifecycle</depend>
<export><build_type>ament_cmake</build_type></export>
```

## 5. YAML 挂载配置

本节点是生命周期节点，可交给 `lifecycle_manager` 统一拉起（详见 [LifecycleNode.md](LifecycleNode.md)）；单独调试时用 `autostart_node` 参数自动配置+激活（该参数由 `nav2::LifecycleNode` 构造函数内 `declare_or_get_parameter("autostart_node", false)` 声明，现场核实）：

```yaml
example_wait_server:
  ros__parameters:
    autostart_node: true        # 免手动 lifecycle set，仅限调试
    bond_heartbeat_period: 0.25 # 置 0.0 可禁用 bond（无 manager 场景）
```

## 6. 运行验证

```bash
# 1. 编译
colcon build --packages-select my_nav2_examples_action --symlink-install
source install/setup.bash

# 2. 启动（方式 A：参数自动激活）
ros2 run my_nav2_examples_action example_wait_server \
  --ros-args -p autostart_node:=true -p bond_heartbeat_period:=0.0

#    启动（方式 B：手动走生命周期）
ros2 lifecycle set /example_wait_server configure
ros2 lifecycle set /example_wait_server activate

# 3. 发送目标并观察反馈（time_left 每 0.1 秒递减）
ros2 action send_goal --feedback /example_wait nav2_msgs/action/Wait "{time: {sec: 5}}"

# 4. 验证抢占：上一条未结束时再开一个终端发新目标，旧目标应返回 ABORTED
# 5. 验证取消：send_goal 过程中 Ctrl+C，服务器日志应打印"等待被取消"
```

## 7. 常见错误排查

1. `Action server is inactive. Rejecting the goal.`——节点未激活就发目标。`activate()` 前 `handle_goal` 一律 REJECT（头文件 `handle_goal` 实现可见此逻辑）；先 `ros2 lifecycle set ... activate`。
2. 编译找不到 `nav2_util/simple_action_server.hpp`——1.5.0 已迁移，改用 `nav2_ros_common/simple_action_server.hpp` 与 `nav2::SimpleActionServer`（老博客/教程里的 `nav2_util::SimpleActionServer` 均为旧口径）。
3. 反馈收不到——`publish_feedback` 要求当前 goal handle 处于 active；若在 `succeeded_current`/`terminate_*` 之后继续发反馈会打印 `Trying to publish feedback when the current goal handle is not active`。
4. deactivate 卡 500ms 后报 `Action callback is still running and missed deadline to stop`——execute 循环没有及时退出。确保循环内轮询 `is_cancel_requested()` 且周期小于 `server_timeout`（构造参数，默认 500ms）。
5. 第二个目标一直不执行——execute 回调返回前没有处理抢占。SimpleActionServer 是"单执行线程+一个待决槽"模型：要么循环内 `accept_pending_goal()` 接手，要么让当前回调尽快结束，`work()` 会自动续跑待决目标。

## 8. 本示例对应源码（映射链）

| 环节 | 位置 |
|---|---|
| 示例类 | `my_nav2_examples::ExampleWaitServer`（本文） |
| 对应官方实现 | `nav2_planner/src/planner_server.cpp`（约 690 行，锚点 `create_action_server<ActionToPose>`、`PlannerServer::computePlan` 中的 `is_cancel_requested`/`is_preempt_requested` 用法）；Wait 语义参照 `nav2_behaviors/plugins/wait.cpp` |
| 接口头文件 | `nav2_ros_common/include/nav2_ros_common/simple_action_server.hpp`（约 683 行，锚点 `handle_accepted`、`work`）；Action 定义 `nav2_msgs/action/Wait.action` |
| 注册处 | 非 pluginlib 插件，无导出宏；创建入口是 `nav2_ros_common/include/nav2_ros_common/lifecycle_node.hpp` 中的工厂 `create_action_server`（约 476 行文件，锚点 `create_action_server`） |
| 官方 YAML 例子 | `nav2_bringup/params/nav2_params.yaml` 中 `planner_server` 段——`/compute_path_to_pose` 等 action 服务器即按本文同款模式在 `on_configure` 中创建 |

闭环路线：读 simple_action_server.hpp 弄懂"当前句柄 + 待决槽"状态机（`handle_accepted` → `work`）→ 对照 PlannerServer 看多 action 服务器的工程组织 → 用本示例骨架为自己的长任务（建图、标定、巡检）包一个可取消、可抢占的 action 接口。
