# 实战示例：用 nav2 LifecycleNode 写一个受 lifecycle_manager 管理的节点（含 bond）

> 测试口径：Nav2 1.5.0（main 分支），C++20。
> 相关章节：[05 生命周期管理](../05_生命周期管理.md) · [21 配置系统](../21_配置系统.md) · [19 Action 与通信机制](../19_Action与通信机制.md)

## 1. 目标

做什么：实现一个最小的被管节点 `ExampleManagedNode`——一个周期发布心跳消息的生命周期节点，实现全部 5 个生命周期回调，在 `on_activate`/`on_deactivate` 中建立/销毁与 `lifecycle_manager` 的 bond 心跳连接，并交由 lifecycle_manager 统一拉起与看护。

学到什么：

- `nav2::LifecycleNode` 相比裸 `rclcpp_lifecycle::LifecycleNode` 的增值：bond 管理（`createBond`/`destroyBond`）、`autostart_node` 自启、rcl 预关闭自动清理（`on_rcl_preshutdown` → `runCleanups`）、`declare_or_get_parameter`、Nav2 风味的 `create_publisher/subscription/service/action_server` 工厂；
- 1.5.0 重要迁移事实：该类**已从 `nav2_util` 迁到 `nav2_ros_common` 包，命名空间从 `nav2_util::` 变为 `nav2::`**（头文件 `nav2_ros_common/lifecycle_node.hpp`，现场核实；`nav2_util/include/nav2_util/` 下已无 lifecycle_node.hpp）；
- bond 机制：被管节点激活后发心跳，manager 掉线检测触发系统降级。

## 2. 最小实现面：需要 override 的回调（现场核实签名）

`nav2::LifecycleNode` **没有纯虚函数**——契约是按需 override 生命周期转换回调。以下事实核实自 `nav2_ros_common/include/nav2_ros_common/lifecycle_node.hpp`（约 476 行）：

- 返回类型别名：`using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;`（即 `nav2::CallbackReturn`，取值 `SUCCESS`/`FAILURE`/`ERROR`）；
- 构造函数：`LifecycleNode(const std::string & node_name, const std::string & ns, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())`，另有无命名空间重载；构造时自动声明 `bond_heartbeat_period`（默认 `0.25` 秒）与 `autostart_node`（默认 `false`）参数；
- bond 接口：`void createBond()`（bond 话题固定为 `"bond"`，bond id 为节点名，心跳超时 4.0 秒）与 `void destroyBond()`；`bond_heartbeat_period <= 0.0` 时两者都是空操作。

5 个转换回调继承自 `rclcpp_lifecycle::LifecycleNode`，Nav2 各服务器的统一写法（签名逐字核实自 `nav2_velocity_smoother/include/nav2_velocity_smoother/velocity_smoother.hpp`）：

| 回调 | 触发转换 |
|---|---|
| `nav2::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;` | `configure`：分配资源、读参数、建通信对象 |
| `nav2::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;` | `activate`：激活发布器、启动定时器、**最后 createBond()** |
| `nav2::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;` | `deactivate`：停止处理、**destroyBond()** |
| `nav2::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;` | `cleanup`：释放全部资源，回到 unconfigured |
| `nav2::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;` | `shutdown`：进程退出前收尾 |

## 3. 完整代码

工程布局（独立 ament 包 `my_nav2_examples_lifecycle`）：

```
my_nav2_examples_lifecycle/
├── include/my_nav2_examples_lifecycle/example_managed_node.hpp
├── src/example_managed_node.cpp
├── src/main.cpp
├── CMakeLists.txt
└── package.xml
```

### 3.1 example_managed_node.hpp

```cpp
#ifndef MY_NAV2_EXAMPLES_LIFECYCLE__EXAMPLE_MANAGED_NODE_HPP_
#define MY_NAV2_EXAMPLES_LIFECYCLE__EXAMPLE_MANAGED_NODE_HPP_

#include <string>

#include "nav2_ros_common/lifecycle_node.hpp"   // nav2::LifecycleNode（含 bond）
#include "std_msgs/msg/string.hpp"

namespace my_nav2_examples
{

/**
 * @class ExampleManagedNode
 * @brief 教学用被管生命周期节点：周期发布心跳字符串。
 *        重点演示 5 个回调的职责划分与 bond 的建立/销毁位置。
 */
class ExampleManagedNode : public nav2::LifecycleNode
{
public:
  explicit ExampleManagedNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ExampleManagedNode() override = default;

protected:
  nav2::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void timerCallback();

  // nav2::Publisher 是受生命周期管理的发布器包装，
  // create_publisher 工厂会把它注册为 managed entity（激活态自动 on_activate）
  nav2::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string message_{"heartbeat"};
  double publish_period_s_{1.0};
  int count_{0};
};

}  // namespace my_nav2_examples

#endif  // MY_NAV2_EXAMPLES_LIFECYCLE__EXAMPLE_MANAGED_NODE_HPP_
```

### 3.2 example_managed_node.cpp

```cpp
#include <chrono>
#include <memory>
#include <string>

#include "my_nav2_examples_lifecycle/example_managed_node.hpp"

namespace my_nav2_examples
{

ExampleManagedNode::ExampleManagedNode(const rclcpp::NodeOptions & options)
: nav2::LifecycleNode("example_managed_node", "", options)
{
  // 构造函数里不做重活——此时节点处于 unconfigured，
  // 基类已自动声明 bond_heartbeat_period / autostart_node 参数
  RCLCPP_INFO(get_logger(), "已创建（unconfigured）");
}

nav2::CallbackReturn
ExampleManagedNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "on_configure：读参数、建发布器");

  message_ = declare_or_get_parameter("message", std::string{"heartbeat"});
  publish_period_s_ = declare_or_get_parameter("publish_period_s", 1.0);
  if (publish_period_s_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "publish_period_s 必须为正，配置失败");
    return nav2::CallbackReturn::FAILURE;   // 返回 FAILURE 会退回 unconfigured
  }

  // Nav2 工厂：默认 QoS 为 nav2::qos::StandardTopicQoS，返回受管发布器
  pub_ = create_publisher<std_msgs::msg::String>("example_heartbeat");

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
ExampleManagedNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "on_activate：启动定时器并建立 bond");

  // 定时器在激活时才创建：inactive 状态下不做任何周期工作
  timer_ = create_timer(
    std::chrono::duration<double>(publish_period_s_),
    std::bind(&ExampleManagedNode::timerCallback, this));

  // 关键：激活的最后一步与 lifecycle_manager 建立 bond。
  // manager 由此确认节点存活；节点崩溃则 manager 触发系统降级/重启
  createBond();
  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
ExampleManagedNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "on_deactivate：停定时器并断开 bond");
  timer_.reset();
  destroyBond();   // 与 createBond 严格成对
  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
ExampleManagedNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "on_cleanup：释放资源");
  pub_.reset();
  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
ExampleManagedNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "on_shutdown");
  return nav2::CallbackReturn::SUCCESS;
}

void ExampleManagedNode::timerCallback()
{
  std_msgs::msg::String msg;
  msg.data = message_ + " #" + std::to_string(++count_);
  pub_->publish(msg);
}

}  // namespace my_nav2_examples
```

### 3.3 main.cpp

```cpp
#include <memory>
#include "my_nav2_examples_lifecycle/example_managed_node.hpp"
#include "rclcpp/rclcpp.hpp"

// 写法对照官方 nav2_velocity_smoother/src/main.cpp：
// 生命周期节点必须 spin 其 node_base_interface
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<my_nav2_examples::ExampleManagedNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
```

## 4. 工程关键段（无需 plugins.xml：这是可执行节点，不是插件）

### 4.1 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.20)
project(my_nav2_examples_lifecycle)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

nav2_package()

add_executable(example_managed_node
  src/example_managed_node.cpp
  src/main.cpp)
target_include_directories(example_managed_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(example_managed_node
  nav2_ros_common::nav2_ros_common
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  std_msgs::std_msgs)

install(TARGETS example_managed_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_package()
```

### 4.2 package.xml 关键段

```xml
<name>my_nav2_examples_lifecycle</name>
<buildtool_depend>ament_cmake</buildtool_depend>
<build_depend>nav2_common</build_depend>
<depend>nav2_ros_common</depend>
<depend>rclcpp</depend>
<depend>rclcpp_lifecycle</depend>
<depend>std_msgs</depend>
<export><build_type>ament_cmake</build_type></export>
```

## 5. YAML 挂载配置（lifecycle_manager 接管）

```yaml
example_managed_node:
  ros__parameters:
    message: "hello nav2"
    publish_period_s: 0.5
    bond_heartbeat_period: 0.25   # 基类参数；0.0 = 不建 bond（脱管调试用）

lifecycle_manager_example:
  ros__parameters:
    autostart: true                        # manager 启动后自动 configure+activate 全部节点
    node_names: ["example_managed_node"]   # 按序管理；顺序即启动序
    bond_timeout: 4.0                      # 心跳超时（秒）；超时判定节点失联
```

launch 写法对照官方 `nav2_bringup/launch/navigation_launch.py`（锚点 `nav2_lifecycle_manager`、`node_names`）：

```python
Node(
    package='my_nav2_examples_lifecycle',
    executable='example_managed_node',
    parameters=[params_file],
),
Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_example',
    parameters=[{'autostart': True,
                 'node_names': ['example_managed_node'],
                 'bond_timeout': 4.0}],
),
```

## 6. 运行验证

```bash
# 1. 编译
colcon build --packages-select my_nav2_examples_lifecycle --symlink-install
source install/setup.bash

# 2. 终端 A：起被管节点（此时停在 unconfigured，只打印"等待外部转换"提示）
ros2 run my_nav2_examples_lifecycle example_managed_node --ros-args --params-file my_params.yaml

# 3. 终端 B：起 manager，自动 configure -> activate
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p autostart:=true -p "node_names:=[example_managed_node]" -p bond_timeout:=4.0

# 4. 验证状态与产出
ros2 lifecycle get /example_managed_node        # 应为 active [3]
ros2 topic echo /example_heartbeat              # 应周期收到 "hello nav2 #N"
ros2 topic list | grep bond                     # 应看到 /bond 心跳话题

# 5. 验证 bond 看护：kill 掉被管节点，manager 日志应报
#    "Have not received a heartbeat from example_managed_node" 并触发系统降级
```

## 7. 常见错误排查

1. manager 卡在 `Waiting for service .../change_state`——被管节点没起来、节点名与 `node_names` 不一致（注意不带命名空间斜杠差异），或被管节点不是生命周期节点。
2. 激活成功但 manager 随即报心跳丢失并 reset 系统——忘了在 `on_activate` 里 `createBond()`；或把 `bond_heartbeat_period` 设成了 `0.0`（节点侧不建 bond）却让 manager 开着 bond 检测。
3. 收不到 `/example_heartbeat`——发布器处于未激活态。用本文的 `create_publisher` 工厂则激活由基类托管；若换成裸 `rclcpp_lifecycle` 发布器，需自行在 `on_activate` 调用 `pub_->on_activate()`。
4. `spin` 用了 `rclcpp::spin(node)` 编译报错/行为异常——生命周期节点要 `rclcpp::spin(node->get_node_base_interface())`（对照官方 `nav2_velocity_smoother/src/main.cpp`）。
5. Ctrl+C 时报参数/服务相关异常——不要在析构里手动调 `deactivate/cleanup`：基类的 `on_rcl_preshutdown` → `runCleanups` 已按当前状态自动补齐 deactivate/cleanup（现场核实于 lifecycle_node.hpp，锚点 `runCleanups`）。

## 8. 本示例对应源码（映射链）

| 环节 | 位置 |
|---|---|
| 示例类 | `my_nav2_examples::ExampleManagedNode`（本文） |
| 对应官方实现 | `nav2_velocity_smoother/src/velocity_smoother.cpp`（约 640 行，锚点 `VelocitySmoother::on_configure`、`on_activate` 末尾的 `createBond()`）——官方最精简的"单一职责被管节点" |
| 接口/基类头文件 | `nav2_ros_common/include/nav2_ros_common/lifecycle_node.hpp`（约 476 行，锚点 `createBond`、`autostart`、`runCleanups`）；管理端 `nav2_lifecycle_manager/src/lifecycle_manager.cpp`（约 630 行，锚点 `startup`、bond 检测） |
| 注册处 | 非 pluginlib 插件，无导出宏；"注册"即把节点名写进 lifecycle_manager 的 `node_names` 参数 |
| 官方 YAML/launch 例子 | `nav2_bringup/launch/navigation_launch.py`（锚点 `package='nav2_lifecycle_manager'`、`{'autostart': autostart}, {'node_names': lifecycle_nodes}`） |

闭环路线：读 lifecycle_node.hpp 弄清基类替你做了什么（bond 参数、autostart、preshutdown 清理）→ 对照 VelocitySmoother 看真实服务器的 5 回调职责划分 → 对照 lifecycle_manager.cpp 看管理端如何按 `node_names` 顺序驱动转换并用 bond 看护 → 用本示例骨架把你自己的传感器/算法节点纳入 Nav2 启动体系。
