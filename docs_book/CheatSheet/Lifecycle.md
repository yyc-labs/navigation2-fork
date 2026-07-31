# 速查卡：生命周期（Lifecycle）

> 口径：Nav2 1.5.0（main 分支）。详解见 [第 05 章](../05_生命周期管理.md)。

## 什么时候打开本速查

- 写 `on_configure` / `on_activate` 时想不起回调顺序与各状态该干什么活；
- 节点报 "bond timeout" 或整个导航栈突然被放倒，需要回忆 bond 判死与恢复逻辑；
- 想手动 `ros2 service call` 拉起/暂停/重置导航栈，忘了 `manage_nodes` 的命令编号；
- 配置 `lifecycle_manager` 的 YAML，想不起 `bond_timeout` 等参数名与默认值；
- 自定义节点要接入 lifecycle_manager 管理，不确定要继承什么、bond 何时建立。

## 1. 状态机与转换回调

四主状态 + 终态，转换回调返回 `SUCCESS` / `FAILURE` / `ERROR`，FAILURE 则状态回滚：

| 转换 | 回调 | 状态迁移 | 该做什么 |
|---|---|---|---|
| configure | `on_configure` | Unconfigured → Inactive | 声明参数、分配内存、创建发布者/订阅者/Action 服务器（未激活） |
| activate | `on_activate` | Inactive → Active | 激活发布者、启动线程、**建立 bond**（`createBond()`） |
| deactivate | `on_deactivate` | Active → Inactive | 停线程、停对外输出、**销毁 bond**（`destroyBond()`） |
| cleanup | `on_cleanup` | Inactive → Unconfigured | 彻底释放资源、断开所有引用 |
| shutdown | `on_shutdown` | 任意 → Finalized | 终结，只待销毁 |

记忆锚点：**Inactive 是"就绪但不工作"**；`cleanup → configure` 循环 = 不杀进程重置节点。详见 [第 05 章](../05_生命周期管理.md)。

Nav2 服务器统一继承封装层 `nav2::LifecycleNode`（`nav2_ros_common/include/nav2_ros_common/lifecycle_node.hpp`，`createBond()` 函数约 366 行处）：附带 bond 心跳、rcl 预关停钩子（Ctrl+C 自动补 deactivate/cleanup）、`autostart_node` 自启、`declare_or_get_parameter()` 等能力。插件的 configure/activate/deactivate/cleanup 由宿主服务器在对应 `on_*` 回调中逐个转发（见 [第 04 章](../04_nav2_core插件接口.md)）。

## 2. lifecycle_manager 七个命令

服务：`<manager_name>/manage_nodes`，类型 `nav2_msgs/srv/ManageLifecycleNodes`（`nav2_msgs/srv/ManageLifecycleNodes.srv`）；另有 `<manager_name>/is_active`（`std_srvs/srv/Trigger`）。

| 命令 | 编号 | 动作组合 | 遍历方向 |
|---|---|---|---|
| STARTUP | 0 | configure 全体 → activate 全体 → 启动 bond 巡检 | 正序 |
| PAUSE | 1 | deactivate 全体 | 逆序 |
| RESUME | 2 | activate 全体 | 正序 |
| RESET | 3 | deactivate → cleanup | 逆序 |
| SHUTDOWN | 4 | deactivate → cleanup → shutdown | 逆序 |
| CONFIGURE | 5 | 仅 configure | 正序 |
| CLEANUP | 6 | 仅 cleanup | 逆序 |

方向规则：**向上（configure/activate）按 `node_names` 正序；向下（deactivate/cleanup/shutdown）逆序**。STARTUP 是"两轮完整遍历"：全体先 configure 再统一 activate，避免半激活撕裂态。详见 [第 05 章](../05_生命周期管理.md)。

常用命令行：

```bash
# 拉起整个导航栈（STARTUP = 0）
ros2 service call /lifecycle_manager_navigation/manage_nodes \
  nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"

# 单节点手动转换（调试用）
ros2 lifecycle set /planner_server configure
ros2 lifecycle get /planner_server
```

## 3. lifecycle_manager 关键参数

源码：`nav2_lifecycle_manager/src/lifecycle_manager.cpp`（构造函数参数声明段，约 43 行起）。

| 参数 | 默认值 | 作用 |
|---|---|---|
| `node_names` | 必填 | 受管节点列表，**顺序即启动顺序** |
| `autostart` | false | true 时管理器初始化完毕立即 STARTUP |
| `bond_timeout` | 4.0 s | 心跳超时判死阈值；设 0 关闭 bond 监测 |
| `bond_heartbeat_period` | 0.25 s | 心跳周期（节点侧同名参数须一致） |
| `bond_respawn_max_duration` | 10.0 s | 故障后等待节点重生的最大时长 |
| `attempt_respawn_reconnection` | true | 故障后是否尝试自动重连重启 |
| `service_timeout` | 5.0 s | 调用节点生命周期服务的超时 |

## 4. bond 心跳速记

- 节点侧：`nav2::LifecycleNode::createBond()` 在 activate 后建立，话题名 `bond`，心跳周期默认 0.25 s；节点侧**禁用超时**（不会因管理器失联自杀），超时判定只在管理器侧。
- 管理器侧：启动完成后以 200 ms 周期执行 `checkBondConnections()`。
- 故障语义：**一个倒下全体放倒 + 整体重启**——任一受管节点 bond 断开，管理器判定系统不完整，逆序放倒全体；在 `bond_respawn_max_duration` 窗口内检测到节点重生则整体重新 STARTUP。
- 详解与"熔断—观察—恢复"流程图见 [第 05 章](../05_生命周期管理.md)；线程与定时器视角见 [第 20 章](../20_线程模型.md)。

## 5. 自定义节点接入清单

1. 继承 `nav2::LifecycleNode`（而非裸 `rclcpp::Node`）；
2. 实现 `on_configure` / `on_activate` / `on_deactivate` / `on_cleanup` / `on_shutdown`；
3. 把节点名加入 lifecycle_manager 的 `node_names`（注意顺序：被依赖者在前）；
4. 心跳参数 `bond_heartbeat_period` 与管理器保持一致（或两侧同时设 0 关闭）。

扩展实战见 [第 24 章](../24_扩展开发指南.md)。
