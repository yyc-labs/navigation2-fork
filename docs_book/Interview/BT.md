# 面试题集：行为树（BehaviorTree.CPP / BT vs FSM / 恢复行为编排）

> 本题集基于 Nav2 1.5.0（main 分支）源码现场核实。
> 每题只给要点提示，不给背诵式完整答案；深入学习请跳转参考章节。

## 初级

1. **行为树节点分哪四大类？tick 返回的三种状态各是什么含义？**
   - 要点提示：Action / Condition / Control / Decorator；SUCCESS / FAILURE / RUNNING；RUNNING 是承载异步长任务的关键
   - 参考章节：[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)

2. **Nav2 默认 NavigateToPose 树的三层骨架是什么？**
   - 要点提示：根 RecoveryNode(number_of_retries=6) → PipelineSequence（规划支 RateController 1 Hz + 控制支 FollowPath）→ 系统恢复支 RoundRobin（清双图 / Spin / Wait / BackUp）
   - 参考章节：[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)、[12 Behaviors恢复行为](../12_Behaviors恢复行为.md)；源码 `nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`

3. **黑板（blackboard）在 Nav2 中承载哪几类东西？**
   - 要点提示：ROS 句柄（node）；超时配置（server_timeout / bt_loop_duration 等）；业务数据流（goal、path、error_code）经 {} 端口重映射传递
   - 参考章节：[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)

4. **如何替换或按次指定导航行为树？**
   - 要点提示：default_nav_to_pose_bt_xml 参数改默认树；NavigateToPose Goal 的 behavior_tree 字段按次指定；nav2_tree_nodes.xml 是节点端口的权威清单
   - 参考章节：[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)、[24 扩展开发指南](../24_扩展开发指南.md)

## 中级

1. **RecoveryNode 的精确语义是什么？默认树里两处 retries 取值为什么不同？**
   - 要点提示：恰好两个子节点（主任务 + 恢复动作）；主失败→tick 恢复→恢复成功后重试主，至多 number_of_retries 轮；上下文恢复 retries=1 快速试错，根节点 retries=6 给系统恢复留预算
   - 参考章节：[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)、[12 Behaviors恢复行为](../12_Behaviors恢复行为.md)

2. **Fallback 与 ReactiveFallback 的差别？默认树用 ReactiveFallback + GoalUpdated 实现了什么能力？**
   - 要点提示：Reactive 变体每次 tick 都重评估前面的子节点；新目标到达可立刻打断正在执行的恢复动作，避免对旧目标白白恢复
   - 参考章节：[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)

3. **PlannerSelector / ControllerSelector 这类 Selector 节点的机制是什么？解决什么问题？**
   - 要点提示：订阅专用话题，运行时改写算法 id 输出端口；不改 XML、不重启即可切换算法插件；默认树里规划/控制/目标检查/进度检查四个 Selector 全套在位
   - 参考章节：[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)、[09 ControllerServer与局部控制](../09_ControllerServer与局部控制.md)

4. **BT 节点从哪里获得 ROS 通信能力？wait_for_service_timeout 管什么？**
   - 要点提示：从黑板取 client_node_ 句柄；BtActionNode 内建 Action 客户端；等待服务器上线的发现超时默认 1000 ms，与运行期响应超时（server_timeout）不同
   - 参考章节：[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)、[19 Action与通信机制](../19_Action与通信机制.md)

5. **bt_navigator 的 navigator 插件层是什么？NavigateToPose 与 NavigateThroughPoses 是什么关系？**
   - 要点提示：两个 navigator 插件共存（navigator_plugins.xml），各持一棵默认树与一个 Action 入口；共享同一套 BT 引擎与黑板设施；导航器负责回调注入与反馈组装
   - 参考章节：[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)；源码 `nav2_bt_navigator/src/navigators/`

## 高级

1. **bt_loop_duration、default_server_timeout、default_cancel_timeout 的默认值与语义分别是什么？误配各有什么症状？**
   - 要点提示：10 ms 全树 tick 节拍 / 20 ms 等 Action 服务器响应 / 50 ms 等取消完成；节拍过大反馈迟钝、响应超时过小误判失败、取消超时过小残留悬挂 goal
   - 参考章节：[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)；源码 `nav2_behavior_tree/include/nav2_behavior_tree/bt_action_server_impl.hpp`（约 156~167 行）

2. **树正在执行时新 goal 到达（抢占），bt_action_server 如何处理？对自定义节点提出什么要求？**
   - 要点提示：不销毁不重建树，黑板热更新目标；GoalUpdated 条件节点感知变化；自定义节点若缓存了旧目标派生状态，须在 tick 中自行检测并刷新
   - 参考章节：[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)

3. **always_reload_bt_xml 参数什么时候开？背后的取舍是什么？**
   - 要点提示：每次请求强制重新加载 XML；开发期改树即生效的迭代便利 vs 生产期加载解析开销与缓存树复用
   - 参考章节：[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)；源码 `nav2_behavior_tree/include/nav2_behavior_tree/bt_action_server_impl.hpp`（约 169 行）

4. **BehaviorTree.CPP v4 的端口系统如何实现类型安全？自定义类型怎么接入 XML？**
   - 要点提示：providedPorts 静态声明 InputPort/OutputPort；getInput\<T\> 运行时取值与类型转换；为自定义类型特化 convertFromString；根标签 BTCPP_format="4"
   - 参考章节：[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)、[23 现代C++专题](../23_现代C++专题.md)

## 架构设计

1. **为什么 Nav2 选择"BT 只做编排 + Action 服务器做执行"，而不是把算法做成进程内的 BT 叶子直接调用？**
   - 要点提示：故障隔离与生命周期独立；算法可整体替换、跨进程部署；树保持轻量高频 tick；代价是通信延迟与接口维护
   - 参考章节：[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)、[01 整体架构](../01_整体架构.md)、[19 Action与通信机制](../19_Action与通信机制.md)

2. **错误码如何让恢复"更聪明"而不是盲目轮换？在树里如何落地？**
   - 要点提示：error_code_id 端口把失败原因写入黑板；按错误类型选择恢复策略（无路→清图换算法，TF 错→等待重定位）；条件节点研判后再进恢复分支
   - 参考章节：[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)、[07 PlannerServer与全局规划](../07_PlannerServer与全局规划.md)、[09 ControllerServer与局部控制](../09_ControllerServer与局部控制.md)

3. **恢复动作集中托管在 behavior_server，对比"每个 BT 节点自带恢复逻辑"，优劣如何？**
   - 要点提示：碰撞检查等安全设施只建一份；恢复动作是带反馈可取消的长任务，天然适合 Action；树中只留编排语义；代价是多一跳通信
   - 参考章节：[12 Behaviors恢复行为](../12_Behaviors恢复行为.md)、[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)

4. **要支持"乘电梯跨楼层导航"，从树结构到新构件的设计路径是什么？**
   - 要点提示：新 Action 服务器（呼梯/门控）；BtActionNode 派生客户端节点并注册 plugin_lib_names；SubTree 封装乘梯流程复用；楼层切换伴随地图/定位重置的编排
   - 参考章节：[24 扩展开发指南](../24_扩展开发指南.md)、[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)

## 源码分析

1. **bt_action_server_impl.hpp 的配置阶段（约 155~191 行）把哪些东西放进了黑板？为什么 node 也要进黑板？**
   - 要点提示：四个超时参数 + client_node_ 句柄逐一写入；BT 节点无法通过构造函数注入依赖，黑板是它们获取 ROS 世界入口的唯一通道
   - 参考章节：[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)；源码 `nav2_behavior_tree/include/nav2_behavior_tree/bt_action_server_impl.hpp`

2. **behavior_tree_engine 的 run 循环如何把 BT 的同步 tick 与 rclcpp 世界缝合起来？**
   - 要点提示：按 bt_loop_duration 节拍 tick 根节点；onLoop 回调每圈发布反馈；注入 cancel 检查函数在圈间响应取消
   - 参考章节：[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)、[20 线程模型](../20_线程模型.md)；源码 `nav2_behavior_tree/src/behavior_tree_engine.cpp` 的 `run()`

3. **round_robin_node.cpp（全文约 108 行）如何维护轮换状态？它和 Fallback 的本质差别是什么？**
   - 要点提示：记录当前子节点索引，成功后推进并环绕；全部失败才返回 FAILURE；Fallback 每次都从第一个孩子开始，RoundRobin"记住上次位置"避免重复同一恢复动作
   - 参考章节：[12 Behaviors恢复行为](../12_Behaviors恢复行为.md)、[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)；源码 `nav2_behavior_tree/plugins/control/round_robin_node.cpp`

4. **逐节点解读默认树的规划支：RateController 内嵌的 Fallback + ReactiveSequence 在做什么？**
   - 要点提示：先查 GoalUpdated 与旧路径有效性决定是否跳过重算（省算力）；需要重算才走 ComputePathToPose；上下文 RecoveryNode 失败时清全局图重试一次
   - 参考章节：[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)、[07 PlannerServer与全局规划](../07_PlannerServer与全局规划.md)；源码 `nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`（约 16~34 行）

## 开放题

1. **BT 适不适合做多机器人协同的任务分配层？它与专用任务规划器的边界在哪里？**
   - 要点提示：BT 强在单体反应式编排，弱在全局优化与资源分配；讨论"上层分配器 + 单机 BT"分层；黑板跨机共享的困难
   - 参考章节：[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)、[25 学习路线图](../25_学习路线图.md)

2. **用 LLM 生成或在线修改行为树 XML，可行性与安全约束怎么设计？**
   - 要点提示：XML 是良好的生成目标（结构化、可校验）；节点白名单与端口 schema 校验；仿真验证门槛与灰度上线；失败语义不可由生成器随意改写
   - 参考章节：[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)、[24 扩展开发指南](../24_扩展开发指南.md)

3. **一棵生产环境的导航树如何做回归测试？**
   - 要点提示：节点级单测（mock 黑板与 Action 服务器）；树结构断言防误改；仿真场景库回放关键失败路径；恢复分支的故障注入覆盖
   - 参考章节：[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)、[24 扩展开发指南](../24_扩展开发指南.md)
