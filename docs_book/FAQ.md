# FAQ：高频"为什么"速答

> 口径：Nav2 1.5.0（main 分支）、C++20。每条 3~5 行给出**一口气能说完的答案**，展开论证请跟随链接回原章及其"X.B 设计权衡"板块——本页只给结论与直觉，不重复推演。返回导航枢纽：[Index.md](Index.md)。

## 通信与任务模型

**1. 为什么导航任务用 Action 而不用 Service 或 Topic？**
导航是分钟级长任务，需要三件 Service/Topic 给不了的东西：执行中的周期反馈、随时取消、新目标抢占。Service 会把调用方吊死在同步等待上，Topic 则需要自己发明"完成确认协议"。选型口诀：要等结果吗？要等多久？中途要管吗？——详见 [19_Action与通信机制.md](19_Action与通信机制.md)（19.1 节与 19.B）。

**2. 为什么自研 SimpleActionServer 而不裸用 rclcpp_action？**
rclcpp_action 允许多 goal 并发并把线程调度交给使用者，但机器人只有一个身体，并发 goal 是错误而非特性。Nav2 在框架层一次性固化"单目标 + pending 抢占"模型，九个服务器免写重复八股；代价是抢占是协作式的，插件必须自觉轮询。1.5.0 中它位于 nav2_ros_common 包、nav2:: 命名空间。详见 [19_Action与通信机制.md](19_Action与通信机制.md)（19.3 节与 19.B）。

**3. 为什么 nav2_msgs 定义近 20 个细粒度 Action，而不是一个"NavigateAndDoEverything"？**
按服务器职责切而非按业务场景切：每个服务器只暴露原子能力，组合逻辑上移到行为树 XML。业务变化只改 XML，不动接口包与服务器；代价是排障要分层看各级 Action 的 feedback 与 error_code。详见 [19_Action与通信机制.md](19_Action与通信机制.md)（19.B）。

## 生命周期与系统管理

**4. Lifecycle 为什么还要加 bond？状态机不是已经知道节点状态了吗？**
生命周期状态机只管"受控转换"，管不了"意外死亡"——节点崩溃时不会礼貌地走 deactivate。bond 心跳一断，lifecycle_manager 立即放倒整组节点（一个倒下全体放倒），配合 launch 的 respawn 完成整体重启，避免"半死系统"发出危险指令。详见 [05_生命周期管理.md](05_生命周期管理.md)（5.4 节与 5.B）。

**5. 为什么"一个倒下全体放倒"，而不是只重启挂掉的那个？**
导航服务器间存在隐式共享状态（TF 缓存、costmap、正在执行的 goal），单独复活一个节点极易带着陈旧状态与其他节点错位。整组放倒重启把系统拉回已知初始态，牺牲可用性换确定性——对带轮子的机器人，确定性优先。详见 [05_生命周期管理.md](05_生命周期管理.md)（5.B）。

**6. 为什么拆成十来个服务器进程，而不是一个大节点（像 move_base）？**
故障隔离（规划器崩溃不拖死控制循环）、独立频率（20Hz 控制与 1Hz 重规划互不干扰）、按需替换与裁剪。代价是进程间通信与启动编排复杂度，由 lifecycle_manager 与 Composition 部署兜底。详见 [01_整体架构.md](01_整体架构.md)（1.1 节）与 [Evolution/BehaviorTree.md](Evolution/BehaviorTree.md) 的历史脉络。

## 行为树与编排

**7. BT 为什么要一直 Tick，而不是事件驱动等回调？**
持续 tick 让"检查取消、检查抢占、监控子任务"变成每个周期的自然副产品，无需为每种异步事件注册回调再处理竞态；树的当前状态永远是"这一轮 tick 的真值"，可观测且可中断。代价是空转开销，用 RateController 等装饰节点限频兜住。详见 [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md)（11.3 节与 11.B）。

**8. 为什么用行为树取代状态机（FSM）？**
FSM 的跳转边数随状态数平方增长，move_base 时代加一个恢复策略要横切改多处；BT 把恢复与重试收敛为局部子树，复杂度线性增长，且逻辑外置为 XML、改编排不发版。状态机没有被消灭，而是下沉到局部（生命周期、靠泊阶段）。详见 [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md)（11.1 节与 11.B）与 [Evolution/BehaviorTree.md](Evolution/BehaviorTree.md)。

**9. 为什么导航逻辑写进 XML 而不是 C++ 代码？**
XML 是配置不是代码：现场工程师改编排（换恢复顺序、加条件分支）无需编译环境与发版流程；同一套二进制可承载无数业务变体。代价是 XML 无类型检查、调试靠 Groot 与日志。详见 [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md)（11.B）与 [21_配置系统.md](21_配置系统.md)（21.3 节）。

## 插件与扩展

**10. Planner/Controller 为什么做成插件，而不是直接写死几个算法？**
算法迭代速度远快于框架：插件让学术算法与工程框架各自演进，换算法只改一行 YAML；宿主服务器只依赖 nav2_core 纯虚接口，编译期互不感知。这是开闭原则的教科书落地。详见 [04_nav2_core插件接口.md](04_nav2_core插件接口.md)（4.B）与 [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md)（7.B）。

**11. 为什么插件的依赖（节点、TF、costmap）全部经 configure 注入，而不是自己去获取？**
依赖注入使插件可以在测试中被 mock、耦合方向可控（插件依赖抽象而非全局状态），也是 Nav2 拒绝单例的直接原因——单例会让"谁在什么线程动了什么"不可审计。详见 [04_nav2_core插件接口.md](04_nav2_core插件接口.md)（4.4 节）与 [22_设计模式专题.md](22_设计模式专题.md)（22.14、22.15 节）。

## Costmap 与感知

**12. Costmap 为什么分层，而不是一张图直接写？**
每层职责单一（静态图/传感障碍/膨胀/过滤），可独立开关、独立调参、独立扩展；合成顺序显式可控，新增语义（禁区、限速）不侵入已有层。代价是两阶段更新协议（updateBounds/updateCosts）对层作者有学习成本。详见 [06_Costmap2D.md](06_Costmap2D.md)（6.B）。

**13. 为什么更新分 updateBounds 与 updateCosts 两阶段？**
先让各层申报"这一轮我弄脏的包围盒"，合并后只对脏区做代价写入——把每周期全图重算变成增量更新，这是 costmap 能高频跑的关键。详见 [06_Costmap2D.md](06_Costmap2D.md) 与 [Assets/sequence/Costmap更新循环.md](Assets/sequence/Costmap更新循环.md)。

**14. 为什么 global 与 local 两张 costmap，不共用一张？**
两者需求相反：全局图要大而全（静态图 + 低频即可），局部图要小而快（滚动窗口 + 高频传感更新）。合用一张要么拖慢控制循环，要么牺牲全局视野。详见 [06_Costmap2D.md](06_Costmap2D.md)（6.B）与 [20_线程模型.md](20_线程模型.md)。

## 速度链路与安全

**15. cmd_vel 为什么走三级链路（cmd_vel_nav → velocity_smoother → collision_monitor → cmd_vel），不让控制器直发底盘？**
把"算得聪明"与"停得可靠"分开：控制器专注跟路径，velocity_smoother 保证加减速物理可执行，collision_monitor 用最简单的几何规则直连传感器做最终裁决——即使上游全部失效仍能兜底。安全不依赖"聪明"，依赖"简单"。详见 [16_CollisionMonitor与VelocitySmoother.md](16_CollisionMonitor与VelocitySmoother.md)（16.B）。

**16. CollisionMonitor 为什么直连原始传感器，不用现成的 costmap？**
costmap 经过 TF、缓冲、分层合成多级加工，任何一级故障或延迟都会污染安全判断；安全层必须最短路径拿到最原始的数据，且逻辑简单到可以被穷尽验证。详见 [16_CollisionMonitor与VelocitySmoother.md](16_CollisionMonitor与VelocitySmoother.md)（16.3 节与 16.B）。

**17. 为什么 behavior_server（恢复行为）与控制器并联输出 cmd_vel_nav，而不是串在控制器后面？**
恢复行为（Spin/BackUp）执行时控制器已经失败退出，两者天然互斥、不会同时发速度；并联到同一话题让下游安全链对"谁在开车"保持无感知，任何速度来源都被一视同仁地平滑与安检。详见 [12_Behaviors恢复行为.md](12_Behaviors恢复行为.md)（12.B）与 [16_CollisionMonitor与VelocitySmoother.md](16_CollisionMonitor与VelocitySmoother.md)。

## 算法选型

**18. 全局规划有了 NavFn，为什么还要 SmacPlanner？**
NavFn 假设机器人是全向质点，路径不保证车辆可跟踪；Smac Hybrid-A* 在 SE(2) 空间用运动基元扩展，输出天然满足最小转弯半径，倒车、泊车、类车底盘必须用它。详见 [08_SmacPlanner.md](08_SmacPlanner.md)（8.1 节与 8.B）与 [Evolution/Planner.md](Evolution/Planner.md)。

**19. MPPI 为什么用上千条随机轨迹采样，而不是解析求解最优控制？**
代价函数含 costmap 查询等不可导项，解析/梯度方法无从下手；采样 + 加权平均对任意代价函数都成立，且天然并行（SIMD/向量化）。代价是算力开销与轻微抖动，用噪声预生成线程与软约束缓解。详见 [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md)（10.3 节与 10.B）与 [Evolution/Controller.md](Evolution/Controller.md)。

**20. 已有自由空间规划器，为什么还要 nav2_route 图路由？**
大型场景（仓库、园区）需要"只走审批过的通道"这类业务约束，自由空间搜索既慢又不可控；预定义路网把搜索空间从百万栅格降到千级图节点，且路线语义（限速、单行）可挂在边上。两者互补：Route 出骨干，Planner 补首尾自由段。详见 [17_Route路线规划.md](17_Route路线规划.md)（17.1 节与 17.B）。

**21. AMCL 为什么发布 map→odom 变换，而不直接发布机器人位姿？**
里程计连续平滑但漂移，AMCL 修正准确但低频跳变；把修正量放在 map→odom 一级，控制循环消费连续的 odom→base_link，全局一致性由 TF 链乘法自动合成——两种传感器特性各取所长。详见 [13_AMCL定位.md](13_AMCL定位.md)（13.B）。

**22. 为什么很多参数不支持热更新，改了还得重启？**
线程、订阅、内存布局类参数（如 costmap 尺寸、控制频率）在 on_configure 阶段固化，运行中改动等于重建对象，热更新收益低风险高；Nav2 只对"数值旋钮"类参数开放动态更新。详见 [21_配置系统.md](21_配置系统.md)（21.2 节与 21.B）。

---

> 更多"为什么"散见各章 **X.B 设计权衡** 板块（04~21、24 章均有）；面试角度的追问与答案见 [Interview/](Interview/ROS2基础.md) 系列。
