# 面试题集：局部控制（DWB / RPP / MPPI / 软实时循环）

> 本题集基于 Nav2 1.5.0（main 分支）源码现场核实。
> 每题只给要点提示，不给背诵式完整答案；深入学习请跳转参考章节。

## 初级

1. **Controller 插件契约的核心方法有哪些？computeVelocityCommands 的三个入参各是什么？**
   - 要点提示：setPlan / computeVelocityCommands / setSpeedLimit + 生命周期回调；入参为当前位姿、当前速度、GoalChecker 指针
   - 参考章节：[04 nav2_core插件接口](../04_nav2_core插件接口.md)、[09 ControllerServer与局部控制](../09_ControllerServer与局部控制.md)

2. **DWB 的一个控制周期内部做哪几步？**
   - 要点提示：速度空间采样 → 轨迹前向模拟 → critics 加权打分 → 取总分最优；非法轨迹被 critic 直接否决
   - 参考章节：[09 ControllerServer与局部控制](../09_ControllerServer与局部控制.md)；源码 `nav2_dwb_controller/dwb_core/`

3. **Pure Pursuit 的几何原理是什么？前视距离扮演什么角色？**
   - 要点提示：追踪路径上前视点的圆弧，曲率 κ = 2y/L²；前视距离决定跟踪松紧与稳定性
   - 参考章节：[10 MPPI与RPP控制器](../10_MPPI与RPP控制器.md)

4. **控制器算出的速度指令到底盘之间还要经过哪些节点？各自职责？**
   - 要点提示：cmd_vel_nav → velocity_smoother（加减速约束）→ cmd_vel_smoothed → collision_monitor（安全裁决）→ cmd_vel；behavior_server 的恢复动作与控制器并联输出 cmd_vel_nav
   - 参考章节：[16 CollisionMonitor与VelocitySmoother](../16_CollisionMonitor与VelocitySmoother.md)、[01 整体架构](../01_整体架构.md)

## 中级

1. **dwb_critics 内置的评分器有哪些代表？各管什么维度？**
   - 要点提示：BaseObstacle/ObstacleFootprint（碰撞与代价）；PathAlign/PathDist 与 GoalAlign/GoalDist（贴路径 vs 奔目标）；RotateToGoal、Oscillation、PreferForward、Twirling（末端与运动风格）
   - 参考章节：[09 ControllerServer与局部控制](../09_ControllerServer与局部控制.md)；源码 `nav2_dwb_controller/dwb_critics/src/`

2. **StandardTrajectoryGenerator 与 LimitedAccelGenerator 的差别是什么？"动态窗口"的本质是什么？**
   - 要点提示：轨迹外推时是否用加速度限制约束起步段速度；动态窗口 = 当前速度出发一个控制周期内的加速度可达集
   - 参考章节：[09 ControllerServer与局部控制](../09_ControllerServer与局部控制.md)；源码 `nav2_dwb_controller/dwb_plugins/src/`

3. **RPP 的三个 regulation 纯函数各自的输入输出是什么？**
   - 要点提示：curvatureConstraint（曲率→线速度上限）、costConstraint（代价地图接近度→减速）、approachVelocityConstraint（距终点距离→减速）；三者取最保守
   - 参考章节：[10 MPPI与RPP控制器](../10_MPPI与RPP控制器.md)；源码 `nav2_regulated_pure_pursuit_controller/include/nav2_regulated_pure_pursuit_controller/regulation_functions.hpp`

4. **MPPI 的 critics 与 DWB 的 critics 有什么本质区别？**
   - 要点提示：DWB 对少量离散候选轨迹打分后"选择"最优；MPPI 对上千条随机轨迹算代价后 softmax 加权"合成"控制序列；选择 vs 合成
   - 参考章节：[10 MPPI与RPP控制器](../10_MPPI与RPP控制器.md)

5. **控制循环每圈开始为什么要 waitUntilCurrent 等代价地图？costmap_update_timeout 默认多少、超时会怎样？**
   - 要点提示：保证控制决策不基于过期障碍信息；默认 0.30 s；超时抛异常映射为错误码进入恢复链路
   - 参考章节：[09 ControllerServer与局部控制](../09_ControllerServer与局部控制.md)、[06 Costmap2D](../06_Costmap2D.md)；源码 `nav2_controller/src/parameter_handler.cpp`（约 47 行）

## 高级

1. **MPPI 的 warm start 机制：上一周期的解如何被下一周期复用？为什么 iteration_count 默认 1 也能收敛？**
   - 要点提示：optimize 前 shiftControlSequence 把控制序列整体前移一拍作为先验；真正的迭代发生在时间维度上（每个控制周期都是一次迭代）
   - 参考章节：[10 MPPI与RPP控制器](../10_MPPI与RPP控制器.md)；源码 `nav2_mppi_controller/src/optimizer.cpp` 的 `shiftControlSequence()`（约 345 行）

2. **applyControlSequenceConstraints 对第一拍与后续拍的加速度约束为什么区别处理？**
   - 要点提示：第 0→1 拍用真实控制周期算 Δv 上限，其后统一用 model_dt（约 434 行注释 intra-iteration feasibility）；保证输出对底盘即时可行
   - 参考章节：[10 MPPI与RPP控制器](../10_MPPI与RPP控制器.md)；源码 `nav2_mppi_controller/src/optimizer.cpp` 的 `applyControlSequenceConstraints()`（约 404 行）

3. **use_realtime_priority / SimpleActionServer 的 realtime 参数在底层做了什么？它能保证截止期吗？**
   - 要点提示：执行线程调用 setSoftRealTimePriority 提升到实时调度类；需要系统 rtprio 权限配合；只压低调度抖动，不构成硬实时保证
   - 参考章节：[20 线程模型](../20_线程模型.md)、[09 ControllerServer与局部控制](../09_ControllerServer与局部控制.md)

4. **Nav2 为什么全栈从 Twist 迁移到 TwistStamped？velocity_smoother 里对时间戳还做了什么特殊处理？**
   - 要点提示：时间戳支撑超时判断与链路延迟测量；TwistPublisher/TwistSubscriber 双协议兼容过渡；平滑输出的时间戳随周期推进而非复用旧戳，避免下游抖动
   - 参考章节：[16 CollisionMonitor与VelocitySmoother](../16_CollisionMonitor与VelocitySmoother.md)；源码 `nav2_velocity_smoother/src/velocity_smoother.cpp`（约 320 行注释）

## 架构设计

1. **速度平滑与碰撞防护为什么不做进控制器内部，而是独立成两个节点？**
   - 要点提示：覆盖所有速度源（恢复行为、遥控、docking）；失效独立性——控制器异常时防线仍在；单一职责与独立调参
   - 参考章节：[16 CollisionMonitor与VelocitySmoother](../16_CollisionMonitor与VelocitySmoother.md)、[01 整体架构](../01_整体架构.md)

2. **controller / goal_checker / progress_checker / path_handler 四类插件正交组合的设计价值是什么？**
   - 要点提示：四个关注点独立替换演进；Goal 里按 id 运行时选择；用组合避免"每种搭配一个子类"的类爆炸
   - 参考章节：[09 ControllerServer与局部控制](../09_ControllerServer与局部控制.md)、[22 设计模式专题](../22_设计模式专题.md)

3. **一次 computeVelocityCommands 失败或超时，系统从内到外有哪几道兜底？**
   - 要点提示：failure_tolerance 零速等待；错误码→行为树恢复分支；collision_monitor 末端硬裁决；底盘侧 cmd_vel 超时自动刹车
   - 参考章节：[09 ControllerServer与局部控制](../09_ControllerServer与局部控制.md)、[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)、[16 CollisionMonitor与VelocitySmoother](../16_CollisionMonitor与VelocitySmoother.md)

4. **要实现"高速走廊与低速人群区自动切换控制策略"，用哪些既有机制可以拼出来？**
   - 要点提示：SpeedFilter → SpeedLimit 话题 → setSpeedLimit 链路；ControllerSelector 话题切换控制器插件；MPPI 动态参数调整
   - 参考章节：[06 Costmap2D](../06_Costmap2D.md)、[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)、[09 ControllerServer与局部控制](../09_ControllerServer与局部控制.md)

## 源码分析

1. **controller_server.cpp 的 `computeControl()`（约 490 行起）主循环每圈做什么？频率不达标时如何暴露？**
   - 要点提示：nav2::Rate 按 controller_frequency（默认 20 Hz）节拍；每圈依次做抢占检查、等代价地图、computeAndPublishVelocity、到达判定；周期超支打印告警日志
   - 参考章节：[09 ControllerServer与局部控制](../09_ControllerServer与局部控制.md)、[20 线程模型](../20_线程模型.md)；源码 `nav2_controller/src/controller_server.cpp`

2. **optimizer.cpp 的 `optimize()`（约 272 行）流水线可以拆成哪几步？底层数值库是什么？**
   - 要点提示：prepare → generateNoisedTrajectories → critics 评分 → softmax 更新控制序列 → 约束裁剪；Eigen ArrayXXf 批量向量化运算
   - 参考章节：[10 MPPI与RPP控制器](../10_MPPI与RPP控制器.md)、[23 现代C++专题](../23_现代C++专题.md)；源码 `nav2_mppi_controller/src/optimizer.cpp`

3. **regulation_functions.hpp 为什么把三个调速逻辑做成 header-only 的 inline 纯函数？**
   - 要点提示：无状态、入参出参明确、便于独立单测；调用点内联无开销；与主控制器类解耦复用
   - 参考章节：[10 MPPI与RPP控制器](../10_MPPI与RPP控制器.md)、[23 现代C++专题](../23_现代C++专题.md)；源码 `nav2_regulated_pure_pursuit_controller/include/nav2_regulated_pure_pursuit_controller/regulation_functions.hpp`

4. **velocity_smoother.cpp 的输入超时保护逻辑是怎样的？为什么超时不能直接停发？**
   - 要点提示：超过 velocity_timeout 未收到新指令则以零速为目标继续按减速度约束滑停；直接停发会让底盘保持上一条速度或急停，二者都危险
   - 参考章节：[16 CollisionMonitor与VelocitySmoother](../16_CollisionMonitor与VelocitySmoother.md)；源码 `nav2_velocity_smoother/src/velocity_smoother.cpp` 的 `smootherTimer` 回调（约 329 行注释）

## 开放题

1. **差速底盘 + 低算力 ARM 板 + 仓储窄巷场景，DWB / RPP / MPPI 怎么选？给出理由与放弃项。**
   - 要点提示：算力预算（MPPI batch×steps 开销）；窄巷对贴线精度的要求；倒车与原地转需求；调参人力成本
   - 参考章节：[10 MPPI与RPP控制器](../10_MPPI与RPP控制器.md)、[09 ControllerServer与局部控制](../09_ControllerServer与局部控制.md)

2. **20 Hz 控制频率的假设在什么场景下不成立？要提到 100 Hz 需要动哪些层？**
   - 要点提示：高速/高动态底盘；代价地图更新频率、MPPI model_dt 与采样量、执行器调度抖动、传感器延迟全链路重估
   - 参考章节：[09 ControllerServer与局部控制](../09_ControllerServer与局部控制.md)、[20 线程模型](../20_线程模型.md)

3. **"礼让行人"这类社会性导航行为，放在 MPPI critic、独立代价层还是行为树层实现？**
   - 要点提示：critic 表达连续偏好但每控制器各写一份；代价层全控制器共享但表达力受栅格限制；BT 层适合离散决策（停让/绕行）；讨论组合方案
   - 参考章节：[10 MPPI与RPP控制器](../10_MPPI与RPP控制器.md)、[06 Costmap2D](../06_Costmap2D.md)、[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)
