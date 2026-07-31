# 面试题集：ROS 2 基础（节点 / 执行器 / QoS / Action / 生命周期 / TF / DDS）

> 本题集基于 Nav2 1.5.0（main 分支）源码现场核实，题目以 Nav2 为语境考察 ROS 2 基础机制。
> 每题只给要点提示，不给背诵式完整答案；深入学习请跳转参考章节。

## 初级

1. **ROS 2 去掉了 ROS 1 的 master，节点之间靠什么互相发现？**
   - 要点提示：DDS 分布式发现（参与者/端点两级）；无单点故障；rmw 抽象层屏蔽厂商差异
   - 参考章节：[01 整体架构](../01_整体架构.md)、[19 Action与通信机制](../19_Action与通信机制.md)

2. **话题、服务、Action 三种通信原语各适合什么场景？在 Nav2 中各举一个实例。**
   - 要点提示：高频单向流（cmd_vel）→ 话题；短原子问答（clear_entirely_global_costmap）→ 服务；长时 + 反馈 + 可取消（navigate_to_pose）→ Action
   - 参考章节：[19 Action与通信机制](../19_Action与通信机制.md)

3. **QoS 的 reliability 与 durability 各有哪些档位？Nav2 的 `StandardTopicQoS` 是什么组合？**
   - 要点提示：reliable / best_effort；volatile / transient_local；StandardTopicQoS = reliable + volatile + KeepLast(10)
   - 参考章节：[19 Action与通信机制](../19_Action与通信机制.md)；源码 `nav2_ros_common/include/nav2_ros_common/qos_profiles.hpp`

4. **生命周期节点有哪些主状态？它与普通节点的本质差异是什么？**
   - 要点提示：Unconfigured / Inactive / Active / Finalized；on_configure 等转换回调；"何时开始工作"交给外部编排者
   - 参考章节：[05 生命周期管理](../05_生命周期管理.md)

5. **TF 树 map→odom→base_link 各段由谁发布？为什么定位不直接发布 map→base_link？**
   - 要点提示：AMCL 发 map→odom，里程计源发 odom→base_link；TF 单亲约束；odom 段连续平滑、map 段吸收跳变纠偏
   - 参考章节：[13 AMCL定位](../13_AMCL定位.md)

## 中级

1. **`nav2::LifecycleNode` 在 rclcpp_lifecycle::LifecycleNode 之上加了哪些能力？**
   - 要点提示：bond 集成（createBond/destroyBond）；on_rcl_preshutdown 受控退出；declare_or_get_parameter；autostart_node
   - 参考章节：[05 生命周期管理](../05_生命周期管理.md)、[22 设计模式专题](../22_设计模式专题.md)；源码 `nav2_ros_common/include/nav2_ros_common/lifecycle_node.hpp`

2. **Nav2 各服务器普遍采用"单线程执行器 + 专属线程"模式，`NodeThread` 是什么？**
   - 要点提示：executor + std::thread 的 RAII 封装；每节点独立 spin 互不干扰；Costmap2DROS 的专属执行器线程是典型消费者
   - 参考章节：[20 线程模型](../20_线程模型.md)；源码 `nav2_ros_common/include/nav2_ros_common/node_thread.hpp`

3. **一个 Action 接口在底层由哪些 DDS 实体组成？**
   - 要点提示：三个服务（send_goal / cancel_goal / get_result）+ 两个话题（feedback / status）；goal_id 贯穿关联
   - 参考章节：[19 Action与通信机制](../19_Action与通信机制.md)

4. **transform_tolerance 在 AMCL 与 Costmap2DROS 中的用法有什么不同？为什么 AMCL 要把变换"贴现到未来"？**
   - 要点提示：AMCL 把 map→odom 的时间戳加 transform_tolerance（默认 1.0 s）未来化，保证下次更新前查询不过期；Costmap 用它做 lookupTransform 的超时与新鲜度判定（默认 0.3 s）
   - 参考章节：[13 AMCL定位](../13_AMCL定位.md)、[06 Costmap2D](../06_Costmap2D.md)；源码 `nav2_amcl/src/amcl_node.cpp` 的 `sendMapToOdomTransform()`（约 903 行）

5. **use_sim_time 如何影响定时与限频？为什么控制循环要用 `nav2::Rate` 而不是 WallRate？**
   - 要点提示：节点时钟可切到 /clock 源；WallRate 在仿真加减速/回放时节拍失配；nav2::Rate 绑定节点时钟
   - 参考章节：[20 线程模型](../20_线程模型.md)、[21 配置系统](../21_配置系统.md)；源码 `nav2_ros_common/include/nav2_ros_common/rate.hpp`

## 高级

1. **SimpleActionServer 构造时传入 `spin_thread=true`，内部会发生什么？解决什么问题？**
   - 要点提示：创建不自动挂入宿主执行器的 MutuallyExclusive 回调组 + 专属 SingleThreadedExecutor；Action 回调与宿主节点其余回调线程隔离，互不阻塞
   - 参考章节：[19 Action与通信机制](../19_Action与通信机制.md)、[20 线程模型](../20_线程模型.md)；源码 `nav2_ros_common/include/nav2_ros_common/simple_action_server.hpp` 构造函数（约 96 行起）

2. **把某个受管节点的 `bond_heartbeat_period` 设为 0 会发生什么？什么场景会这么用？**
   - 要点提示：createBond 只在该值 > 0 时才建 bond，节点从此脱离 lifecycle_manager 的存活监督；调试/单节点手动驱动场景；管理器侧 bond_timeout=0 则全局关闭巡检
   - 参考章节：[05 生命周期管理](../05_生命周期管理.md)；源码 `nav2_ros_common/include/nav2_ros_common/lifecycle_node.hpp` 的 `createBond()`（约 366 行）

3. **QoS 兼容性判定规则（Request vs Offered）是什么？举一个 Nav2 中典型的"静默不连接"组合。**
   - 要点提示：订阅端请求的级别不能高于发布端提供的；best_effort 发布 + reliable 订阅不连；volatile 发布 + transient_local 订阅不连（订地图最常见翻车点）
   - 参考章节：[19 Action与通信机制](../19_Action与通信机制.md)

4. **组合（composition）部署下进程内通信如何生效？对 Nav2 的收益与风险各是什么？**
   - 要点提示：同容器组件走 intra-process 零拷贝；省序列化与网络栈；回调阻塞跨节点传染、单组件崩溃带走全容器
   - 参考章节：[20 线程模型](../20_线程模型.md)、[01 整体架构](../01_整体架构.md)

## 架构设计

1. **cmd_vel 链路为什么设计成"控制器与 behavior_server 并联输出 cmd_vel_nav → velocity_smoother → cmd_vel_smoothed → collision_monitor → cmd_vel"的串联安全链？**
   - 要点提示：所有速度源先收敛再统一平滑；collision_monitor 位于最后一环拥有最终否决权；若平滑放在安全裁决之后会破坏裁决结果
   - 参考章节：[16 CollisionMonitor与VelocitySmoother](../16_CollisionMonitor与VelocitySmoother.md)、[01 整体架构](../01_整体架构.md)

2. **生命周期编排为什么采用集中式 lifecycle_manager，而不是让各节点自治地等待依赖就绪？**
   - 要点提示：全局顺序可控（正序启动/逆序关停）；故障语义统一——bond 断开即"一个倒下全体放倒 + 整体重启"；自治等待易出现启动死锁与状态发散
   - 参考章节：[05 生命周期管理](../05_生命周期管理.md)

3. **Nav2 为什么坚持用 Action 作为服务器之间的任务边界，而不是库调用或共享内存？**
   - 要点提示：跨进程可部署、可抢占可反馈、错误码契约稳定、实现可整体替换；代价是序列化与调度延迟
   - 参考章节：[19 Action与通信机制](../19_Action与通信机制.md)、[01 整体架构](../01_整体架构.md)

4. **如果需求是"定位挂掉时仍允许遥控与建图"这类部分降级运行，现有 bond 全体放倒机制该怎么改？**
   - 要点提示：按依赖子图分组多管理器（Nav2 的 localization/navigation 双 lifecycle_manager 已是分组实践）；降级策略分级而非二值生死；健康状态对外暴露
   - 参考章节：[05 生命周期管理](../05_生命周期管理.md)、[01 整体架构](../01_整体架构.md)

## 源码分析

1. **SimpleActionServer 中新 goal 的执行回调是如何被调度到线程上的？**
   - 要点提示：`std::async(std::launch::async, ...)` 启动 work 闭包（约 237 行）；可选 `setSoftRealTimePriority()` 提升执行线程调度优先级；completion_callback 收尾
   - 参考章节：[19 Action与通信机制](../19_Action与通信机制.md)、[20 线程模型](../20_线程模型.md)；源码 `nav2_ros_common/include/nav2_ros_common/simple_action_server.hpp`

2. **lifecycle_manager 的 `checkBondConnections()`（约 534 行）如何判死并触发系统级复位？**
   - 要点提示：200 ms 定时器巡检 bond_map_；发现断链 → CRITICAL 日志 → hard reset 逆序放倒；随后在 bond_respawn_max_duration（默认 10 s）内探测全员复活并重新 startup
   - 参考章节：[05 生命周期管理](../05_生命周期管理.md)；源码 `nav2_lifecycle_manager/src/lifecycle_manager.cpp`

3. **`nav2::LifecycleNode` 的 `on_rcl_preshutdown()` 机制解决什么问题？**
   - 要点提示：注册 rclcpp pre-shutdown 回调（约 419 行），Ctrl+C 时先执行受控清理（销毁 bond、runCleanups）再进入 rcl 关闭；规避静态析构顺序竞态
   - 参考章节：[05 生命周期管理](../05_生命周期管理.md)、[23 现代C++专题](../23_现代C++专题.md)；源码 `nav2_ros_common/include/nav2_ros_common/lifecycle_node.hpp`

4. **qos_profiles.hpp 为什么用"继承 rclcpp::QoS 的具名类"（StandardTopicQoS / LatchedPublisherQoS 等）而不是散落的常量？**
   - 要点提示：类型名即语义文档；构造参数保留 depth 微调口；全栈 QoS 口径收敛到单一文件，杜绝各处手写 magic 组合
   - 参考章节：[19 Action与通信机制](../19_Action与通信机制.md)、[22 设计模式专题](../22_设计模式专题.md)；源码 `nav2_ros_common/include/nav2_ros_common/qos_profiles.hpp`

## 开放题

1. **Nav2 能不能做到硬实时？瓶颈分别在哪一层？**
   - 要点提示：DDS 通信抖动、executor 调度不确定、动态内存分配；现有 SimpleActionServer/控制器仅提供软实时优先级钩子；讨论"软实时 + 底盘侧兜底"是否足够
   - 参考章节：[20 线程模型](../20_线程模型.md)、[09 ControllerServer与局部控制](../09_ControllerServer与局部控制.md)

2. **如果把 rmw 换成 zenoh，或把导航栈跨广域网分布部署，哪些设计假设会被打破？**
   - 要点提示：局域网发现假设、QoS 语义映射、TF 高频广播的带宽、bond 心跳超时的时延余量
   - 参考章节：[19 Action与通信机制](../19_Action与通信机制.md)、[01 整体架构](../01_整体架构.md)

3. **ROS 2 参数系统适合承载 Nav2 数百个参数的配置吗？谈谈声明式参数、YAML 快照、运行期动态更新三者的张力。**
   - 要点提示：声明期校验 vs 运行期一致性；动态更新回调的原子性；参数与生命周期状态的耦合
   - 参考章节：[21 配置系统](../21_配置系统.md)
