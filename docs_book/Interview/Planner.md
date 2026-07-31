# 面试题集：全局规划（NavFn / Smac 系列 / 代价感知 A*）

> 本题集基于 Nav2 1.5.0（main 分支）源码现场核实。
> 每题只给要点提示，不给背诵式完整答案；深入学习请跳转参考章节。

## 初级

1. **Nav2 内置的全局规划器有哪几个包？各自的搜索空间是什么？**
   - 要点提示：nav2_navfn_planner（2D 栅格势场）；nav2_smac_planner（2D / Hybrid / Lattice 三个插件）；nav2_theta_star_planner（any-angle）；nav2_route 是图路由，不是自由空间规划
   - 参考章节：[07 PlannerServer与全局规划](../07_PlannerServer与全局规划.md)、[08 SmacPlanner](../08_SmacPlanner.md)、[17 Route路线规划](../17_Route路线规划.md)

2. **GlobalPlanner 插件契约的核心方法有哪些？分别在什么时机被调用？**
   - 要点提示：configure/activate/deactivate/cleanup 跟随宿主生命周期；createPlan 在每次规划请求时同步调用并携带取消检查器
   - 参考章节：[04 nav2_core插件接口](../04_nav2_core插件接口.md)、[07 PlannerServer与全局规划](../07_PlannerServer与全局规划.md)

3. **NavFn 的"势场"指什么？路径是怎么从势场中提取出来的？**
   - 要点提示：从目标反向传播的最优到达代价场；波前传播；从起点沿势场负梯度回溯成路径
   - 参考章节：[07 PlannerServer与全局规划](../07_PlannerServer与全局规划.md)

4. **代价地图的 0~255 值是如何进入 NavFn 搜索代价的？**
   - 要点提示：0~252 线性折算为 COST_NEUTRAL(50) + COST_FACTOR(0.8)×v；253/254 视为禁行；255（未知）按 allow_unknown 处理
   - 参考章节：[07 PlannerServer与全局规划](../07_PlannerServer与全局规划.md)；源码 `nav2_navfn_planner/include/nav2_navfn_planner/navfn.hpp` 宏定义（约 48 行起）

## 中级

1. **"代价感知 A*"与教科书 A* 差在哪？Smac 2D 的遍历代价公式是什么？**
   - 要点提示：g 值除步长外叠加 1.0 + cost_travel_multiplier × 归一化代价（对角线再乘 √2）；乘子为 0 退化为二值 A*；引导路径在走廊居中
   - 参考章节：[08 SmacPlanner](../08_SmacPlanner.md)；源码 `nav2_smac_planner/src/node_2d.cpp` 的 `getTraversalCost()`（约 76 行）

2. **Hybrid-A* 的 motion table 中，Dubins 与 Reeds-Shepp 模型各生成几条运动基元？差异意味着什么？**
   - 要点提示：initDubin 3 条（前直/前左/前右，约 46 行起）；initReedsShepp 6 条（再加倒车三向，约 175 行起）；allow_primitive_interpolation 可加密中间转角基元
   - 参考章节：[08 SmacPlanner](../08_SmacPlanner.md)；源码 `nav2_smac_planner/src/node_hybrid.cpp`

3. **max_iterations 与 max_planning_time 双重限制为什么都需要？max_on_approach_iterations 又是什么？**
   - 要点提示：迭代数限空间与内存、时间限节拍；两者失配任一先触发即止；接近目标但未达容差时的"限次逼近"退出策略
   - 参考章节：[08 SmacPlanner](../08_SmacPlanner.md)

4. **规划前的代价地图降采样（downsample_costmap / downsampling_factor）为什么能加速？代价是什么？**
   - 要点提示：搜索栅格数按因子平方缩减；聚合取最大代价导致窄通道被误判不可通行；输出路径还需映射回原分辨率
   - 参考章节：[08 SmacPlanner](../08_SmacPlanner.md)

5. **SE(2) 规划中足迹碰撞检查什么时候才真正执行？为什么不是每个节点都查？**
   - 要点提示：中心代价低于"可能内接"阈值时直接放行；只有落在内接~外接灰色带才做足迹多边形逐边检查；多边形检查昂贵，省查是性能关键
   - 参考章节：[06 Costmap2D](../06_Costmap2D.md)、[08 SmacPlanner](../08_SmacPlanner.md)

## 高级

1. **obstacle heuristic 是如何计算出来的？cache_obstacle_heuristic 为什么能加速、何时失效？**
   - 要点提示：在（可降采样的）代价图上从目标做代价感知的 2D 波前扩展，作为 SE(2) 搜索下界；同一目标反复重规划可复用；目标一变即须重算
   - 参考章节：[08 SmacPlanner](../08_SmacPlanner.md)

2. **解析扩展并非每个节点都尝试，它的触发频率是怎么调度的？成功还要满足什么条件？**
   - 要点提示：按启发式估计的目标距离缩放尝试频率，越接近越频繁；整条 Dubins/RS 曲线逐点无碰；长度不超过 analytic_expansion_max_length
   - 参考章节：[08 SmacPlanner](../08_SmacPlanner.md)

3. **reverse_penalty、change_penalty 这类惩罚项如何塑形路径风格？把 reverse_penalty 调得很大会有什么副作用？**
   - 要点提示：乘性软惩罚而非硬约束；过大导致本该短倒车的窄区绕远甚至规划失败；需与控制器 allow_reversing 联动
   - 参考章节：[08 SmacPlanner](../08_SmacPlanner.md)、[10 MPPI与RPP控制器](../10_MPPI与RPP控制器.md)

4. **NavFn 势场传播中 `priInc = 2 × COST_NEUTRAL` 的分批出队策略是什么思路？**
   - 要点提示：按阈值分批处理优先级缓冲区，近似 Dijkstra 的桶排序；牺牲严格最优序换取传播吞吐；ROS 1 时代遗留的工程折衷
   - 参考章节：[07 PlannerServer与全局规划](../07_PlannerServer与全局规划.md)；源码 `nav2_navfn_planner/src/navfn.cpp`（约 124 行）

## 架构设计

1. **为什么 ComputePathToPose 与 ComputePathThroughPoses 是两个独立 Action，而不是一个带路点数组的通用接口？**
   - 要点提示：Goal/Feedback/错误码语义不同；类型安全与接口演进独立；planner_server 同时挂两个 action server 分别绑定回调
   - 参考章节：[07 PlannerServer与全局规划](../07_PlannerServer与全局规划.md)、[19 Action与通信机制](../19_Action与通信机制.md)；源码 `nav2_planner/src/planner_server.cpp` 的 `on_configure()`（约 129 行）

2. **路径校验（IsPathValid）做成服务、路径规划做成 Action，背后的判据是什么？**
   - 要点提示：校验是短同步原子操作，无进度/抢占语义；行为树"1 Hz 限频重规划 + 先校验旧路径"组合的算力经济学
   - 参考章节：[07 PlannerServer与全局规划](../07_PlannerServer与全局规划.md)、[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)

3. **目标不可达时的 tolerance 处理为什么放在各插件内部，而不是 PlannerServer 统一实现？**
   - 要点提示：各算法"退而求其次"语义不同（势场邻域最优 vs 搜索过程中最接近节点）；接口契约只约定行为不约定实现；服务器统一做会丢失算法内部信息
   - 参考章节：[07 PlannerServer与全局规划](../07_PlannerServer与全局规划.md)、[08 SmacPlanner](../08_SmacPlanner.md)

4. **要支持"单行道 / 礼让区域"这类运营规则，应该改规划器还是改代价地图？边界在哪里？**
   - 要点提示：禁入/权重类规则走 costmap filter（Keepout 等）；方向性规则超出各向同性栅格的表达力，需 Route 图或专用层；原则是规则数据不侵入算法实现
   - 参考章节：[06 Costmap2D](../06_Costmap2D.md)、[17 Route路线规划](../17_Route路线规划.md)

## 源码分析

1. **navfn.cpp 的 `setCostmap()`（约 240 行起）对输入代价做了哪几类变换？注释里警告了什么？**
   - 要点提示：0~252 线性映射到 50~252 区间；253→COST_OBS_ROS、254→COST_OBS 特判；unknown 按 allow_unknown 给次高代价；头文件注释警告 COST_FACTOR 过大会在障碍周围产生代价平台，路径质量劣化
   - 参考章节：[07 PlannerServer与全局规划](../07_PlannerServer与全局规划.md)；源码 `nav2_navfn_planner/src/navfn.cpp`

2. **Smac 的 AStarAlgorithm 为什么把实现放在 `*_impl.hpp` 并做显式模板实例化？**
   - 要点提示：NodeT（Node2D/NodeHybrid/NodeLattice）静态绑定 + if constexpr 编译期裁剪；实现与声明分离控制编译时间；避免虚调用开销
   - 参考章节：[08 SmacPlanner](../08_SmacPlanner.md)、[23 现代C++专题](../23_现代C++专题.md)；源码 `nav2_smac_planner/include/nav2_smac_planner/a_star_impl.hpp`

3. **node_2d.cpp 的 `getTraversalCost()`（约 76 行）如何区分直行与对角步？这对启发式有什么要求？**
   - 要点提示：对角步长乘 √2 后再叠加代价项；启发式必须保持下界（可采纳性），否则丢失最优性
   - 参考章节：[08 SmacPlanner](../08_SmacPlanner.md)；源码 `nav2_smac_planner/src/node_2d.cpp`

4. **从插件抛异常到行为树拿到错误码，这条链路在源码中如何走通？**
   - 要点提示：createPlan 抛 nav2_core 语义化异常 → planner_server 的 computePlan catch 后映射 Action Result error_code → BT 节点经 error_code_id 端口写黑板 → 恢复分支研判
   - 参考章节：[07 PlannerServer与全局规划](../07_PlannerServer与全局规划.md)、[04 nav2_core插件接口](../04_nav2_core插件接口.md)、[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)

## 开放题

1. **动态环境里"更快重规划"与"路径稳定性"如何权衡？**
   - 要点提示：滞回机制、新旧路径相似性约束、局部修补 vs 全局重算；讨论默认树"先校验旧路径再重算"的适用边界
   - 参考章节：[07 PlannerServer与全局规划](../07_PlannerServer与全局规划.md)、[11 行为树与BTNavigator](../11_行为树与BTNavigator.md)

2. **全局规划该不该考虑更多动力学？曲率之外的速度/加速度约束放在哪一层最合适？**
   - 要点提示：规划-平滑-控制的职责切分；约束越靠上游路径越可执行但搜索越贵；Lattice 基元编码底盘能力的思路
   - 参考章节：[08 SmacPlanner](../08_SmacPlanner.md)、[15 Smoother路径平滑](../15_Smoother路径平滑.md)

3. **学习型规划器（扩散模型、端到端网络）以 GlobalPlanner 插件形式接入 Nav2，可行性与安全边界在哪里？**
   - 要点提示：接口契约可满足（输入位姿输出 Path）；不可解释失败模式与 253/254 安全语义的冲突；建议的混合架构（学习提议 + 经典校验）
   - 参考章节：[04 nav2_core插件接口](../04_nav2_core插件接口.md)、[24 扩展开发指南](../24_扩展开发指南.md)
