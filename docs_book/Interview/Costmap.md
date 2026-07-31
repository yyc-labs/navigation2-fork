# 面试题集：代价地图（分层代价地图 / 膨胀 / 传感器融合）

> 本题集基于 Nav2 1.5.0（main 分支）源码现场核实。
> 每题只给要点提示，不给背诵式完整答案；深入学习请跳转参考章节。

## 初级

1. **全局代价地图与局部代价地图有什么差别？各服务于谁？**
   - 要点提示：全图静态框架 vs rolling window 滚动窗口；PlannerServer 与 ControllerServer 各持一份；尺寸/层配置/更新频率取向不同
   - 参考章节：[06 Costmap2D](../06_Costmap2D.md)、[07 PlannerServer与全局规划](../07_PlannerServer与全局规划.md)

2. **Nav2 内置的 Layer 插件有哪些？filter 有哪些？**
   - 要点提示：Static / Obstacle / Voxel / RangeSensor / Denoise / Inflation（另有 Legacy、Asymmetric 变体）/ PluginContainerLayer；filters：Keepout / Speed / Binary
   - 参考章节：[06 Costmap2D](../06_Costmap2D.md)；源码 `nav2_costmap_2d/costmap_plugins.xml`

3. **footprint 与 robot_radius 怎么选？它们决定了哪些派生量？**
   - 要点提示：多边形足迹 vs 圆形近似；决定内接/外接半径，进而决定 253 判定阈值与足迹碰撞检查的必要性
   - 参考章节：[06 Costmap2D](../06_Costmap2D.md)

4. **rolling window 模式下代价地图如何跟随机器人移动？**
   - 要点提示：每次更新按机器人位姿平移原点；重叠区数据保留、新进入区置默认值
   - 参考章节：[06 Costmap2D](../06_Costmap2D.md)

## 中级

1. **各层把数据写进主图时有哪几种合成（combination）语义？NO_INFORMATION 特殊在哪里？**
   - 要点提示：覆盖写 / 取最大 / 未知不覆盖三类；cost_values.hpp 头部注释（约 51~64 行）逐一定义；track_unknown_space 决定未知是否保留
   - 参考章节：[06 Costmap2D](../06_Costmap2D.md)；源码 `nav2_costmap_2d/include/nav2_costmap_2d/cost_values.hpp`

2. **VoxelLayer 相比 ObstacleLayer 多了什么？什么场景必须用它？**
   - 要点提示：三维体素栅格做标记/清除后投影 2D；mark_threshold 抑噪；深度相机等 3D 传感器、桌面悬空障碍场景
   - 参考章节：[06 Costmap2D](../06_Costmap2D.md)；源码 `nav2_costmap_2d/plugins/voxel_layer.cpp`、`nav2_voxel_grid/`

3. **DenoiseLayer 解决什么问题？它在插件序列里应该放在哪？**
   - 要点提示：剔除孤立障碍点/小连通域（传感器盐噪声）；置于障碍类层之后、膨胀层之前，否则噪点会被膨胀放大
   - 参考章节：[06 Costmap2D](../06_Costmap2D.md)；源码 `nav2_costmap_2d/plugins/denoise_layer.cpp`

4. **StaticLayer 的数据从哪里来？为什么它的订阅必须用 transient_local？SLAM 模式下有何不同？**
   - 要点提示：订阅 map 话题而非拉服务；锁存语义补收最后一帧地图；SLAM 时地图持续更新需允许多次接收
   - 参考章节：[14 MapServer](../14_MapServer.md)、[06 Costmap2D](../06_Costmap2D.md)

5. **为什么膨胀层必须放在插件列表最后？放错位置会怎样？**
   - 要点提示：膨胀读取的是前序各层合成后的障碍分布；放前面则新障碍不被膨胀、或膨胀结果被后写层覆盖；顺序即数据流
   - 参考章节：[06 Costmap2D](../06_Costmap2D.md)

## 高级

1. **InflationLayer 的 `computeCaches()`（约 352 行）缓存了什么？什么时候需要重算？**
   - 要点提示：距离→代价、距离矩阵两张查找表；膨胀时查表免去逐格算 exp；inflation_radius、cost_scaling_factor 或分辨率变化时重建
   - 参考章节：[06 Costmap2D](../06_Costmap2D.md)；源码 `nav2_costmap_2d/plugins/inflation_layer.cpp`

2. **InflationLayer::updateBounds 为什么要把脏区域向外再扩一个膨胀半径？**
   - 要点提示：边界外侧的 LETHAL 会影响边界内侧代价；不外扩会在增量更新边界留下"缝合痕"；代价是每次重算区域更大
   - 参考章节：[06 Costmap2D](../06_Costmap2D.md)

3. **Costmap Filter 的 filter_info + filter_mask 双话题协议为什么这样设计？**
   - 要点提示：info 描述 mask 的数值编码（base/multiplier）与 mask 话题名；mask 地图可独立于导航地图（分辨率/范围不同）；运营语义与感知数据彻底分离
   - 参考章节：[06 Costmap2D](../06_Costmap2D.md)

4. **更新线程、执行器线程、外部读者三方并发访问 costmap，如何保证一致性？**
   - 要点提示：mapUpdateLoop 定频写图（update_frequency 默认 5 Hz）；recursive_mutex 保护主图；stop/pause 原子标志与生命周期状态协作
   - 参考章节：[06 Costmap2D](../06_Costmap2D.md)、[20 线程模型](../20_线程模型.md)

## 架构设计

1. **PluginContainerLayer 解决什么问题？举一个必须用它的场景。**
   - 要点提示：把多个层编组为一个复合层，组内使用独立的合成策略；实现"两套障碍源（如激光组与视觉组）互不污染、分别膨胀"类需求
   - 参考章节：[06 Costmap2D](../06_Costmap2D.md)；源码 `nav2_costmap_2d/plugins/plugin_container_layer.cpp`

2. **新接入一种毫米波雷达：直接作为 ObstacleLayer 的一路数据源，还是写一个新 Layer？判据是什么？**
   - 要点提示：数据形态可转 scan/点云则复用现成 marking/clearing；噪声模型特殊（多径、鬼影）需要专用滤波则独立成层；RangeSensorLayer 是超声独立成层的先例
   - 参考章节：[06 Costmap2D](../06_Costmap2D.md)、[24 扩展开发指南](../24_扩展开发指南.md)

3. **把代价地图分辨率从 0.05 m 提高到 0.025 m，做一次系统性影响评估。**
   - 要点提示：内存与每帧更新耗时约 4 倍；膨胀半径栅格数、raytrace 开销联动；Smac 搜索规模与降采样参数、控制端足迹检查成本重估
   - 参考章节：[06 Costmap2D](../06_Costmap2D.md)、[08 SmacPlanner](../08_SmacPlanner.md)

## 源码分析

1. **costmap_2d_ros.cpp 的 `mapUpdateLoop()`（约 533 行）每圈做什么？更新频率与发布频率为什么分开？**
   - 要点提示：取机器人位姿 → layered_costmap 两阶段更新 → 按 publish_frequency 节流发布；更新服务于本地消费者、发布只服务于可视化与旁路订阅者
   - 参考章节：[06 Costmap2D](../06_Costmap2D.md)、[20 线程模型](../20_线程模型.md)；源码 `nav2_costmap_2d/src/costmap_2d_ros.cpp`

2. **getRobotPose 依赖哪些配置？现场排查它失败的思路？**
   - 要点提示：tf_buffer + global_frame/robot_base_frame + transform_tolerance（默认 0.3 s）；排查 TF 断链、时钟不同步（sim time）、tolerance 过小三类
   - 参考章节：[06 Costmap2D](../06_Costmap2D.md)、[13 AMCL定位](../13_AMCL定位.md)；源码 `nav2_costmap_2d/src/costmap_2d_ros.cpp` 的 `getRobotPose()`（约 740 行）

3. **inflation_layer.cpp 的波前膨胀主循环用什么数据结构组织？为什么不是普通 BFS 队列？**
   - 要点提示：按距离分级的待膨胀单元桶（层级队列）+ seen 数组去重；保证按距离递增出队，代价单调写入不需回退
   - 参考章节：[06 Costmap2D](../06_Costmap2D.md)；源码 `nav2_costmap_2d/plugins/inflation_layer.cpp`

4. **cost_values.hpp 里 MAX_NON_OBSTACLE(252) 的存在意义是什么？**
   - 要点提示：0~252 是"可通行代价谱"，253/254/255 是语义保留值；层实现写代价时以 252 为上限避免误伤安全语义
   - 参考章节：[06 Costmap2D](../06_Costmap2D.md)；源码 `nav2_costmap_2d/include/nav2_costmap_2d/cost_values.hpp`（约 76 行）

## 开放题

1. **行人等动态障碍在静态栅格模型中的根本局限是什么？有哪些补救方向？**
   - 要点提示：无速度维度、清除延迟产生拖尾/幽灵；速度障碍层、预测轨迹代价、时空（x,y,t）扩展的取舍
   - 参考章节：[06 Costmap2D](../06_Costmap2D.md)、[10 MPPI与RPP控制器](../10_MPPI与RPP控制器.md)

2. **10 万平米级仓库，单张全局 costmap 撑不住时怎么办？**
   - 要点提示：Route 图路由 + 局部自由空间规划的分工；多分辨率/分片加载；全局层降配（只留 static+膨胀）
   - 参考章节：[17 Route路线规划](../17_Route路线规划.md)、[06 Costmap2D](../06_Costmap2D.md)

3. **语义信息（草地可走但不优先、地毯区限速、坡道）如何进入代价体系而不破坏 253/254 安全语义？**
   - 要点提示：0~252 谱段表达偏好、保留值不可占用；限速类走 SpeedFilter 而非代价；讨论语义层与规则 mask 的生产流程
   - 参考章节：[06 Costmap2D](../06_Costmap2D.md)、[21 配置系统](../21_配置系统.md)
