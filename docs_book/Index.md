# 导航枢纽（Index）

> 口径：Nav2 1.5.0（main 分支）、C++20。本页是全书的**唯一入口**：先按需求找入口，再按角色选路径。阅读进度自查表见 [ReadingProgress.md](ReadingProgress.md)。

## 一、"你想…"快速入口

| 你想… | 从这里开始 |
|---|---|
| 想把 Nav2 跑起来 | [00_项目概览.md](00_项目概览.md) → [25_学习路线图.md](25_学习路线图.md)（25.3 实践阶梯：loopback_sim 跑通） |
| 想看懂一次导航的全流程 | [01_整体架构.md](01_整体架构.md) → [Assets/sequence/NavigateToPose.md](Assets/sequence/NavigateToPose.md) |
| 想系统读源码 | [ClassIndex.md](ClassIndex.md)（Top 10 必学类阅读顺序）→ [FileIndex.md](FileIndex.md) → 各章 X.A 源码导航板块 |
| 想写自己的插件 | [04_nav2_core插件接口.md](04_nav2_core插件接口.md) → [24_扩展开发指南.md](24_扩展开发指南.md) → [Examples/PlannerPlugin.md](Examples/PlannerPlugin.md) 等五个实战示例 |
| 想刷面试题 | [Interview/ROS2基础.md](Interview/ROS2基础.md) 起步，各章"常见面试题"小节补充 |
| 想查参数怎么配 | [21_配置系统.md](21_配置系统.md) → 各域 [CheatSheet/](CheatSheet/Lifecycle.md) 速查卡 |
| 想查一个术语/类/函数 | [Glossary.md](Glossary.md) / [ClassIndex.md](ClassIndex.md) / [APIIndex.md](APIIndex.md) |
| 想知道"为什么这样设计" | [FAQ.md](FAQ.md) 速答 → 各章 X.B 设计权衡 → [Evolution/](Evolution/Planner.md) 演进史 |
| 想选算法（规划器/控制器怎么挑） | [CheatSheet/Planner.md](CheatSheet/Planner.md)、[CheatSheet/Controller.md](CheatSheet/Controller.md) |
| 想排查"机器人不动了" | [KnowledgeGraph/Business.md](KnowledgeGraph/Business.md)（沿链路从后往前定位） |

## 二、三索引与四图谱

| 类型 | 文件 | 用途 |
|---|---|---|
| 索引 | [ClassIndex.md](ClassIndex.md) | 类 → 成员/方法/章节/源码位置，含 Top 10 必学类 |
| 索引 | [FileIndex.md](FileIndex.md) | 源码文件 → 职责/章节 |
| 索引 | [APIIndex.md](APIIndex.md) | 函数级双向索引 |
| 图谱 | [KnowledgeGraph/Knowledge.md](KnowledgeGraph/Knowledge.md) | 知识点前置依赖：先学什么后学什么 |
| 图谱 | [KnowledgeGraph/Architecture.md](KnowledgeGraph/Architecture.md) | 组件分层：谁管理谁、谁依赖谁 |
| 图谱 | [KnowledgeGraph/Business.md](KnowledgeGraph/Business.md) | 业务链路：请求如何变成轮速，含排障走法 |
| 图谱 | [KnowledgeGraph/Plugin.md](KnowledgeGraph/Plugin.md) | 插件族谱：接口 → 全部官方实现 → 章节 |

## 三、专题资料全目录

**CheatSheet/ 速查卡（6）**：[Lifecycle.md](CheatSheet/Lifecycle.md) ｜ [Action.md](CheatSheet/Action.md) ｜ [Plugin.md](CheatSheet/Plugin.md) ｜ [Costmap.md](CheatSheet/Costmap.md) ｜ [Planner.md](CheatSheet/Planner.md) ｜ [Controller.md](CheatSheet/Controller.md)

**Examples/ 实战示例（5）**：[PlannerPlugin.md](Examples/PlannerPlugin.md) ｜ [ControllerPlugin.md](Examples/ControllerPlugin.md) ｜ [CostmapLayer.md](Examples/CostmapLayer.md) ｜ [ActionServer.md](Examples/ActionServer.md) ｜ [LifecycleNode.md](Examples/LifecycleNode.md)

**Interview/ 面试题集（5）**：[ROS2基础.md](Interview/ROS2基础.md) ｜ [Planner.md](Interview/Planner.md) ｜ [Controller.md](Interview/Controller.md) ｜ [Costmap.md](Interview/Costmap.md) ｜ [BT.md](Interview/BT.md)

**Evolution/ 设计演进史（3）**：[Planner.md](Evolution/Planner.md) ｜ [Controller.md](Evolution/Controller.md) ｜ [BehaviorTree.md](Evolution/BehaviorTree.md)

**Assets/sequence/ 端到端时序图（5）**：[NavigateToPose.md](Assets/sequence/NavigateToPose.md) ｜ [NavigateThroughPoses与FollowWaypoints.md](Assets/sequence/NavigateThroughPoses与FollowWaypoints.md) ｜ [Route图路由.md](Assets/sequence/Route图路由.md) ｜ [Docking靠泊.md](Assets/sequence/Docking靠泊.md) ｜ [Costmap更新循环.md](Assets/sequence/Costmap更新循环.md)

**全书辅读**：[Glossary.md](Glossary.md)（术语表）｜ [FAQ.md](FAQ.md)（高频"为什么"）｜ [ReadingProgress.md](ReadingProgress.md)（进度自查）

## 四、三条读者路径

### 路径 A：新手（应用集成向，目标 = 跑通并会调）

[00_项目概览.md](00_项目概览.md) → [01_整体架构.md](01_整体架构.md) → [03_模块总览.md](03_模块总览.md) → [05_生命周期管理.md](05_生命周期管理.md) → [06_Costmap2D.md](06_Costmap2D.md) → [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) → [21_配置系统.md](21_配置系统.md) → [25_学习路线图.md](25_学习路线图.md)
（配套：跑通 loopback_sim，改膨胀半径/控制频率验证现象；速查卡随查随用）

### 路径 B：算法工程师（规划/控制向，目标 = 读懂并能改算法）

[01_整体架构.md](01_整体架构.md) → [04_nav2_core插件接口.md](04_nav2_core插件接口.md) → [06_Costmap2D.md](06_Costmap2D.md) → [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md) → [08_SmacPlanner.md](08_SmacPlanner.md) → [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) → [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md) → [15_Smoother路径平滑.md](15_Smoother路径平滑.md) → [13_AMCL定位.md](13_AMCL定位.md) → [24_扩展开发指南.md](24_扩展开发指南.md)
（配套：[Evolution/](Evolution/Planner.md) 理解算法取舍脉络，[Examples/](Examples/PlannerPlugin.md) 动手写插件）

### 路径 C：架构师（系统机制向，目标 = 吃透设计并可迁移）

[01_整体架构.md](01_整体架构.md) → [02_目录设计.md](02_目录设计.md) → [04_nav2_core插件接口.md](04_nav2_core插件接口.md) → [05_生命周期管理.md](05_生命周期管理.md) → [19_Action与通信机制.md](19_Action与通信机制.md) → [20_线程模型.md](20_线程模型.md) → [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) → [16_CollisionMonitor与VelocitySmoother.md](16_CollisionMonitor与VelocitySmoother.md) → [22_设计模式专题.md](22_设计模式专题.md) → [23_现代C++专题.md](23_现代C++专题.md) → [24_扩展开发指南.md](24_扩展开发指南.md)
（配套：[FAQ.md](FAQ.md) 与各章 X.B 设计权衡连读，[KnowledgeGraph/Architecture.md](KnowledgeGraph/Architecture.md) 常备手边）

## 五、全书章节总目录

| 分组 | 章节 |
|---|---|
| 全景（00~03） | [00_项目概览.md](00_项目概览.md) ｜ [01_整体架构.md](01_整体架构.md) ｜ [02_目录设计.md](02_目录设计.md) ｜ [03_模块总览.md](03_模块总览.md) |
| 骨架（04~05） | [04_nav2_core插件接口.md](04_nav2_core插件接口.md) ｜ [05_生命周期管理.md](05_生命周期管理.md) |
| 核心域（06~11） | [06_Costmap2D.md](06_Costmap2D.md) ｜ [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md) ｜ [08_SmacPlanner.md](08_SmacPlanner.md) ｜ [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) ｜ [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md) ｜ [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) |
| 专题域（12~18） | [12_Behaviors恢复行为.md](12_Behaviors恢复行为.md) ｜ [13_AMCL定位.md](13_AMCL定位.md) ｜ [14_MapServer.md](14_MapServer.md) ｜ [15_Smoother路径平滑.md](15_Smoother路径平滑.md) ｜ [16_CollisionMonitor与VelocitySmoother.md](16_CollisionMonitor与VelocitySmoother.md) ｜ [17_Route路线规划.md](17_Route路线规划.md) ｜ [18_Docking靠泊.md](18_Docking靠泊.md) |
| 机制与素养（19~23） | [19_Action与通信机制.md](19_Action与通信机制.md) ｜ [20_线程模型.md](20_线程模型.md) ｜ [21_配置系统.md](21_配置系统.md) ｜ [22_设计模式专题.md](22_设计模式专题.md) ｜ [23_现代C++专题.md](23_现代C++专题.md) |
| 实战与收官（24~25） | [24_扩展开发指南.md](24_扩展开发指南.md) ｜ [25_学习路线图.md](25_学习路线图.md) |
