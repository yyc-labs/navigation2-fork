# 类索引（ClassIndex）

> 口径：Nav2 1.5.0（main 分支）、C++20。源码位置为仓库相对路径；重要程度 ★~★★★★★（★★★★★=必学）。
> 配套索引：[FileIndex.md](FileIndex.md)（文件索引）、[APIIndex.md](APIIndex.md)（函数级双向索引）。

## 必学类 Top 10 阅读顺序表

| 序号 | 类名 | 为什么先读它 | 章节 |
|---|---|---|---|
| 1 | `nav2::LifecycleNode` | 所有 Nav2 服务器的共同基类，不懂它就看不懂任何服务器的 on_configure/on_activate 骨架 | [05_生命周期管理.md](05_生命周期管理.md) |
| 2 | `nav2::SimpleActionServer<ActionT>` | 每个服务器的任务入口与抢占语义都由它定义，是理解"目标如何变成执行"的钥匙 | [19_Action与通信机制.md](19_Action与通信机制.md) |
| 3 | `Costmap2D` | 最纯粹的数据结构，规划与控制的公共底座，读它建立栅格/世界坐标的直觉 | [06_Costmap2D.md](06_Costmap2D.md) |
| 4 | `Costmap2DROS` | 把 Costmap2D 接入 ROS 世界：线程、TF、插件装载，一个文件看懂 Nav2 的组合式设计 | [06_Costmap2D.md](06_Costmap2D.md) |
| 5 | `nav2_core::GlobalPlanner` / `nav2_core::Controller` | 两大核心插件接口，全部算法插件的契约，读完才能看懂插件实现 | [04_nav2_core插件接口.md](04_nav2_core插件接口.md) |
| 6 | `PlannerServer` | 最短小完整的服务器实现（约 694 行），Action→插件→结果的标准样板 | [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md) |
| 7 | `ControllerServer` | 唯一的高频实时循环服务器，理解 Nav2 控制热路径的必经之地 | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) |
| 8 | `BtActionServer<ActionT>` | BT XML 如何被加载、tick、取消的全部机制所在 | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) |
| 9 | `BtActionNode<ActionT>` | 行为树叶节点与 ROS Action 的桥梁，几十个 BT 插件都长它这样 | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) |
| 10 | `LifecycleManager` | 从系统视角看全部服务器如何被拉起、监活与关停，收束前 9 个类的全局图景 | [05_生命周期管理.md](05_生命周期管理.md) |

阅读顺序建议：**第一阶段**=Top 10 必学类；**第二阶段**=各业务域主干类（服务器与默认插件）；**进阶**=算法内核与特化插件。

## 一、基础设施与工具类（nav2_ros_common / nav2_util）

| 类名 | 模块 | 重要程度 | 阅读顺序 | 主要成员 | 关键方法 | 关联章节 | 源码位置 |
|---|---|---|---|---|---|---|---|
| `nav2::LifecycleNode` | nav2_ros_common | ★★★★★ | 第一阶段 | bond 连接、rcl 预关闭回调 | `createBond` `destroyBond` `on_rcl_preshutdown` | [05_生命周期管理.md](05_生命周期管理.md)、[CheatSheet/Lifecycle.md](CheatSheet/Lifecycle.md)、[Examples/LifecycleNode.md](Examples/LifecycleNode.md) | nav2_ros_common/include/nav2_ros_common/lifecycle_node.hpp |
| `nav2::SimpleActionServer<ActionT>` | nav2_ros_common | ★★★★★ | 第一阶段 | `current_handle_` `pending_handle_` `execution_future_` `update_mutex_` | `handle_accepted` `work` `is_preempt_requested` `accept_pending_goal` `terminate_current` `succeeded_current` | [19_Action与通信机制.md](19_Action与通信机制.md)、[CheatSheet/Action.md](CheatSheet/Action.md)、[Examples/ActionServer.md](Examples/ActionServer.md) | nav2_ros_common/include/nav2_ros_common/simple_action_server.hpp |
| `nav2::ServiceClient<ServiceT>` | nav2_ros_common | ★★★ | 第二阶段 | 内部独立 executor | `invoke` `wait_for_service` | [19_Action与通信机制.md](19_Action与通信机制.md) | nav2_ros_common/include/nav2_ros_common/service_client.hpp |
| `nav2::NodeThread` | nav2_ros_common | ★★ | 进阶 | 独立 spin 线程 | 构造即 spin | [20_线程模型.md](20_线程模型.md) | nav2_ros_common/include/nav2_ros_common/node_thread.hpp |
| `nav2_util::OdomSmoother` | nav2_util | ★★★ | 第二阶段 | 里程计滑动窗口 | `getTwist` `updateState` | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) | nav2_util/include/nav2_util/odometry_utils.hpp |
| `nav2_util::TwistPublisher` | nav2_util | ★★★ | 第二阶段 | Twist/TwistStamped 双发布器 | `publish` | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) | nav2_util/include/nav2_util/twist_publisher.hpp |
| `nav2_util::TwistSubscriber` | nav2_util | ★★ | 进阶 | 双格式订阅 | 构造注册回调 | [16_CollisionMonitor与VelocitySmoother.md](16_CollisionMonitor与VelocitySmoother.md) | nav2_util/include/nav2_util/twist_subscriber.hpp |
| `nav2_util::LifecycleServiceClient` | nav2_util | ★★ | 进阶 | change_state/get_state 客户端 | `change_state` `get_state` | [05_生命周期管理.md](05_生命周期管理.md) | nav2_util/include/nav2_util/lifecycle_service_client.hpp |

## 二、nav2_core 插件接口

| 类名 | 模块 | 重要程度 | 阅读顺序 | 主要成员 | 关键方法 | 关联章节 | 源码位置 |
|---|---|---|---|---|---|---|---|
| `nav2_core::GlobalPlanner` | nav2_core | ★★★★★ | 第一阶段 | 纯虚接口，无成员 | `configure` `activate` `deactivate` `cleanup` `createPlan` | [04_nav2_core插件接口.md](04_nav2_core插件接口.md)、[CheatSheet/Planner.md](CheatSheet/Planner.md)、[Examples/PlannerPlugin.md](Examples/PlannerPlugin.md) | nav2_core/include/nav2_core/global_planner.hpp |
| `nav2_core::Controller` | nav2_core | ★★★★★ | 第一阶段 | 纯虚接口，无成员 | `configure` `setPlan` `computeVelocityCommands` `setSpeedLimit` | [04_nav2_core插件接口.md](04_nav2_core插件接口.md)、[CheatSheet/Controller.md](CheatSheet/Controller.md)、[Examples/ControllerPlugin.md](Examples/ControllerPlugin.md) | nav2_core/include/nav2_core/controller.hpp |
| `nav2_core::Smoother` | nav2_core | ★★★ | 第二阶段 | 纯虚接口 | `smooth` | [15_Smoother路径平滑.md](15_Smoother路径平滑.md) | nav2_core/include/nav2_core/smoother.hpp |
| `nav2_core::Behavior` | nav2_core | ★★★ | 第二阶段 | 纯虚接口 | `configure` `activate` `run` 由子类扩展 | [12_Behaviors恢复行为.md](12_Behaviors恢复行为.md) | nav2_core/include/nav2_core/behavior.hpp |
| `nav2_core::GoalChecker` | nav2_core | ★★★★ | 第二阶段 | 纯虚接口 | `isGoalReached` `getTolerances` | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) | nav2_core/include/nav2_core/goal_checker.hpp |
| `nav2_core::ProgressChecker` | nav2_core | ★★★ | 第二阶段 | 纯虚接口 | `check` `reset` | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) | nav2_core/include/nav2_core/progress_checker.hpp |
| `nav2_core::BehaviorTreeNavigator<ActionT>` | nav2_core | ★★★★ | 第二阶段 | `bt_action_server_`（模板基类持有） | `on_configure` `goalReceived` `onLoop` `onPreempt` `goalCompleted` | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) | nav2_core/include/nav2_core/behavior_tree_navigator.hpp |
| `nav2_core::WaypointTaskExecutor` | nav2_core | ★★ | 进阶 | 纯虚接口 | `processAtWaypoint` | [19_Action与通信机制.md](19_Action与通信机制.md) | nav2_core/include/nav2_core/waypoint_task_executor.hpp |

## 三、生命周期管理

| 类名 | 模块 | 重要程度 | 阅读顺序 | 主要成员 | 关键方法 | 关联章节 | 源码位置 |
|---|---|---|---|---|---|---|---|
| `LifecycleManager` | nav2_lifecycle_manager | ★★★★★ | 第一阶段 | 受管节点名列表、bond 连接表、监活定时器 | `startup` `shutdown` `reset` `changeStateForAllNodes` `createBondConnection` `checkBondConnections` | [05_生命周期管理.md](05_生命周期管理.md) | nav2_lifecycle_manager/src/lifecycle_manager.cpp |
| `LifecycleManagerClient` | nav2_lifecycle_manager | ★★ | 第二阶段 | manage/is_active 服务客户端 | `startup` `shutdown` `pause` `resume` `reset` | [05_生命周期管理.md](05_生命周期管理.md) | nav2_lifecycle_manager/src/lifecycle_manager_client.cpp |

## 四、Costmap 体系（nav2_costmap_2d）

| 类名 | 模块 | 重要程度 | 阅读顺序 | 主要成员 | 关键方法 | 关联章节 | 源码位置 |
|---|---|---|---|---|---|---|---|
| `Costmap2D` | nav2_costmap_2d | ★★★★★ | 第一阶段 | 代价数组、分辨率/原点/尺寸 | `getCost` `setCost` `worldToMap` `mapToWorld` `resizeMap` `updateOrigin` `setConvexPolygonCost` | [06_Costmap2D.md](06_Costmap2D.md)、[CheatSheet/Costmap.md](CheatSheet/Costmap.md) | nav2_costmap_2d/src/costmap_2d.cpp |
| `Costmap2DROS` | nav2_costmap_2d | ★★★★★ | 第一阶段 | `layered_costmap_` `map_update_thread_` `map_update_thread_shutdown_` | `on_configure` `on_activate` `mapUpdateLoop` `updateMap` `getRobotPose` | [06_Costmap2D.md](06_Costmap2D.md)、[20_线程模型.md](20_线程模型.md) | nav2_costmap_2d/src/costmap_2d_ros.cpp |
| `LayeredCostmap` | nav2_costmap_2d | ★★★★ | 第二阶段 | `primary_costmap_` `combined_costmap_`、插件/过滤器列表 | `updateMap` `resizeMap` `addPlugin` | [06_Costmap2D.md](06_Costmap2D.md) | nav2_costmap_2d/src/layered_costmap.cpp |
| `Layer` | nav2_costmap_2d | ★★★★ | 第二阶段 | `layered_costmap_` `enabled_` | `initialize` `updateBounds` `updateCosts`（纯虚） | [06_Costmap2D.md](06_Costmap2D.md)、[Examples/CostmapLayer.md](Examples/CostmapLayer.md) | nav2_costmap_2d/include/nav2_costmap_2d/layer.hpp |
| `CostmapLayer` | nav2_costmap_2d | ★★★ | 第二阶段 | 继承 Layer + Costmap2D 私有栅格 | `updateWithMax` `updateWithOverwrite` `touch` | [06_Costmap2D.md](06_Costmap2D.md) | nav2_costmap_2d/src/costmap_layer.cpp |
| `StaticLayer` | nav2_costmap_2d | ★★★★ | 第二阶段 | 地图订阅、`map_buffer_` | `incomingMap` `processMap` `updateBounds` `updateCosts` | [06_Costmap2D.md](06_Costmap2D.md) | nav2_costmap_2d/plugins/static_layer.cpp |
| `ObstacleLayer` | nav2_costmap_2d | ★★★★ | 第二阶段 | 观测缓冲列表（marking/clearing） | `updateBounds` `updateCosts` `raytraceFreespace` | [06_Costmap2D.md](06_Costmap2D.md) | nav2_costmap_2d/plugins/obstacle_layer.cpp |
| `VoxelLayer` | nav2_costmap_2d | ★★★ | 进阶 | 3D 体素网格 | `updateBounds` `raytraceFreespace`（3D 版） | [06_Costmap2D.md](06_Costmap2D.md) | nav2_costmap_2d/plugins/voxel_layer.cpp |
| `InflationLayer` | nav2_costmap_2d | ★★★★ | 第二阶段 | 代价/距离缓存矩阵 | `updateCosts` `computeCaches` | [06_Costmap2D.md](06_Costmap2D.md) | nav2_costmap_2d/plugins/inflation_layer.cpp |
| `DenoiseLayer` | nav2_costmap_2d | ★★ | 进阶 | 图像去噪算子 | `updateCosts` | [06_Costmap2D.md](06_Costmap2D.md) | nav2_costmap_2d/plugins/denoise_layer.cpp |
| `CostmapFilter` | nav2_costmap_2d | ★★★ | 进阶 | 过滤信息/掩码订阅 | `updateCosts`（模板方法骨架） | [06_Costmap2D.md](06_Costmap2D.md)、[22_设计模式专题.md](22_设计模式专题.md) | nav2_costmap_2d/plugins/costmap_filters/costmap_filter.cpp |
| `KeepoutFilter` | nav2_costmap_2d | ★★ | 进阶 | 掩码地图 | `process` | [06_Costmap2D.md](06_Costmap2D.md) | nav2_costmap_2d/plugins/costmap_filters/keepout_filter.cpp |
| `SpeedFilter` | nav2_costmap_2d | ★★ | 进阶 | 限速区掩码、限速发布 | `process` | [06_Costmap2D.md](06_Costmap2D.md) | nav2_costmap_2d/plugins/costmap_filters/speed_filter.cpp |
| `ObservationBuffer` | nav2_costmap_2d | ★★★ | 进阶 | 观测队列、TF 缓冲 | `bufferCloud` `getObservations` | [06_Costmap2D.md](06_Costmap2D.md) | nav2_costmap_2d/src/observation_buffer.cpp |
| `FootprintCollisionChecker<CostmapT>` | nav2_costmap_2d | ★★★ | 第二阶段 | costmap 句柄 | `footprintCost` `footprintCostAtPose` `worldToMap` | [06_Costmap2D.md](06_Costmap2D.md) | nav2_costmap_2d/src/footprint_collision_checker.cpp |
| `CostmapTopicCollisionChecker` | nav2_costmap_2d | ★★ | 进阶 | costmap/footprint 订阅器 | `isCollisionFree` | [06_Costmap2D.md](06_Costmap2D.md) | nav2_costmap_2d/src/costmap_topic_collision_checker.cpp |
| `ClearCostmapService` | nav2_costmap_2d | ★★ | 进阶 | 清图服务三件套 | 清除全部/局部/指定区域 | [06_Costmap2D.md](06_Costmap2D.md) | nav2_costmap_2d/src/clear_costmap_service.cpp |
| `Costmap2DPublisher` | nav2_costmap_2d | ★★ | 进阶 | 栅格/更新增量发布器 | `publishCostmap` `updateBounds` | [06_Costmap2D.md](06_Costmap2D.md) | nav2_costmap_2d/src/costmap_2d_publisher.cpp |

## 五、全局规划域

| 类名 | 模块 | 重要程度 | 阅读顺序 | 主要成员 | 关键方法 | 关联章节 | 源码位置 |
|---|---|---|---|---|---|---|---|
| `PlannerServer` | nav2_planner | ★★★★★ | 第一阶段 | `planners_`（插件映射）、全局 Costmap 实例 | `computePlan` `computePlanThroughPoses` `getPlan` `transformPosesToGlobalFrame` `validatePath` | [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md) | nav2_planner/src/planner_server.cpp |
| `NavfnPlanner` | nav2_navfn_planner | ★★★★ | 第二阶段 | `planner_`（NavFn 实例）、`costmap_` | `createPlan` `makePlan` `getPlanFromPotential` `smoothApproachToGoal` | [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md)、[Examples/PlannerPlugin.md](Examples/PlannerPlugin.md) | nav2_navfn_planner/src/navfn_planner.cpp |
| `NavFn` | nav2_navfn_planner | ★★★ | 进阶 | 势场数组、优先队列缓冲 | `setNavArr` `setCostmap` `calcNavFnDijkstra` `calcNavFnAstar` `calcPath` | [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md) | nav2_navfn_planner/src/navfn.cpp |
| `SmacPlannerHybridT<NodeT>` | nav2_smac_planner | ★★★★ | 第二阶段 | `_a_star` `_smoother`、碰撞检查器 | `createPlan` | [08_SmacPlanner.md](08_SmacPlanner.md) | nav2_smac_planner/include/nav2_smac_planner/smac_planner_hybrid_impl.hpp |
| `SmacPlanner2DT<NodeT>` | nav2_smac_planner | ★★★ | 进阶 | `_a_star` `_smoother`、降采样器 | `createPlan` | [08_SmacPlanner.md](08_SmacPlanner.md) | nav2_smac_planner/include/nav2_smac_planner/smac_planner_2d_impl.hpp |
| `SmacPlannerLatticeT<NodeT>` | nav2_smac_planner | ★★★ | 进阶 | `_a_star` `_smoother`、lattice 基元表 | `createPlan` | [08_SmacPlanner.md](08_SmacPlanner.md) | nav2_smac_planner/include/nav2_smac_planner/smac_planner_lattice_impl.hpp |
| `AStarAlgorithm<NodeT>` | nav2_smac_planner | ★★★★ | 进阶 | `_graph`（节点图）、`_best_heuristic_node`、`_expander` | `setStart` `setGoal` `createPath` | [08_SmacPlanner.md](08_SmacPlanner.md) | nav2_smac_planner/include/nav2_smac_planner/a_star_impl.hpp |
| `NodeHybrid` | nav2_smac_planner | ★★★ | 进阶 | 运动基元、代价缓存 | `getNeighbors` `getTraversalCost` `backtracePath` | [08_SmacPlanner.md](08_SmacPlanner.md) | nav2_smac_planner/src/node_hybrid.cpp |
| `Node2D` | nav2_smac_planner | ★★ | 进阶 | 4/8 邻域 | `getNeighbors` `backtracePath` | [08_SmacPlanner.md](08_SmacPlanner.md) | nav2_smac_planner/src/node_2d.cpp |
| `NodeLattice` | nav2_smac_planner | ★★ | 进阶 | lattice 基元状态 | `getNeighbors` `backtracePath` | [08_SmacPlanner.md](08_SmacPlanner.md) | nav2_smac_planner/src/node_lattice.cpp |
| `AnalyticExpansion<NodeT>` | nav2_smac_planner | ★★★ | 进阶 | Dubins/RS 状态空间 | `tryAnalyticExpansion` | [08_SmacPlanner.md](08_SmacPlanner.md) | nav2_smac_planner/include/nav2_smac_planner/analytic_expansion_impl.hpp |
| `GridCollisionChecker` | nav2_smac_planner | ★★★ | 进阶 | footprint、代价阈值 | `inCollision` | [08_SmacPlanner.md](08_SmacPlanner.md) | nav2_smac_planner/include/nav2_smac_planner/collision_checker.hpp |
| `Smoother`（smac） | nav2_smac_planner | ★★ | 进阶 | 平滑参数 | `smooth` | [08_SmacPlanner.md](08_SmacPlanner.md) | nav2_smac_planner/src/smoother.cpp |
| `ThetaStarPlanner` | nav2_theta_star_planner | ★★ | 进阶 | `planner_`（ThetaStar 实例） | `createPlan` `linearInterpolation` | [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md) | nav2_theta_star_planner/src/theta_star_planner.cpp |
| `ThetaStar` | nav2_theta_star_planner | ★★ | 进阶 | 开放列表、视线检查 | `setStartAndGoal` `generatePath` | [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md) | nav2_theta_star_planner/src/theta_star.cpp |

## 六、局部控制域

| 类名 | 模块 | 重要程度 | 阅读顺序 | 主要成员 | 关键方法 | 关联章节 | 源码位置 |
|---|---|---|---|---|---|---|---|
| `ControllerServer` | nav2_controller | ★★★★★ | 第一阶段 | `controllers_` `progress_checkers_` `current_progress_checker_`、局部 Costmap | `computeControl` `computeAndPublishVelocity` `setPlannerPath` `updateGlobalPath` `isGoalReached` `publishVelocity` | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) | nav2_controller/src/controller_server.cpp |
| `SimpleGoalChecker` | nav2_controller | ★★★ | 第二阶段 | xy/yaw 容差 | `isGoalReached` | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) | nav2_controller/plugins/simple_goal_checker.cpp |
| `StoppedGoalChecker` | nav2_controller | ★★ | 进阶 | 速度阈值（继承 SimpleGoalChecker） | `isGoalReached` | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) | nav2_controller/plugins/stopped_goal_checker.cpp |
| `PositionGoalChecker` | nav2_controller | ★★ | 进阶 | 仅位置容差 | `isGoalReached` | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) | nav2_controller/plugins/position_goal_checker.cpp |
| `SimpleProgressChecker` | nav2_controller | ★★★ | 第二阶段 | 移动半径/超时窗口 | `check` | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) | nav2_controller/plugins/simple_progress_checker.cpp |
| `PoseProgressChecker` | nav2_controller | ★★ | 进阶 | 平移+角度双判据 | `check` | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) | nav2_controller/plugins/pose_progress_checker.cpp |
| `MPPIController` | nav2_mppi_controller | ★★★★ | 第二阶段 | `optimizer_` `trajectory_visualizer_` | `computeVelocityCommands` `visualize` | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md) | nav2_mppi_controller/src/controller.cpp |
| `Optimizer`（MPPI） | nav2_mppi_controller | ★★★★★ | 进阶 | `state_` `control_sequence_` `generated_trajectories_` `noise_generator_` `critic_manager_` `settings_` | `evalControl` `optimize` `prepare` `generateNoisedTrajectories` `updateControlSequence` `integrateStateVelocities` | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md) | nav2_mppi_controller/src/optimizer.cpp |
| `CriticManager` | nav2_mppi_controller | ★★★ | 进阶 | critic 插件列表 | `evalTrajectoriesScores` | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md) | nav2_mppi_controller/src/critic_manager.cpp |
| `CriticFunction` | nav2_mppi_controller | ★★★ | 进阶 | 权重/开关参数 | `score`（子类实现） | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md) | nav2_mppi_controller/include/nav2_mppi_controller/critic_function.hpp |
| `NoiseGenerator` | nav2_mppi_controller | ★★ | 进阶 | 噪声矩阵、后台线程 | `generateNextNoises` `setNoisedControls` | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md)、[20_线程模型.md](20_线程模型.md) | nav2_mppi_controller/src/noise_generator.cpp |
| `TrajectoryVisualizer` | nav2_mppi_controller | ★★ | 进阶 | Marker 发布器 | `visualize` | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md) | nav2_mppi_controller/src/trajectory_visualizer.cpp |
| `RegulatedPurePursuitController` | nav2_regulated_pure_pursuit_controller | ★★★★ | 第二阶段 | 前瞻距离参数、`collision_checker_` | `computeVelocityCommands` `shouldRotateToPath` `rotateToHeading` `applyConstraints` | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md)、[Examples/ControllerPlugin.md](Examples/ControllerPlugin.md) | nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp |
| `CollisionChecker`（RPP） | nav2_regulated_pure_pursuit_controller | ★★ | 进阶 | costmap/footprint 检查器 | `isCollisionImminent` | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md) | nav2_regulated_pure_pursuit_controller/src/collision_checker.cpp |
| `DWBLocalPlanner` | nav2_dwb_controller (dwb_core) | ★★★ | 进阶 | critic 插件列表、轨迹生成器 | `computeVelocityCommands` `coreScoringAlgorithm` `scoreTrajectory` | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) | nav2_dwb_controller/dwb_core/src/dwb_local_planner.cpp |
| `RotationShimController` | nav2_rotation_shim_controller | ★★★ | 进阶 | 被装饰的主控制器句柄 | `computeVelocityCommands` | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md)、[22_设计模式专题.md](22_设计模式专题.md) | nav2_rotation_shim_controller/src/nav2_rotation_shim_controller.cpp |
| `GracefulController` | nav2_graceful_controller | ★★ | 进阶 | 平滑控制律对象 | `computeVelocityCommands` | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) | nav2_graceful_controller/src/graceful_controller.cpp |
| `VelocitySmoother` | nav2_velocity_smoother | ★★★ | 第二阶段 | 定时器、上一周期指令 | `inputCommandCallback` `smootherTimer` `findEtaConstraint` | [16_CollisionMonitor与VelocitySmoother.md](16_CollisionMonitor与VelocitySmoother.md) | nav2_velocity_smoother/src/velocity_smoother.cpp |
| `CollisionMonitor` | nav2_collision_monitor | ★★★ | 第二阶段 | 多边形列表、数据源列表 | `cmdVelInCallbackStamped` `process` `processStopSlowdownLimit` `processApproach` | [16_CollisionMonitor与VelocitySmoother.md](16_CollisionMonitor与VelocitySmoother.md) | nav2_collision_monitor/src/collision_monitor_node.cpp |
| `Polygon`（CM） | nav2_collision_monitor | ★★★ | 进阶 | 顶点集、动作类型 | `getCollisionTime` | [16_CollisionMonitor与VelocitySmoother.md](16_CollisionMonitor与VelocitySmoother.md) | nav2_collision_monitor/src/polygon.cpp |

## 七、行为树与导航器

| 类名 | 模块 | 重要程度 | 阅读顺序 | 主要成员 | 关键方法 | 关联章节 | 源码位置 |
|---|---|---|---|---|---|---|---|
| `BtNavigator` | nav2_bt_navigator | ★★★★ | 第二阶段 | `navigators_` `class_loader_` | `on_configure` `on_activate` | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) | nav2_bt_navigator/src/bt_navigator.cpp |
| `NavigateToPoseNavigator` | nav2_bt_navigator | ★★★★ | 第二阶段 | 目标订阅、反馈计时 | `goalReceived` `onLoop` `onPreempt` `initializeGoalPose` `goalCompleted` | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) | nav2_bt_navigator/src/navigators/navigate_to_pose.cpp |
| `NavigateThroughPosesNavigator` | nav2_bt_navigator | ★★★ | 进阶 | 途经点集合 | `goalReceived` `onLoop` `onPreempt` | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) | nav2_bt_navigator/src/navigators/navigate_through_poses.cpp |
| `BtActionServer<ActionT>` | nav2_behavior_tree | ★★★★★ | 第一阶段 | `bt_`（引擎）、`tree_`、`default_bt_xml_filename_or_id_` | `on_configure` `on_activate` `loadBehaviorTree` `executeCallback` `haltTree` | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) | nav2_behavior_tree/include/nav2_behavior_tree/bt_action_server_impl.hpp |
| `BehaviorTreeEngine` | nav2_behavior_tree | ★★★★ | 第二阶段 | `factory_`（BT 工厂） | `run` `createTreeFromFile` `haltAllActions` | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) | nav2_behavior_tree/src/behavior_tree_engine.cpp |
| `BtActionNode<ActionT>` | nav2_behavior_tree | ★★★★★ | 第一阶段 | Action 客户端、`goal_updated_` | `tick` `on_tick` `on_wait_for_result` | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md)、[19_Action与通信机制.md](19_Action与通信机制.md) | nav2_behavior_tree/include/nav2_behavior_tree/bt_action_node.hpp |
| `BtServiceNode<ServiceT>` | nav2_behavior_tree | ★★★ | 第二阶段 | 服务客户端 | `tick` `on_tick` | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) | nav2_behavior_tree/include/nav2_behavior_tree/bt_service_node.hpp |
| `BtCancelActionNode<ActionT>` | nav2_behavior_tree | ★★ | 进阶 | Action 客户端 | `tick`（发取消请求） | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) | nav2_behavior_tree/include/nav2_behavior_tree/bt_cancel_action_node.hpp |
| `RosTopicLogger` | nav2_behavior_tree | ★★ | 进阶 | BT 状态变更发布器 | `callback` `flush` | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) | nav2_behavior_tree/include/nav2_behavior_tree/ros_topic_logger.hpp |

## 八、恢复行为与任务服务器

| 类名 | 模块 | 重要程度 | 阅读顺序 | 主要成员 | 关键方法 | 关联章节 | 源码位置 |
|---|---|---|---|---|---|---|---|
| `BehaviorServer` | nav2_behaviors | ★★★ | 第二阶段 | 行为插件列表 | `loadBehaviorPlugins` | [12_Behaviors恢复行为.md](12_Behaviors恢复行为.md) | nav2_behaviors/src/behavior_server.cpp |
| `TimedBehavior<ActionT>` | nav2_behaviors | ★★★★ | 第二阶段 | `action_server_`、周期频率 | `execute` `onRun` `onCycleUpdate` `onActionCompletion` `stopRobot` | [12_Behaviors恢复行为.md](12_Behaviors恢复行为.md) | nav2_behaviors/include/nav2_behaviors/timed_behavior.hpp |
| `Spin` | nav2_behaviors | ★★★ | 第二阶段 | 目标角度、已转角度 | `onRun` `onCycleUpdate` | [12_Behaviors恢复行为.md](12_Behaviors恢复行为.md) | nav2_behaviors/plugins/spin.cpp |
| `DriveOnHeading<ActionT>` | nav2_behaviors | ★★★ | 进阶 | 目标距离/速度 | `onRun` `onCycleUpdate` | [12_Behaviors恢复行为.md](12_Behaviors恢复行为.md) | nav2_behaviors/include/nav2_behaviors/plugins/drive_on_heading.hpp |
| `BackUp` | nav2_behaviors | ★★ | 进阶 | 继承 DriveOnHeading | `onRun`（取反方向） | [12_Behaviors恢复行为.md](12_Behaviors恢复行为.md) | nav2_behaviors/plugins/back_up.cpp |
| `Wait` | nav2_behaviors | ★★ | 进阶 | 剩余等待时长 | `onRun` `onCycleUpdate` | [12_Behaviors恢复行为.md](12_Behaviors恢复行为.md) | nav2_behaviors/plugins/wait.cpp |
| `AssistedTeleop` | nav2_behaviors | ★★ | 进阶 | 遥操作输入投影 | `onRun` `onCycleUpdate` | [12_Behaviors恢复行为.md](12_Behaviors恢复行为.md) | nav2_behaviors/plugins/assisted_teleop.cpp |
| `SmootherServer` | nav2_smoother | ★★★ | 第二阶段 | 平滑器插件映射 | `smoothPlan` | [15_Smoother路径平滑.md](15_Smoother路径平滑.md) | nav2_smoother/src/nav2_smoother.cpp |
| `SimpleSmoother` | nav2_smoother | ★★★ | 进阶 | 平滑权重/迭代上限 | `smooth` | [15_Smoother路径平滑.md](15_Smoother路径平滑.md) | nav2_smoother/src/simple_smoother.cpp |
| `SavitzkyGolaySmoother` | nav2_smoother | ★★ | 进阶 | SG 滤波窗口 | `smooth` | [15_Smoother路径平滑.md](15_Smoother路径平滑.md) | nav2_smoother/src/savitzky_golay_smoother.cpp |
| `WaypointFollower` | nav2_waypoint_follower | ★★★ | 第二阶段 | NavigateToPose 客户端、任务执行器插件 | `followWaypointsCallback` `followWaypointsHandler` `resultCallback` | [19_Action与通信机制.md](19_Action与通信机制.md) | nav2_waypoint_follower/src/waypoint_follower.cpp |
| `RouteServer` | nav2_route | ★★★ | 第二阶段 | `route_planner_`、图加载器、跟踪器 | `computeRoute` `findRoute` | [17_Route路线规划.md](17_Route路线规划.md) | nav2_route/src/route_server.cpp |
| `RoutePlanner` | nav2_route | ★★★ | 进阶 | 边评分器集合 | `findRoute` | [17_Route路线规划.md](17_Route路线规划.md) | nav2_route/src/route_planner.cpp |
| `RouteTracker` | nav2_route | ★★ | 进阶 | 路线进度状态 | `trackRoute` | [17_Route路线规划.md](17_Route路线规划.md) | nav2_route/src/route_tracker.cpp |
| `GoalIntentExtractor` | nav2_route | ★★ | 进阶 | 空间索引树 | `findStartandGoal` | [17_Route路线规划.md](17_Route路线规划.md) | nav2_route/src/goal_intent_extractor.cpp |
| `DockingServer` | nav2_docking (opennav_docking) | ★★★ | 第二阶段 | dock 数据库、导航器/控制器子对象 | `dockRobot` `undockRobot` `approachDock` `rotateToDock` | [18_Docking靠泊.md](18_Docking靠泊.md) | nav2_docking/opennav_docking/src/docking_server.cpp |

## 九、定位与地图

| 类名 | 模块 | 重要程度 | 阅读顺序 | 主要成员 | 关键方法 | 关联章节 | 源码位置 |
|---|---|---|---|---|---|---|---|
| `AmclNode` | nav2_amcl | ★★★★ | 第二阶段 | `pf_`（粒子滤波器）、激光/运动模型 | `laserReceived` `initialPoseReceived` `handleInitialPose` | [13_AMCL定位.md](13_AMCL定位.md) | nav2_amcl/src/amcl_node.cpp |
| `pf_t`（C 结构） | nav2_amcl | ★★★ | 进阶 | 双粒子集、KLD 参数 | `pf_update_action` `pf_update_sensor` `pf_update_resample` | [13_AMCL定位.md](13_AMCL定位.md) | nav2_amcl/src/pf/pf.c |
| `Laser` / `LikelihoodFieldModel` | nav2_amcl | ★★★ | 进阶 | 似然场参数 | `sensorUpdate` | [13_AMCL定位.md](13_AMCL定位.md) | nav2_amcl/include/nav2_amcl/sensors/laser/laser.hpp |
| `DifferentialMotionModel` | nav2_amcl | ★★ | 进阶 | odom 噪声参数 alpha1~4 | `odometryUpdate` | [13_AMCL定位.md](13_AMCL定位.md) | nav2_amcl/include/nav2_amcl/motion_model/differential_motion_model.hpp |
| `MapServer` | nav2_map_server | ★★★ | 第二阶段 | 地图消息缓存、加载服务 | `on_configure` `getMapCallback` `loadMapCallback` | [14_MapServer.md](14_MapServer.md) | nav2_map_server/src/map_server/map_server.cpp |
| `MapSaver` | nav2_map_server | ★★ | 进阶 | 保存参数 | `saveMapTopicToFile` | [14_MapServer.md](14_MapServer.md) | nav2_map_server/src/map_saver/map_saver.cpp |
