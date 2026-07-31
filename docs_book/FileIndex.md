# 文件索引（FileIndex）

> 口径：Nav2 1.5.0（main 分支）、C++20。行数为写作时现场 `wc` 统计结果，随上游演进会有出入，均标注"约"。
> 配套索引：[ClassIndex.md](ClassIndex.md)（类索引）、[APIIndex.md](APIIndex.md)（函数级双向索引）。

## 使用说明

- 路径均为仓库相对路径；点击"关联章节"可跳转对应书稿章节。
- 本表只收录理解 Nav2 架构与热路径所必需的约 55 个核心文件，完整目录结构见 [02_目录设计.md](02_目录设计.md)。

## 一、基础设施（nav2_ros_common / nav2_util / nav2_core）

| 路径 | 所属包 | 约行数 | 主类/主内容 | 一句话职责 | 关联章节 |
|---|---|---|---|---|---|
| nav2_ros_common/include/nav2_ros_common/lifecycle_node.hpp | nav2_ros_common | 约 475 行 | `nav2::LifecycleNode` | 所有 Nav2 服务器的生命周期节点基类，封装 bond 心跳与参数接口 | [05_生命周期管理.md](05_生命周期管理.md)、[CheatSheet/Lifecycle.md](CheatSheet/Lifecycle.md) |
| nav2_ros_common/include/nav2_ros_common/simple_action_server.hpp | nav2_ros_common | 约 682 行 | `nav2::SimpleActionServer<ActionT>` | 单目标+抢占语义的 Action 服务器封装，所有服务器的任务入口 | [19_Action与通信机制.md](19_Action与通信机制.md)、[CheatSheet/Action.md](CheatSheet/Action.md) |
| nav2_ros_common/include/nav2_ros_common/node_utils.hpp | nav2_ros_common | 约 455 行 | `declare_parameter_if_not_declared` 等 | 参数声明/节点名处理等节点级工具函数集 | [21_配置系统.md](21_配置系统.md) |
| nav2_ros_common/include/nav2_ros_common/service_client.hpp | nav2_ros_common | 约 246 行 | `nav2::ServiceClient<ServiceT>` | 带独立 executor 的同步服务客户端封装 | [19_Action与通信机制.md](19_Action与通信机制.md) |
| nav2_util/src/robot_utils.cpp | nav2_util | 约 214 行 | `getCurrentPose` / `transformPoseInTargetFrame` | TF 位姿查询与坐标系变换的全局工具函数 | [20_线程模型.md](20_线程模型.md) |
| nav2_util/include/nav2_util/odometry_utils.hpp | nav2_util | 约 166 行 | `OdomSmoother` | 里程计滑窗平滑，为控制/行为提供当前速度 | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) |
| nav2_util/include/nav2_util/geometry_utils.hpp | nav2_util | 约 310 行 | `euclidean_distance` 等 | 距离/朝向/路径长度等几何工具函数 | [23_现代C++专题.md](23_现代C++专题.md) |
| nav2_util/include/nav2_util/twist_publisher.hpp | nav2_util | 约 129 行 | `TwistPublisher` | 统一发布 Twist/TwistStamped 两种速度消息 | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) |
| nav2_core/include/nav2_core/global_planner.hpp | nav2_core | 约 87 行 | `nav2_core::GlobalPlanner` | 全局规划器插件纯虚接口 | [04_nav2_core插件接口.md](04_nav2_core插件接口.md)、[CheatSheet/Planner.md](CheatSheet/Planner.md) |
| nav2_core/include/nav2_core/controller.hpp | nav2_core | 约 152 行 | `nav2_core::Controller` | 局部控制器插件纯虚接口 | [04_nav2_core插件接口.md](04_nav2_core插件接口.md)、[CheatSheet/Controller.md](CheatSheet/Controller.md) |
| nav2_core/include/nav2_core/behavior_tree_navigator.hpp | nav2_core | 约 436 行 | `BehaviorTreeNavigator<ActionT>` | 导航器插件基类，桥接 BtActionServer 与具体导航任务 | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) |

## 二、生命周期管理（nav2_lifecycle_manager）

| 路径 | 所属包 | 约行数 | 主类 | 一句话职责 | 关联章节 |
|---|---|---|---|---|---|
| nav2_lifecycle_manager/src/lifecycle_manager.cpp | nav2_lifecycle_manager | 约 632 行 | `LifecycleManager` | 编排全部服务器的生命周期状态迁移并用 bond 监活 | [05_生命周期管理.md](05_生命周期管理.md)、[Examples/LifecycleNode.md](Examples/LifecycleNode.md) |

## 三、Costmap（nav2_costmap_2d）

| 路径 | 所属包 | 约行数 | 主类 | 一句话职责 | 关联章节 |
|---|---|---|---|---|---|
| nav2_costmap_2d/src/costmap_2d_ros.cpp | nav2_costmap_2d | 约 946 行 | `Costmap2DROS` | Costmap 的 ROS 生命周期节点外壳，管理更新线程与插件 | [06_Costmap2D.md](06_Costmap2D.md)、[CheatSheet/Costmap.md](CheatSheet/Costmap.md) |
| nav2_costmap_2d/src/costmap_2d.cpp | nav2_costmap_2d | 约 603 行 | `Costmap2D` | 纯数据结构：栅格代价数组及坐标换算/几何填充 | [06_Costmap2D.md](06_Costmap2D.md) |
| nav2_costmap_2d/src/layered_costmap.cpp | nav2_costmap_2d | 约 323 行 | `LayeredCostmap` | 分层聚合器，按序调用各层 updateBounds/updateCosts | [06_Costmap2D.md](06_Costmap2D.md) |
| nav2_costmap_2d/src/costmap_layer.cpp | nav2_costmap_2d | 约 282 行 | `CostmapLayer` | 带私有栅格的图层基类，提供 updateWithMax 等合并策略 | [06_Costmap2D.md](06_Costmap2D.md)、[Examples/CostmapLayer.md](Examples/CostmapLayer.md) |
| nav2_costmap_2d/src/observation_buffer.cpp | nav2_costmap_2d | 约 220 行 | `ObservationBuffer` | 缓存传感器点云观测并做 TF 变换与超时管理 | [06_Costmap2D.md](06_Costmap2D.md) |
| nav2_costmap_2d/src/footprint_collision_checker.cpp | nav2_costmap_2d | 约 150 行 | `FootprintCollisionChecker` | 按机器人footprint 在代价地图上做碰撞打分 | [06_Costmap2D.md](06_Costmap2D.md) |
| nav2_costmap_2d/plugins/static_layer.cpp | nav2_costmap_2d | 约 593 行 | `StaticLayer` | 订阅静态地图并写入代价层 | [06_Costmap2D.md](06_Costmap2D.md) |
| nav2_costmap_2d/plugins/obstacle_layer.cpp | nav2_costmap_2d | 约 876 行 | `ObstacleLayer` | 融合激光/点云障碍观测，标记与射线清除 | [06_Costmap2D.md](06_Costmap2D.md) |
| nav2_costmap_2d/plugins/voxel_layer.cpp | nav2_costmap_2d | 约 615 行 | `VoxelLayer` | 3D 体素网格障碍层，向下投影为 2D 代价 | [06_Costmap2D.md](06_Costmap2D.md) |
| nav2_costmap_2d/plugins/inflation_layer.cpp | nav2_costmap_2d | 约 494 行 | `InflationLayer` | 由障碍向外指数衰减膨胀代价 | [06_Costmap2D.md](06_Costmap2D.md) |
| nav2_costmap_2d/plugins/costmap_filters/keepout_filter.cpp | nav2_costmap_2d | 约 421 行 | `KeepoutFilter` | 基于过滤掩码地图的禁行区代价过滤器 | [06_Costmap2D.md](06_Costmap2D.md) |

## 四、全局规划（nav2_planner 及规划器插件）

| 路径 | 所属包 | 约行数 | 主类 | 一句话职责 | 关联章节 |
|---|---|---|---|---|---|
| nav2_planner/src/planner_server.cpp | nav2_planner | 约 694 行 | `PlannerServer` | 规划服务器：Action 入口、插件调度、路径校验 | [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md) |
| nav2_planner/include/nav2_planner/planner_server.hpp | nav2_planner | 约 247 行 | `PlannerServer` | 规划服务器类声明与成员定义 | [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md) |
| nav2_navfn_planner/src/navfn_planner.cpp | nav2_navfn_planner | 约 530 行 | `NavfnPlanner` | NavFn 插件外壳：把 createPlan 请求翻译成势场计算 | [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md)、[Examples/PlannerPlugin.md](Examples/PlannerPlugin.md) |
| nav2_navfn_planner/src/navfn.cpp | nav2_navfn_planner | 约 1035 行 | `NavFn` | Dijkstra/A* 势场算法本体（传承自 ROS1） | [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md) |
| nav2_smac_planner/include/nav2_smac_planner/smac_planner_hybrid_impl.hpp | nav2_smac_planner | 约 912 行 | `SmacPlannerHybridT<NodeT>` | Hybrid-A* 插件实现（模板实现文件） | [08_SmacPlanner.md](08_SmacPlanner.md) |
| nav2_smac_planner/include/nav2_smac_planner/a_star_impl.hpp | nav2_smac_planner | 约 592 行 | `AStarAlgorithm<NodeT>` | 模板化 A* 主搜索循环，Smac 三种规划器共用 | [08_SmacPlanner.md](08_SmacPlanner.md) |
| nav2_smac_planner/include/nav2_smac_planner/analytic_expansion_impl.hpp | nav2_smac_planner | 约 494 行 | `AnalyticExpansion<NodeT>` | Dubins/Reeds-Shepp 解析扩展提前命中目标 | [08_SmacPlanner.md](08_SmacPlanner.md) |
| nav2_smac_planner/src/node_hybrid.cpp | nav2_smac_planner | 约 540 行 | `NodeHybrid` | Hybrid-A* 搜索节点：运动基元、代价与启发式 | [08_SmacPlanner.md](08_SmacPlanner.md) |
| nav2_smac_planner/src/smoother.cpp | nav2_smac_planner | 约 459 行 | `Smoother`（smac） | Smac 输出路径的梯度平滑后处理 | [08_SmacPlanner.md](08_SmacPlanner.md) |
| nav2_theta_star_planner/src/theta_star_planner.cpp | nav2_theta_star_planner | 约 217 行 | `ThetaStarPlanner` | Theta* 任意角度规划器插件外壳 | [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md) |

## 五、局部控制（nav2_controller 及控制器插件）

| 路径 | 所属包 | 约行数 | 主类 | 一句话职责 | 关联章节 |
|---|---|---|---|---|---|
| nav2_controller/src/controller_server.cpp | nav2_controller | 约 946 行 | `ControllerServer` | 控制服务器：FollowPath 循环、进度/到点检查调度 | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) |
| nav2_controller/plugins/simple_goal_checker.cpp | nav2_controller | 约 273 行 | `SimpleGoalChecker` | 按 xy/yaw 容差判断是否到达目标 | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) |
| nav2_controller/plugins/simple_progress_checker.cpp | nav2_controller | 约 157 行 | `SimpleProgressChecker` | 按时间窗内移动距离判断是否卡死 | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) |
| nav2_mppi_controller/src/controller.cpp | nav2_mppi_controller | 约 172 行 | `MPPIController` | MPPI 插件外壳，把控制请求转交 Optimizer | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md) |
| nav2_mppi_controller/src/optimizer.cpp | nav2_mppi_controller | 约 726 行 | `Optimizer` | MPPI 采样优化主循环（生成轨迹→评分→更新控制序列） | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md) |
| nav2_mppi_controller/src/critic_manager.cpp | nav2_mppi_controller | 约 123 行 | `CriticManager` | 加载并逐一执行 critic 插件为轨迹打分 | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md) |
| nav2_mppi_controller/src/noise_generator.cpp | nav2_mppi_controller | 约 121 行 | `NoiseGenerator` | 后台线程预生成控制噪声样本 | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md)、[20_线程模型.md](20_线程模型.md) |
| nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp | nav2_regulated_pure_pursuit_controller | 约 439 行 | `RegulatedPurePursuitController` | 带曲率/接近度调速的纯追踪控制器 | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md)、[Examples/ControllerPlugin.md](Examples/ControllerPlugin.md) |
| nav2_rotation_shim_controller/src/nav2_rotation_shim_controller.cpp | nav2_rotation_shim_controller | 约 360 行 | `RotationShimController` | 先原地转向对准路径再移交主控制器的装饰器 | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md)、[22_设计模式专题.md](22_设计模式专题.md) |
| nav2_graceful_controller/src/graceful_controller.cpp | nav2_graceful_controller | 约 639 行 | `GracefulController` | 基于平滑控制律的姿态收敛控制器 | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) |
| nav2_dwb_controller/dwb_core/src/dwb_local_planner.cpp | nav2_dwb_controller (dwb_core) | 约 402 行 | `DWBLocalPlanner` | DWB 动态窗口控制器：轨迹采样+critic 评分 | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) |

## 六、行为树与导航器（nav2_bt_navigator / nav2_behavior_tree）

| 路径 | 所属包 | 约行数 | 主类 | 一句话职责 | 关联章节 |
|---|---|---|---|---|---|
| nav2_bt_navigator/src/bt_navigator.cpp | nav2_bt_navigator | 约 196 行 | `BtNavigator` | 装载导航器插件的生命周期壳节点 | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) |
| nav2_bt_navigator/src/navigators/navigate_to_pose.cpp | nav2_bt_navigator | 约 292 行 | `NavigateToPoseNavigator` | 点到点导航任务：目标接收、反馈回路、抢占 | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) |
| nav2_bt_navigator/src/navigators/navigate_through_poses.cpp | nav2_bt_navigator | 约 321 行 | `NavigateThroughPosesNavigator` | 多途经点导航任务实现 | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) |
| nav2_behavior_tree/include/nav2_behavior_tree/bt_action_server_impl.hpp | nav2_behavior_tree | 约 553 行 | `BtActionServer<ActionT>` | BT 承载器：加载 XML、驱动 tick 循环（模板实现） | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) |
| nav2_behavior_tree/include/nav2_behavior_tree/bt_action_node.hpp | nav2_behavior_tree | 约 509 行 | `BtActionNode<ActionT>` | BT 叶节点基类：把 ROS Action 客户端包装成 BT 节点 | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md)、[19_Action与通信机制.md](19_Action与通信机制.md) |
| nav2_behavior_tree/include/nav2_behavior_tree/bt_service_node.hpp | nav2_behavior_tree | 约 309 行 | `BtServiceNode<ServiceT>` | BT 叶节点基类：包装 ROS Service 客户端 | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) |
| nav2_behavior_tree/src/behavior_tree_engine.cpp | nav2_behavior_tree | 约 218 行 | `BehaviorTreeEngine` | BT 工厂与 run 循环（tickOnce + onLoop + 取消检查） | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) |

## 七、恢复行为与其他任务服务器

| 路径 | 所属包 | 约行数 | 主类 | 一句话职责 | 关联章节 |
|---|---|---|---|---|---|
| nav2_behaviors/include/nav2_behaviors/timed_behavior.hpp | nav2_behaviors | 约 312 行 | `TimedBehavior<ActionT>` | 恢复行为模板基类：onRun + 周期 onCycleUpdate 骨架 | [12_Behaviors恢复行为.md](12_Behaviors恢复行为.md) |
| nav2_behaviors/src/behavior_server.cpp | nav2_behaviors | 约 238 行 | `BehaviorServer` | 加载 Spin/BackUp/Wait 等行为插件的宿主 | [12_Behaviors恢复行为.md](12_Behaviors恢复行为.md) |
| nav2_behaviors/plugins/spin.cpp | nav2_behaviors | 约 184 行 | `Spin` | 原地旋转恢复行为（含碰撞前瞻） | [12_Behaviors恢复行为.md](12_Behaviors恢复行为.md) |
| nav2_smoother/src/nav2_smoother.cpp | nav2_smoother | 约 372 行 | `SmootherServer` | 路径平滑服务器：SmoothPath Action 入口 | [15_Smoother路径平滑.md](15_Smoother路径平滑.md) |
| nav2_smoother/src/simple_smoother.cpp | nav2_smoother | 约 225 行 | `SimpleSmoother` | 迭代加权平均的默认平滑器插件 | [15_Smoother路径平滑.md](15_Smoother路径平滑.md) |
| nav2_velocity_smoother/src/velocity_smoother.cpp | nav2_velocity_smoother | 约 636 行 | `VelocitySmoother` | 对 cmd_vel 做加减速约束平滑的定时器节点 | [16_CollisionMonitor与VelocitySmoother.md](16_CollisionMonitor与VelocitySmoother.md) |
| nav2_collision_monitor/src/collision_monitor_node.cpp | nav2_collision_monitor | 约 774 行 | `CollisionMonitor` | 速度指令安全闸：按多边形区域停止/减速/限速 | [16_CollisionMonitor与VelocitySmoother.md](16_CollisionMonitor与VelocitySmoother.md) |
| nav2_collision_monitor/src/polygon.cpp | nav2_collision_monitor | 约 718 行 | `Polygon` | 监测多边形：点包含判断与碰撞时间估计 | [16_CollisionMonitor与VelocitySmoother.md](16_CollisionMonitor与VelocitySmoother.md) |
| nav2_waypoint_follower/src/waypoint_follower.cpp | nav2_waypoint_follower | 约 486 行 | `WaypointFollower` | 逐个下发 NavigateToPose 目标并执行途经点任务 | [19_Action与通信机制.md](19_Action与通信机制.md) |
| nav2_route/src/route_server.cpp | nav2_route | 约 412 行 | `RouteServer` | 路网路线服务器：ComputeRoute/ComputeAndTrackRoute | [17_Route路线规划.md](17_Route路线规划.md) |
| nav2_route/src/route_planner.cpp | nav2_route | 约 217 行 | `RoutePlanner` | 路网图上的 Dijkstra 式最优路线搜索 | [17_Route路线规划.md](17_Route路线规划.md) |
| nav2_docking/opennav_docking/src/docking_server.cpp | nav2_docking (opennav_docking) | 约 803 行 | `DockingServer` | 靠泊服务器：DockRobot/UndockRobot 状态机 | [18_Docking靠泊.md](18_Docking靠泊.md) |

## 八、定位与地图（nav2_amcl / nav2_map_server）

| 路径 | 所属包 | 约行数 | 主类 | 一句话职责 | 关联章节 |
|---|---|---|---|---|---|
| nav2_amcl/src/amcl_node.cpp | nav2_amcl | 约 1650 行 | `AmclNode` | AMCL 主节点：激光回调驱动粒子滤波并发布 map→odom | [13_AMCL定位.md](13_AMCL定位.md) |
| nav2_amcl/src/pf/pf.c | nav2_amcl | 约 645 行 | `pf_t`（C） | 粒子滤波核心：采样、更新、KLD 自适应重采样 | [13_AMCL定位.md](13_AMCL定位.md) |
| nav2_map_server/src/map_server/map_server.cpp | nav2_map_server | 约 256 行 | `MapServer` | 地图服务器：加载 YAML 地图并提供话题/服务 | [14_MapServer.md](14_MapServer.md) |
| nav2_map_server/src/map_io.cpp | nav2_map_server | 约 673 行 | `map_io` 自由函数 | 地图 YAML/图片文件的读写与格式转换 | [14_MapServer.md](14_MapServer.md) |
