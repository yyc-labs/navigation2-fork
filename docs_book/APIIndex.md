# 函数级双向索引（APIIndex）

> 口径：Nav2 1.5.0（main 分支）、C++20。定义位置 = 仓库相对路径 + 函数名锚点（"约 N 行"处，随上游演进会漂移，请以函数名搜索为准）。
> Caller / Callee 均为**直接调用（1 跳）**，不做递归展开；带 ⚡ 的为控制/规划热路径函数，线程与阻塞信息见下方「热路径函数线程档案」。
> 配套索引：[ClassIndex.md](ClassIndex.md)（类索引）、[FileIndex.md](FileIndex.md)（文件索引）。

## 使用说明

- **正向查**：已知函数名，查它调用了谁（Callee 列）。
- **反向查**：已知函数名，查谁调用了它（Caller 列）；也可全文搜索函数名，在其他条目的 Callee 列中反查。
- Caller 只列生产代码中的直接调用点，测试代码一律不收录。

## 热路径函数线程档案 ⚡

| 函数 | 线程安全性 | 是否阻塞 | 由哪个线程调用 |
|---|---|---|---|
| `SimpleActionServer::work` | 内部以 `update_mutex_`（递归锁）保护目标句柄 | 阻塞：循环执行直至无待处理目标 | `handle_accepted` 经 `std::async` 派生的专用工作线程 |
| `PlannerServer::computePlan` | 非线程安全；内部依赖全局 costmap 锁 | 阻塞：直至规划完成/失败/被抢占 | ComputePathToPose Action 工作线程（SimpleActionServer） |
| `PlannerServer::computePlanThroughPoses` | 同上 | 阻塞：逐段规划直至全部完成 | ComputePathThroughPoses Action 工作线程 |
| `AStarAlgorithm::createPath` | 非线程安全（独占 `_graph`） | 阻塞：搜索直至命中/迭代上限/超时 | 规划器 Action 工作线程内同步执行 |
| `ControllerServer::computeControl` | 非线程安全；每周期对 costmap 加锁 | 阻塞：while 循环直至到达/取消/失败 | FollowPath Action 工作线程（SimpleActionServer） |
| `ControllerServer::computeAndPublishVelocity` | 调用时持有 costmap 锁 | 单周期非阻塞（受控制频率节拍约束） | 同上（computeControl 循环体内） |
| `MPPIController::computeVelocityCommands` | 非线程安全 | 单周期阻塞式计算（毫秒级） | FollowPath Action 工作线程 |
| `Optimizer::evalControl` | 非线程安全 | 阻塞：完成全部迭代后返回 | 控制器执行线程内同步调用 |
| `Optimizer::optimize` | 非线程安全 | 阻塞：iteration_count 轮采样-评分-更新 | 同上；噪声由 NoiseGenerator 后台线程预生成 |
| `RegulatedPurePursuitController::computeVelocityCommands` | 非线程安全；碰撞检查读 costmap | 单周期非阻塞 | FollowPath Action 工作线程 |
| `Costmap2DROS::mapUpdateLoop` | 循环体内对 costmap 加锁 | 阻塞：常驻循环直至节点 deactivate | 专属 `map_update_thread_`（on_activate 创建） |
| `Costmap2DROS::updateMap` | 内部持 `Costmap2D` mutex | 单次调用非阻塞（更新耗时与图幅相关） | `map_update_thread_`；参数动态更新回调也会调用 |
| `LayeredCostmap::updateMap` | 非线程安全，调用方须先持 costmap 锁 | 单次非阻塞 | `map_update_thread_`（经 Costmap2DROS::updateMap） |
| `BtActionServer::executeCallback` | 非线程安全 | 阻塞：内部 `run` 循环直至 BT 返回终态 | 导航 Action 工作线程（SimpleActionServer） |
| `BehaviorTreeEngine::run` | 非线程安全 | 阻塞：按 loop_rate 循环 tick 直至终态/取消 | 调用方线程（即 executeCallback 所在工作线程） |
| `VelocitySmoother::smootherTimer` | 与订阅回调共享最近指令（同一回调组串行） | 单次非阻塞 | ROS 定时器回调线程（executor） |
| `CollisionMonitor::process` | 数据源读取在回调线程内串行 | 单次非阻塞 | cmd_vel 订阅回调线程（executor） |
| `AmclNode::laserReceived` | 非线程安全；与初始位姿回调共享互斥锁 | 单次调用可能耗时（滤波+重采样） | 激光消息订阅回调线程（executor） |

## 一、生命周期域

| 函数 | 一句话职责 | 定义位置 | Caller（1 跳） | Callee（关键 1 跳） | 关联章节 |
|---|---|---|---|---|---|
| `LifecycleManager::startup` | 按序 configure→activate 全部受管节点 | nav2_lifecycle_manager/src/lifecycle_manager.cpp（`startup` 锚点，约 367 行处） | manage_nodes 服务回调（构造中绑定） | `changeStateForAllNodes`（两次） | [05_生命周期管理.md](05_生命周期管理.md) |
| `LifecycleManager::shutdown` | 逆序 deactivate→cleanup→shutdown 全部节点 | 同上文件（`shutdown` 锚点，约 412 行处） | manage_nodes 服务回调、rcl 预关闭回调 | `shutdownAllNodes`→`changeStateForAllNodes`、`destroyBondTimer` | [05_生命周期管理.md](05_生命周期管理.md) |
| `LifecycleManager::reset` | 逆序回退到 unconfigured 状态 | 同上文件（`reset` 锚点，约 425 行处） | manage_nodes 服务回调 | `changeStateForAllNodes` | [05_生命周期管理.md](05_生命周期管理.md) |
| `LifecycleManager::changeStateForAllNodes` | 对节点列表逐一（正/逆序）做状态迁移 | 同上文件（`changeStateForAllNodes` 锚点，约 320 行处） | `startup` `shutdownAllNodes` `reset` `pause` `resume` | `changeStateForNode` | [05_生命周期管理.md](05_生命周期管理.md) |
| `LifecycleManager::createBondConnection` | 与目标节点建立 bond 心跳连接 | 同上文件（`createBondConnection` 锚点，约 267 行处） | `changeStateForNode`（activate 成功后） | bond 库 `start` | [05_生命周期管理.md](05_生命周期管理.md) |
| `LifecycleManager::checkBondConnections` | 定时巡检 bond，发现掉线即触发重置/关停 | 同上文件（`checkBondConnections` 锚点，约 534 行处) | bond 定时器回调（`createBondTimer` 中绑定） | `reset`（按 bond_respawn 配置） | [05_生命周期管理.md](05_生命周期管理.md)、[20_线程模型.md](20_线程模型.md) |
| `nav2::LifecycleNode::createBond` | 服务器侧创建 bond 心跳（与管理器对接） | nav2_ros_common/include/nav2_ros_common/lifecycle_node.hpp（`createBond` 锚点，约 366 行处） | 各服务器 `on_activate` | bond 库 `start` | [05_生命周期管理.md](05_生命周期管理.md)、[CheatSheet/Lifecycle.md](CheatSheet/Lifecycle.md) |
| `nav2::LifecycleNode::destroyBond` | 断开并销毁 bond 连接 | 同上文件（`destroyBond` 锚点，约 385 行处） | 各服务器 `on_deactivate`、`runCleanups` | bond 复位 | [05_生命周期管理.md](05_生命周期管理.md) |
| `LifecycleManagerClient::startup` | 客户端封装：调用 manage_nodes 服务发起 STARTUP | nav2_lifecycle_manager/src/lifecycle_manager_client.cpp（`startup` 锚点，约 29 行处） | 应用/测试代码 | `callService` | [05_生命周期管理.md](05_生命周期管理.md)、[Examples/LifecycleNode.md](Examples/LifecycleNode.md) |

## 二、Action 与工具基础设施

| 函数 | 一句话职责 | 定义位置 | Caller（1 跳） | Callee（关键 1 跳） | 关联章节 |
|---|---|---|---|---|---|
| `SimpleActionServer::handle_goal` | 目标准入检查（服务器激活才收） | nav2_ros_common/include/nav2_ros_common/simple_action_server.hpp（`handle_goal` 锚点，约 155 行处） | rclcpp_action 服务器（构造时注册） | — | [19_Action与通信机制.md](19_Action与通信机制.md)、[CheatSheet/Action.md](CheatSheet/Action.md) |
| `SimpleActionServer::handle_accepted` | 新目标入位：有活跃目标则挂 pending（抢占），否则 async 启动工作线程 | 同上文件（`handle_accepted` 锚点，约 209 行处） | rclcpp_action 服务器 | `terminate`、`std::async`→`work` | [19_Action与通信机制.md](19_Action与通信机制.md)、[Examples/ActionServer.md](Examples/ActionServer.md) |
| ⚡ `SimpleActionServer::work` | 工作线程主循环：执行用户回调并消化 pending 目标 | 同上文件（`work` 锚点，约 248 行处） | `handle_accepted`（经 std::async） | `execute_callback_`、`accept_pending_goal`、`terminate_all` | [19_Action与通信机制.md](19_Action与通信机制.md)、[20_线程模型.md](20_线程模型.md) |
| `SimpleActionServer::is_preempt_requested` | 查询是否有 pending 目标等待抢占 | 同上文件（`is_preempt_requested` 锚点，约 362 行处） | `PlannerServer::computePlan*`、`ControllerServer::computeControl` 等各服务器循环 | — | [19_Action与通信机制.md](19_Action与通信机制.md) |
| `SimpleActionServer::accept_pending_goal` | 将 pending 目标切换为当前目标 | 同上文件（`accept_pending_goal` 锚点，约 372 行处） | `work`、各服务器抢占分支 | `terminate` | [19_Action与通信机制.md](19_Action与通信机制.md) |
| `SimpleActionServer::terminate_current` | 以 abort/cancel 终结当前目标 | 同上文件（`terminate_current` 锚点，约 497 行处） | 各服务器异常/取消分支 | `terminate` | [19_Action与通信机制.md](19_Action与通信机制.md) |
| `SimpleActionServer::succeeded_current` | 以成功状态终结当前目标并回传结果 | 同上文件（`succeeded_current` 锚点，约 509 行处） | 各服务器成功分支 | 目标句柄 `succeed` | [19_Action与通信机制.md](19_Action与通信机制.md) |
| `nav2_util::getCurrentPose` | 查询机器人当前位姿（TF：global←robot） | nav2_util/src/robot_utils.cpp（`getCurrentPose` 锚点，约 32 行处） | `Costmap2DROS::getRobotPose`、`Spin::onRun`/`onCycleUpdate`、`BackUp::onRun`、`DriveOnHeading`、`NavigateToPoseNavigator::onLoop`/`initializeGoalPose`、`NavigateThroughPosesNavigator`、`RouteTracker`、`GoalIntentExtractor`、`FootprintSubscriber`、`AssistedTeleop` | `transformPoseInTargetFrame` | [20_线程模型.md](20_线程模型.md) |
| `nav2_util::transformPoseInTargetFrame` | 把任意位姿变换到目标坐标系（带超时） | 同上文件（`transformPoseInTargetFrame` 锚点，约 46 行处） | `getCurrentPose`、`nav2_util` path_utils 内 `getCurrentPath` 相关工具 | tf2 `transform` | [20_线程模型.md](20_线程模型.md) |
| `nav2_util::isPathUpdated` | 判断新路径相对旧路径是否真正更新 | nav2_util/src/path_utils.cpp（`isPathUpdated` 锚点，约 223 行处） | `NavigateToPoseNavigator::onLoop`、`NavigateThroughPosesNavigator::onLoop`、BT 节点 `PathLongerOnApproach` | — | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) |

## 三、全局规划域

| 函数 | 一句话职责 | 定义位置 | Caller（1 跳） | Callee（关键 1 跳） | 关联章节 |
|---|---|---|---|---|---|
| ⚡ `PlannerServer::computePlan` | ComputePathToPose Action 回调：单目标规划全流程 | nav2_planner/src/planner_server.cpp（`computePlan` 锚点，约 511 行处） | SimpleActionServer 工作线程（on_configure 中绑定） | `transformPosesToGlobalFrame`、`getPlan`、`validatePath` | [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md) |
| ⚡ `PlannerServer::computePlanThroughPoses` | ComputePathThroughPoses 回调：多途经点逐段规划拼接 | 同上文件（`computePlanThroughPoses` 锚点，约 338 行处） | SimpleActionServer 工作线程 | `transformPosesToGlobalFrame`、`getPlan`、`validatePath` | [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md) |
| `PlannerServer::getPlan` | 按 planner_id 分发到对应规划器插件 | 同上文件（`getPlan` 锚点，约 616 行处） | `computePlan`、`computePlanThroughPoses` | `planners_[planner_id]->createPlan` | [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md)、[CheatSheet/Planner.md](CheatSheet/Planner.md) |
| `PlannerServer::transformPosesToGlobalFrame` | 把起点/终点变换到全局代价地图坐标系 | 同上文件（`transformPosesToGlobalFrame` 锚点，约 302 行处） | `computePlan`、`computePlanThroughPoses` | `costmap_ros_->transformPoseToGlobalFrame` | [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md) |
| `PlannerServer::validatePath` | 校验规划结果非空且抵达目标容差内 | 同上文件（`validatePath` 锚点，约 316 行处） | `computePlan`、`computePlanThroughPoses` | 几何距离计算 | [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md) |
| `NavfnPlanner::createPlan` | NavFn 插件入口：校验起终点后转交 makePlan | nav2_navfn_planner/src/navfn_planner.cpp（`createPlan` 锚点，约 121 行处） | `PlannerServer::getPlan`（经插件基类指针） | `NavFn::setNavArr`、`makePlan` | [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md)、[Examples/PlannerPlugin.md](Examples/PlannerPlugin.md) |
| `NavfnPlanner::makePlan` | 组织势场计算与回溯取路的主流程 | 同上文件（`makePlan` 锚点，约 218 行处） | `createPlan` | `NavFn::setNavArr`/`setCostmap`/`calcNavFnAstar`/`calcNavFnDijkstra`、`getPlanFromPotential`、`smoothApproachToGoal` | [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md) |
| `NavfnPlanner::getPlanFromPotential` | 从势场回溯出栅格路径并转世界坐标 | 同上文件（`getPlanFromPotential` 锚点，约 380 行处） | `makePlan` | `NavFn::calcPath` | [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md) |
| `NavfnPlanner::smoothApproachToGoal` | 修正路径末端使其精确落在目标点 | 同上文件（`smoothApproachToGoal` 锚点，约 351 行处） | `makePlan` | — | [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md) |
| `NavFn::setNavArr` | 分配/重置势场与代价数组 | nav2_navfn_planner/src/navfn.cpp（`setNavArr` 锚点，约 203 行处） | `NavfnPlanner::createPlan`、`makePlan` | — | [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md) |
| `NavFn::setCostmap` | 把 Costmap2D 代价翻译成 NavFn 内部代价 | 同上文件（`setCostmap` 锚点，约 243 行处） | `NavfnPlanner::makePlan` | — | [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md) |
| `NavFn::calcNavFnDijkstra` | Dijkstra 全场势能传播 | 同上文件（`calcNavFnDijkstra` 锚点，约 293 行处） | `NavfnPlanner::makePlan` | `setupNavFn`、`propNavFnDijkstra`、`calcPath` | [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md) |
| `NavFn::calcNavFnAstar` | A* 启发式势能传播（到起点即止） | 同上文件（`calcNavFnAstar` 锚点，约 307 行处） | `NavfnPlanner::makePlan` | `setupNavFn`、`propNavFnAstar` | [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md) |
| `NavFn::calcPath` | 沿势场负梯度回溯生成路径点 | 同上文件（`calcPath` 锚点，约 759 行处） | `NavfnPlanner::getPlanFromPotential`、`calcNavFnDijkstra` | `gradCell` | [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md) |
| ⚡ `SmacPlannerHybridT::createPlan` | Hybrid-A* 插件入口：预处理→搜索→平滑→输出 | nav2_smac_planner/include/nav2_smac_planner/smac_planner_hybrid_impl.hpp（`createPlan` 锚点，约 352 行处） | `PlannerServer::getPlan` | `AStarAlgorithm::setStart`/`setGoal`/`createPath`、`Smoother::smooth` | [08_SmacPlanner.md](08_SmacPlanner.md) |
| `SmacPlanner2DT::createPlan` | 2D A* 插件入口（带降采样） | nav2_smac_planner/include/nav2_smac_planner/smac_planner_2d_impl.hpp（`createPlan` 锚点，约 207 行处） | `PlannerServer::getPlan` | `setStart`/`setGoal`/`createPath`、`smooth` | [08_SmacPlanner.md](08_SmacPlanner.md) |
| `SmacPlannerLatticeT::createPlan` | State Lattice 插件入口（运动基元图搜索） | nav2_smac_planner/include/nav2_smac_planner/smac_planner_lattice_impl.hpp（`createPlan` 锚点，约 313 行处） | `PlannerServer::getPlan` | `setStart`/`setGoal`/`createPath`、`smooth` | [08_SmacPlanner.md](08_SmacPlanner.md) |
| `AStarAlgorithm::setStart` | 设置搜索起点节点（含姿态离散化） | nav2_smac_planner/include/nav2_smac_planner/a_star_impl.hpp（`setStart` 锚点，约 139 行处） | 三种 Smac 插件的 `createPlan` | `addToGraph` | [08_SmacPlanner.md](08_SmacPlanner.md) |
| `AStarAlgorithm::setGoal` | 设置目标节点（含目标朝向处理） | 同上文件（`setGoal` 锚点，约 188 行处） | 三种 Smac 插件的 `createPlan` | `addToGraph` | [08_SmacPlanner.md](08_SmacPlanner.md) |
| ⚡ `AStarAlgorithm::createPath` | A* 主搜索循环：出队-解析扩展-邻居入队-回溯 | 同上文件（`createPath` 锚点，约 334 行处） | 三种 Smac 插件的 `createPlan` | `AnalyticExpansion::tryAnalyticExpansion`、`NodeT::getNeighbors`、`NodeT::backtracePath` | [08_SmacPlanner.md](08_SmacPlanner.md) |
| `AnalyticExpansion::tryAnalyticExpansion` | 尝试用 Dubins/Reeds-Shepp 曲线直连目标 | nav2_smac_planner/include/nav2_smac_planner/analytic_expansion_impl.hpp（`tryAnalyticExpansion` 锚点，约 59 行处） | `AStarAlgorithm::createPath` | `getAnalyticPath`、碰撞检查 | [08_SmacPlanner.md](08_SmacPlanner.md) |
| `ThetaStarPlanner::createPlan` | Theta* 插件入口：搜索+线性插值 | nav2_theta_star_planner/src/theta_star_planner.cpp（`createPlan` 锚点，约 69 行处） | `PlannerServer::getPlan` | `ThetaStar::setStartAndGoal`/`generatePath`、`linearInterpolation` | [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md) |
| `ThetaStar::generatePath` | 任意角度 A* 搜索（视线检查跳父节点） | nav2_theta_star_planner/src/theta_star.cpp（`generatePath` 锚点，约 47 行处） | `ThetaStarPlanner::createPlan` | `losCheck`、`setNeighbors`、`backtracePath` | [07_PlannerServer与全局规划.md](07_PlannerServer与全局规划.md) |

## 四、局部控制域

| 函数 | 一句话职责 | 定义位置 | Caller（1 跳） | Callee（关键 1 跳） | 关联章节 |
|---|---|---|---|---|---|
| ⚡ `ControllerServer::computeControl` | FollowPath Action 回调：控制主循环（设路径→逐周期计算→到点退出） | nav2_controller/src/controller_server.cpp（`computeControl` 锚点，约 448 行处） | SimpleActionServer 工作线程（on_configure 中绑定） | `setPlannerPath`、`updateGlobalPath`、`isGoalReached`、`computeAndPublishVelocity` | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) |
| `ControllerServer::setPlannerPath` | 把新路径下发给当前控制器插件与目标检查器 | 同上文件（`setPlannerPath` 锚点，约 642 行处） | `computeControl`、`updateGlobalPath` | `controllers_[...]->setPlan`、末端位姿缓存 | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) |
| ⚡ `ControllerServer::computeAndPublishVelocity` | 单控制周期：取位姿→进度检查→算速度→发布反馈 | 同上文件（`computeAndPublishVelocity` 锚点，约 665 行处） | `computeControl`（循环体） | `progress_checkers_[...]->check`、`controllers_[...]->computeVelocityCommands`、`publishVelocity` | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md)、[CheatSheet/Controller.md](CheatSheet/Controller.md) |
| `ControllerServer::updateGlobalPath` | 抢占分支：接受 pending 目标并换路径 | 同上文件（`updateGlobalPath` 锚点，约 794 行处） | `computeControl` | `accept_pending_goal`、`setPlannerPath` | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) |
| `ControllerServer::publishVelocity` | 把速度指令发布到 cmd_vel | 同上文件（`publishVelocity` 锚点，约 860 行处） | `computeAndPublishVelocity`、`publishZeroVelocity` | `vel_publisher_->publish` | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) |
| `ControllerServer::publishZeroVelocity` | 发布零速指令（停车） | 同上文件（`publishZeroVelocity` 锚点，约 872 行处） | `computeControl` 结束/异常分支、`on_deactivate` | `publishVelocity` | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) |
| `ControllerServer::isGoalReached` | 委托 goal_checker 判断是否到达 | 同上文件（`isGoalReached` 锚点，约 898 行处） | `computeControl` | `goal_checkers_[...]->isGoalReached` | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) |
| ⚡ `MPPIController::computeVelocityCommands` | MPPI 插件入口：转交优化器并可视化 | nav2_mppi_controller/src/controller.cpp（`computeVelocityCommands` 锚点，约 93 行处） | `ControllerServer::computeAndPublishVelocity`（经插件基类） | `Optimizer::evalControl`、`visualize` | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md) |
| ⚡ `Optimizer::evalControl` | MPPI 单周期评估：准备→优化→取控制量→序列前移 | nav2_mppi_controller/src/optimizer.cpp（`evalControl` 锚点，约 230 行处） | `MPPIController::computeVelocityCommands` | `prepare`、`optimize`、`getControlFromSequenceAsTwist`、`shiftControlSequence` | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md) |
| ⚡ `Optimizer::optimize` | 迭代循环：生成噪声轨迹→critic 评分→更新控制序列 | 同上文件（`optimize` 锚点，约 272 行处） | `evalControl` | `generateNoisedTrajectories`、`CriticManager::evalTrajectoriesScores`、`updateControlSequence` | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md) |
| `Optimizer::prepare` | 装载本周期状态（位姿/速度/路径/目标） | 同上文件（`prepare` 锚点，约 300 行处） | `evalControl` | critics 数据结构重置 | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md) |
| `Optimizer::shiftControlSequence` | 控制序列整体前移一格（时域滚动） | 同上文件（`shiftControlSequence` 锚点，约 345 行处） | `evalControl` | — | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md) |
| `Optimizer::generateNoisedTrajectories` | 叠加噪声生成候选控制并积分成轨迹 | 同上文件（`generateNoisedTrajectories` 锚点，约 359 行处） | `optimize` | `NoiseGenerator::setNoisedControls`/`generateNextNoises`、`integrateStateVelocities` | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md)、[20_线程模型.md](20_线程模型.md) |
| `Optimizer::integrateStateVelocities` | 按运动学模型把速度序列积分为轨迹点 | 同上文件（`integrateStateVelocities` 锚点，约 489 行处，另有重载约 539 行处） | `generateNoisedTrajectories`、`getOptimizedTrajectory` | 运动模型 `predict` | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md) |
| `Optimizer::updateControlSequence` | 按 softmax 权重融合样本更新最优控制序列 | 同上文件（`updateControlSequence` 锚点，约 605 行处） | `optimize` | 代价归一化、加权求和 | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md) |
| `Optimizer::setSpeedLimit` | 按绝对值/百分比更新速度约束 | 同上文件（`setSpeedLimit` 锚点，约 691 行处） | `MPPIController::setSpeedLimit` | — | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md) |
| `CriticManager::evalTrajectoriesScores` | 逐一调用 critic 插件为全部候选轨迹累加代价 | nav2_mppi_controller/src/critic_manager.cpp（`evalTrajectoriesScores` 锚点，约 76 行处） | `Optimizer::optimize` | 各 `CriticFunction::score` | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md) |
| ⚡ `RegulatedPurePursuitController::computeVelocityCommands` | RPP 主流程：取前瞻点→曲率调速→碰撞前瞻 | nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp（`computeVelocityCommands` 锚点，约 161 行处） | `ControllerServer::computeAndPublishVelocity` | `getLookAheadPoint`、`shouldRotateToPath`、`rotateToHeading`、`applyConstraints`、`CollisionChecker::isCollisionImminent` | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md)、[Examples/ControllerPlugin.md](Examples/ControllerPlugin.md) |
| `RegulatedPurePursuitController::shouldRotateToPath` | 判断与路径夹角是否大到需要原地转向 | 同上文件（`shouldRotateToPath` 锚点，约 322 行处） | `computeVelocityCommands` | — | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md) |
| `RegulatedPurePursuitController::rotateToHeading` | 生成带加速度限幅的原地旋转指令 | 同上文件（`rotateToHeading` 锚点，约 351 行处） | `computeVelocityCommands` | — | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md) |
| `RegulatedPurePursuitController::applyConstraints` | 按曲率/障碍接近度/目标距离调节线速度 | 同上文件（`applyConstraints` 锚点，约 372 行处） | `computeVelocityCommands` | 各调速启发式 | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md) |
| `CollisionChecker::isCollisionImminent`（RPP） | 沿投影轨迹逐点做 footprint 碰撞前瞻 | nav2_regulated_pure_pursuit_controller/src/collision_checker.cpp（`isCollisionImminent` 锚点，约 48 行处） | `RegulatedPurePursuitController::computeVelocityCommands` | `inCollision`→`footprintCostAtPose` | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md) |
| `nav2_util::getLookAheadPoint` | 在路径上取（可插值的）前瞻点 | nav2_util/src/controller_utils.cpp（`getLookAheadPoint` 锚点，约 75 行处） | `RegulatedPurePursuitController::computeVelocityCommands`、`GracefulController` | 圆-线段交点插值 | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md) |
| `DWBLocalPlanner::computeVelocityCommands` | DWB 入口：准备评价数据后进入评分主流程 | nav2_dwb_controller/dwb_core/src/dwb_local_planner.cpp（`computeVelocityCommands` 锚点，约 203 行处，重载约 239 行处） | `ControllerServer::computeAndPublishVelocity` | `coreScoringAlgorithm` | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) |
| `DWBLocalPlanner::coreScoringAlgorithm` | 遍历速度样本轨迹并选出最优 | 同上文件（`coreScoringAlgorithm` 锚点，约 299 行处） | `computeVelocityCommands` | `scoreTrajectory` | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) |
| `DWBLocalPlanner::scoreTrajectory` | 逐 critic 累加单条轨迹的总代价 | 同上文件（`scoreTrajectory` 锚点，约 367 行处） | `coreScoringAlgorithm` | 各 `TrajectoryCritic::scoreTrajectory` | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) |
| `RotationShimController::computeVelocityCommands` | 未对准路径时先原地转向，否则转交主控制器 | nav2_rotation_shim_controller/src/nav2_rotation_shim_controller.cpp（`computeVelocityCommands` 锚点，约 119 行处） | `ControllerServer::computeAndPublishVelocity` | `computeRotateToHeadingCommand`、`primary_controller_->computeVelocityCommands` | [10_MPPI与RPP控制器.md](10_MPPI与RPP控制器.md)、[22_设计模式专题.md](22_设计模式专题.md) |
| `GracefulController::computeVelocityCommands` | 基于平滑控制律计算收敛到目标位姿的速度 | nav2_graceful_controller/src/graceful_controller.cpp（`computeVelocityCommands` 锚点，约 119 行处） | `ControllerServer::computeAndPublishVelocity` | `SmoothControlLaw::calculateRegularVelocity`、轨迹仿真 | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) |
| `SimpleGoalChecker::isGoalReached` | 按 xy/yaw 容差判断到点 | nav2_controller/plugins/simple_goal_checker.cpp（`isGoalReached` 锚点，约 118 行处） | `ControllerServer::isGoalReached`、`StoppedGoalChecker::isGoalReached` | — | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) |
| `SimpleProgressChecker::check` | 按时间窗内移动距离判断是否失去进度 | nav2_controller/plugins/simple_progress_checker.cpp（`check` 锚点，约 70 行处） | `ControllerServer::computeAndPublishVelocity` | `is_robot_moved_enough` | [09_ControllerServer与局部控制.md](09_ControllerServer与局部控制.md) |
| `VelocitySmoother::inputCommandCallback` | 缓存最近一次输入速度指令 | nav2_velocity_smoother/src/velocity_smoother.cpp（`inputCommandCallback` 锚点，约 250 行处） | cmd_vel 订阅回调（executor） | 指令合法性检查 | [16_CollisionMonitor与VelocitySmoother.md](16_CollisionMonitor与VelocitySmoother.md) |
| `VelocitySmoother::findEtaConstraint` | 计算三轴同缩放的加减速约束因子 η | 同上文件（`findEtaConstraint` 锚点，约 258 行处） | `smootherTimer` | — | [16_CollisionMonitor与VelocitySmoother.md](16_CollisionMonitor与VelocitySmoother.md) |
| ⚡ `VelocitySmoother::smootherTimer` | 定时器主循环：按约束平滑输出 cmd_vel | 同上文件（`smootherTimer` 锚点，约 312 行处） | ROS 定时器回调（on_activate 创建） | `findEtaConstraint`、`applyConstraints`、发布器 | [16_CollisionMonitor与VelocitySmoother.md](16_CollisionMonitor与VelocitySmoother.md)、[20_线程模型.md](20_线程模型.md) |
| `CollisionMonitor::cmdVelInCallbackStamped` | 输入速度回调：转入 process 安全处理 | nav2_collision_monitor/src/collision_monitor_node.cpp（`cmdVelInCallbackStamped` 锚点，约 211 行处） | cmd_vel_in 订阅回调、`cmdVelInCallbackUnstamped` | `process` | [16_CollisionMonitor与VelocitySmoother.md](16_CollisionMonitor与VelocitySmoother.md) |
| ⚡ `CollisionMonitor::process` | 安全闸主流程：按多边形动作类型修正速度后放行 | 同上文件（`process` 锚点，约 429 行处） | `cmdVelInCallbackStamped` | `processStopSlowdownLimit`、`processApproach`、`publishVelocity` | [16_CollisionMonitor与VelocitySmoother.md](16_CollisionMonitor与VelocitySmoother.md) |
| `CollisionMonitor::processStopSlowdownLimit` | 统计多边形内点数并执行停止/减速/限速 | 同上文件（`processStopSlowdownLimit` 锚点，约 544 行处） | `process` | `Polygon::getPointsInside` | [16_CollisionMonitor与VelocitySmoother.md](16_CollisionMonitor与VelocitySmoother.md) |
| `CollisionMonitor::processApproach` | 按预计碰撞时间缩放速度（approach 模式） | 同上文件（`processApproach` 锚点，约 608 行处） | `process` | `Polygon::getCollisionTime` | [16_CollisionMonitor与VelocitySmoother.md](16_CollisionMonitor与VelocitySmoother.md) |
| `Polygon::getCollisionTime` | 沿当前速度外推机器人，估计进入多边形的时间 | nav2_collision_monitor/src/polygon.cpp（`getCollisionTime` 锚点，约 339 行处） | `CollisionMonitor::processApproach` | 位姿外推、点包含判断 | [16_CollisionMonitor与VelocitySmoother.md](16_CollisionMonitor与VelocitySmoother.md) |

## 五、Costmap 域

| 函数 | 一句话职责 | 定义位置 | Caller（1 跳） | Callee（关键 1 跳） | 关联章节 |
|---|---|---|---|---|---|
| `Costmap2DROS::on_configure` | 创建 LayeredCostmap、装载图层/过滤器插件、建 TF 与发布器 | nav2_costmap_2d/src/costmap_2d_ros.cpp（`on_configure` 锚点，约 129 行处） | 生命周期管理器（经 rclcpp_lifecycle 状态机） | 插件 `initialize`、`LayeredCostmap` 构造 | [06_Costmap2D.md](06_Costmap2D.md)、[CheatSheet/Costmap.md](CheatSheet/Costmap.md) |
| `Costmap2DROS::on_activate` | 激活各层并创建地图更新线程 | 同上文件（`on_activate` 锚点，约 262 行处） | 生命周期管理器 | 各层 `activate`、创建 `map_update_thread_`→`mapUpdateLoop` | [06_Costmap2D.md](06_Costmap2D.md)、[20_线程模型.md](20_线程模型.md) |
| ⚡ `Costmap2DROS::mapUpdateLoop` | 更新线程常驻循环：按频率触发地图更新与发布 | 同上文件（`mapUpdateLoop` 锚点，约 533 行处） | `map_update_thread_`（on_activate 创建） | `updateMap`、`Costmap2DPublisher::publishCostmap` | [06_Costmap2D.md](06_Costmap2D.md)、[20_线程模型.md](20_线程模型.md) |
| ⚡ `Costmap2DROS::updateMap` | 单次更新：取机器人位姿后驱动分层更新 | 同上文件（`updateMap` 锚点，约 602 行处） | `mapUpdateLoop`、参数动态更新回调 | `getRobotPose`、`LayeredCostmap::updateMap`、footprint 发布 | [06_Costmap2D.md](06_Costmap2D.md) |
| `Costmap2DROS::getRobotPose` | 查询机器人在 global_frame 下的位姿 | 同上文件（`getRobotPose` 锚点，约 740 行处） | `updateMap`、`ClearCostmapService` | `nav2_util::getCurrentPose` | [06_Costmap2D.md](06_Costmap2D.md) |
| ⚡ `LayeredCostmap::updateMap` | 两阶段聚合：各层 updateBounds 后按序 updateCosts | nav2_costmap_2d/src/layered_costmap.cpp（`updateMap` 锚点，约 136 行处） | `Costmap2DROS::updateMap` | 各层 `updateBounds`/`updateCosts`、`Costmap2D::updateOrigin`（滚动窗口） | [06_Costmap2D.md](06_Costmap2D.md) |
| `LayeredCostmap::resizeMap` | 重设主/合成栅格尺寸并通知各层 matchSize | 同上文件（`resizeMap` 锚点，约 106 行处） | `Costmap2DROS::on_configure`、`StaticLayer` 地图变化 | `Costmap2D::resizeMap`、各层 `matchSize` | [06_Costmap2D.md](06_Costmap2D.md) |
| `Costmap2D::resizeMap` | 重新分配代价数组并重置元信息 | nav2_costmap_2d/src/costmap_2d.cpp（`resizeMap` 锚点，约 111 行处） | `LayeredCostmap::resizeMap`、`CostmapLayer::matchSize` | `initMaps`、`resetMaps` | [06_Costmap2D.md](06_Costmap2D.md) |
| `Costmap2D::worldToMap` | 世界坐标 → 栅格索引（越界返回 false） | 同上文件（`worldToMap` 锚点，约 292 行处） | 各图层/规划器/碰撞检查器（全仓高频调用） | — | [06_Costmap2D.md](06_Costmap2D.md) |
| `Costmap2D::mapToWorld` | 栅格索引 → 栅格中心世界坐标 | 同上文件（`mapToWorld` 锚点，约 280 行处） | 各图层/规划器（高频调用） | — | [06_Costmap2D.md](06_Costmap2D.md) |
| `Costmap2D::updateOrigin` | 滚动窗口平移：保留重叠区数据搬移原点 | 同上文件（`updateOrigin` 锚点，约 350 行处） | `LayeredCostmap::updateMap`（滚动窗口分支）、`ObstacleLayer::updateOrigin` | `copyMapRegion` | [06_Costmap2D.md](06_Costmap2D.md) |
| `Costmap2D::setConvexPolygonCost` | 把凸多边形内全部栅格设为指定代价 | 同上文件（`setConvexPolygonCost` 锚点，约 406 行处） | footprint 清除等调用方 | `nav2_util::raytraceLine`、`convexFillCells` | [06_Costmap2D.md](06_Costmap2D.md) |
| `StaticLayer::incomingMap` | 地图话题回调：缓存新地图待更新线程处理 | nav2_costmap_2d/plugins/static_layer.cpp（`incomingMap` 锚点，约 302 行处） | map 订阅回调（executor） | `processMap`（首次） | [06_Costmap2D.md](06_Costmap2D.md) |
| `StaticLayer::processMap` | 把 OccupancyGrid 翻译成本层代价栅格 | 同上文件（`processMap` 锚点，约 184 行处） | `incomingMap`、`updateBounds` | `interpretValue`、`LayeredCostmap::resizeMap` | [06_Costmap2D.md](06_Costmap2D.md) |
| `StaticLayer::updateBounds` | 报告静态图覆盖范围（含缓存图消化） | 同上文件（`updateBounds` 锚点，约 365 行处） | `LayeredCostmap::updateMap` | `processMap` | [06_Costmap2D.md](06_Costmap2D.md) |
| `StaticLayer::updateCosts` | 把本层代价合并进主栅格 | 同上文件（`updateCosts` 锚点，约 441 行处） | `LayeredCostmap::updateMap` | `updateWithMax`/`updateWithOverwrite` | [06_Costmap2D.md](06_Costmap2D.md) |
| `ObstacleLayer::updateBounds` | 消化观测：标记障碍并射线清除空闲区 | nav2_costmap_2d/plugins/obstacle_layer.cpp（`updateBounds` 锚点，约 476 行处） | `LayeredCostmap::updateMap` | `getMarkingObservations`/`getClearingObservations`、`raytraceFreespace` | [06_Costmap2D.md](06_Costmap2D.md) |
| `ObstacleLayer::updateCosts` | 把障碍标记合并进主栅格 | 同上文件（`updateCosts` 锚点，约 604 行处） | `LayeredCostmap::updateMap` | `updateWithMax` 等合并策略 | [06_Costmap2D.md](06_Costmap2D.md) |
| `ObstacleLayer::raytraceFreespace` | 从传感器原点向观测点射线清除自由空间 | 同上文件（`raytraceFreespace` 锚点，约 705 行处） | `ObstacleLayer::updateBounds` | `raytraceLine`、`updateRaytraceBounds` | [06_Costmap2D.md](06_Costmap2D.md) |
| `VoxelLayer::updateBounds` | 3D 体素版观测消化与向下投影 | nav2_costmap_2d/plugins/voxel_layer.cpp（`updateBounds` 锚点，约 161 行处） | `LayeredCostmap::updateMap` | 自身 `raytraceFreespace`（3D 版，约 278 行处） | [06_Costmap2D.md](06_Costmap2D.md) |
| `InflationLayer::updateCosts` | 以障碍为种子 BFS 向外写入膨胀代价 | nav2_costmap_2d/plugins/inflation_layer.cpp（`updateCosts` 锚点，约 283 行处） | `LayeredCostmap::updateMap` | `enqueue`、`costLookup`（缓存表） | [06_Costmap2D.md](06_Costmap2D.md) |
| `InflationLayer::computeCaches` | 预计算距离/代价查找表 | 同上文件（`computeCaches` 锚点，约 352 行处） | `onFootprintChanged`、参数更新 | `computeCost` | [06_Costmap2D.md](06_Costmap2D.md) |
| `ObservationBuffer::bufferCloud` | 把点云观测变换到全局系并入队 | nav2_costmap_2d/src/observation_buffer.cpp（`bufferCloud` 锚点，约 79 行处） | `ObstacleLayer` 的激光/点云订阅回调 | tf2 变换、`purgeStaleObservations` | [06_Costmap2D.md](06_Costmap2D.md) |
| `FootprintCollisionChecker::footprintCost` | 对给定 footprint 各边做栅格代价评估 | nav2_costmap_2d/src/footprint_collision_checker.cpp（`footprintCost` 锚点，约 48 行处） | `footprintCostAtPose`、Smac `GridCollisionChecker`、RPP 碰撞检查 | `lineCost`、`worldToMap` | [06_Costmap2D.md](06_Costmap2D.md) |

## 六、行为树域

| 函数 | 一句话职责 | 定义位置 | Caller（1 跳） | Callee（关键 1 跳） | 关联章节 |
|---|---|---|---|---|---|
| `BtActionServer::on_activate` | 激活 Action 服务器并加载默认行为树 | nav2_behavior_tree/include/nav2_behavior_tree/bt_action_server_impl.hpp（`on_activate` 锚点，约 197 行处） | 导航器插件 `activate`（经 BehaviorTreeNavigator） | `loadBehaviorTree`、`action_server_->activate` | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) |
| `BtActionServer::loadBehaviorTree` | 解析 BT XML、建树并挂接黑板 | 同上文件（`loadBehaviorTree` 锚点，约 242 行处） | `on_activate`、`NavigateToPoseNavigator::goalReceived`、`NavigateThroughPosesNavigator::goalReceived` | `BehaviorTreeEngine::createTreeFromFile`、黑板初始化 | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) |
| ⚡ `BtActionServer::executeCallback` | 导航目标执行入口：驱动 BT 运行至终态并回传结果 | 同上文件（`executeCallback` 锚点，约 400 行处） | SimpleActionServer 工作线程（构造中绑定） | `on_goal_received_callback_`、`BehaviorTreeEngine::run`、`haltTree` | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md)、[19_Action与通信机制.md](19_Action与通信机制.md) |
| ⚡ `BehaviorTreeEngine::run` | BT 主循环：检查取消→tickOnce→onLoop，直至终态 | nav2_behavior_tree/src/behavior_tree_engine.cpp（`run` 锚点，约 46 行处） | `BtActionServer::executeCallback` | `cancelRequested`（回调）、`tree->tickOnce`、`onLoop`（回调） | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) |
| `BehaviorTreeEngine::createTreeFromFile` | 从 XML 文件实例化行为树 | 同上文件（`createTreeFromFile` 锚点，约 113 行处） | `BtActionServer::loadBehaviorTree` | BT.CPP 工厂 `createTreeFromFile` | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) |
| `BehaviorTreeEngine::haltAllActions` | 停树：halt 根节点使全部子节点复位 | 同上文件（`haltAllActions` 锚点，约 212 行处） | `BtActionServer::haltTree` | `tree->haltTree` | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) |
| `BtActionNode::tick` | BT 叶节点核心：首 tick 发目标，后续等待结果并映射状态 | nav2_behavior_tree/include/nav2_behavior_tree/bt_action_node.hpp（`tick` 锚点，约 203 行处） | BT.CPP 引擎（树 tick 传播） | `on_tick`、`send_new_goal`、`on_wait_for_result`、`on_success`/`on_aborted`/`on_cancelled` | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md)、[19_Action与通信机制.md](19_Action与通信机制.md) |
| `BtNavigator::on_configure` | 创建 odom 平滑器并装载全部导航器插件 | nav2_bt_navigator/src/bt_navigator.cpp（`on_configure` 锚点，约 48 行处） | 生命周期管理器 | 导航器插件 `on_configure`（pluginlib 加载） | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) |
| `NavigateToPoseNavigator::goalReceived` | 新导航目标：按需换 BT 文件并写黑板 | nav2_bt_navigator/src/navigators/navigate_to_pose.cpp（`goalReceived` 锚点，约 93 行处） | `BtActionServer::executeCallback`（经 on_goal_received 回调） | `loadBehaviorTree`、`initializeGoalPose` | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) |
| `NavigateToPoseNavigator::onLoop` | 每轮 tick 后发布反馈（ETA/距离/恢复次数） | 同上文件（`onLoop` 锚点，约 127 行处） | `BehaviorTreeEngine::run`（经 onLoop 回调） | `nav2_util::getCurrentPose`、`isPathUpdated`、`publishFeedback` | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) |
| `NavigateToPoseNavigator::onPreempt` | 抢占请求：同 BT 文件则热切换目标，否则拒绝 | 同上文件（`onPreempt` 锚点，约 200 行处） | `BtActionServer` 抢占分支 | `initializeGoalPose`/`terminatePendingGoal` | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) |
| `NavigateToPoseNavigator::initializeGoalPose` | 把目标位姿与恢复计数写入黑板 | 同上文件（`initializeGoalPose` 锚点，约 231 行处） | `goalReceived`、`onPreempt` | 黑板 `set` | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) |
| `NavigateToPoseNavigator::goalCompleted` | 导航结束回调（结果码处理） | 同上文件（`goalCompleted` 锚点，约 106 行处） | `BtActionServer::executeCallback` 尾部 | — | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) |
| `NavigateThroughPosesNavigator::goalReceived` | 多途经点目标接收与黑板初始化 | nav2_bt_navigator/src/navigators/navigate_through_poses.cpp（`goalReceived` 锚点，约 83 行处） | `BtActionServer::executeCallback` | `loadBehaviorTree`、`initializeGoalPoses` | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) |
| `NavigateThroughPosesNavigator::onLoop` | 多途经点反馈发布（含剩余途经点数） | 同上文件（`onLoop` 锚点，约 133 行处） | `BehaviorTreeEngine::run` | `getCurrentPose`、`isPathUpdated`、`publishFeedback` | [11_行为树与BTNavigator.md](11_行为树与BTNavigator.md) |

## 七、恢复行为与其他任务服务器

| 函数 | 一句话职责 | 定义位置 | Caller（1 跳） | Callee（关键 1 跳） | 关联章节 |
|---|---|---|---|---|---|
| `TimedBehavior::execute` | 行为模板方法：onRun 初始化后按频率循环 onCycleUpdate | nav2_behaviors/include/nav2_behaviors/timed_behavior.hpp（`execute` 锚点，约 203 行处） | SimpleActionServer 工作线程（configure 中绑定） | `onRun`、`onCycleUpdate`、`stopRobot`、`onActionCompletion` | [12_Behaviors恢复行为.md](12_Behaviors恢复行为.md)、[22_设计模式专题.md](22_设计模式专题.md) |
| `TimedBehavior::stopRobot` | 行为结束/失败时发布零速 | 同上文件（`stopRobot` 锚点，约 297 行处） | `execute`（取消/失败分支） | `vel_pub_->publish` | [12_Behaviors恢复行为.md](12_Behaviors恢复行为.md) |
| `Spin::onRun` | 记录起始朝向并解析目标旋转角 | nav2_behaviors/plugins/spin.cpp（`onRun` 锚点，约 58 行处） | `TimedBehavior::execute` | `nav2_util::getCurrentPose` | [12_Behaviors恢复行为.md](12_Behaviors恢复行为.md) |
| `Spin::onCycleUpdate` | 每周期：计算剩余角度、碰撞前瞻、发旋转速度 | 同上文件（`onCycleUpdate` 锚点，约 85 行处） | `TimedBehavior::execute`（循环体） | `getCurrentPose`、碰撞检查、`vel_pub_->publish` | [12_Behaviors恢复行为.md](12_Behaviors恢复行为.md) |
| `BehaviorServer::loadBehaviorPlugins` | 用 pluginlib 装载全部行为插件 | nav2_behaviors/src/behavior_server.cpp（`loadBehaviorPlugins` 锚点，约 76 行处） | `on_configure` | pluginlib `createUniqueInstance`、插件 `configure` | [12_Behaviors恢复行为.md](12_Behaviors恢复行为.md)、[CheatSheet/Plugin.md](CheatSheet/Plugin.md) |
| `SmootherServer::smoothPlan` | SmoothPath Action 回调：分发到平滑器插件并校验碰撞 | nav2_smoother/src/nav2_smoother.cpp（`smoothPlan` 锚点，约 238 行处） | SimpleActionServer 工作线程（on_configure 中绑定） | `smoothers_[...]->smooth`、碰撞校验 | [15_Smoother路径平滑.md](15_Smoother路径平滑.md) |
| `SimpleSmoother::smooth` | 分段迭代加权平均平滑路径 | nav2_smoother/src/simple_smoother.cpp（`smooth` 锚点，约 53 行处） | `SmootherServer::smoothPlan` | `smoothImpl`、代价检查 | [15_Smoother路径平滑.md](15_Smoother路径平滑.md) |
| `WaypointFollower::followWaypointsCallback` | FollowWaypoints Action 回调入口 | nav2_waypoint_follower/src/waypoint_follower.cpp（`followWaypointsCallback` 锚点，约 357 行处） | SimpleActionServer 工作线程 | `followWaypointsHandler` | [19_Action与通信机制.md](19_Action与通信机制.md) |
| `WaypointFollower::followWaypointsHandler` | 逐途经点下发 NavigateToPose 并执行到点任务 | 同上文件（`followWaypointsHandler` 锚点，约 169 行处） | `followWaypointsCallback`、GPS 版回调 | `nav_to_pose_client_->async_send_goal`、`WaypointTaskExecutor::processAtWaypoint` | [19_Action与通信机制.md](19_Action与通信机制.md) |
| `WaypointFollower::resultCallback` | 子导航任务结果回调（成功/失败记账） | 同上文件（`resultCallback` 锚点，约 382 行处） | rclcpp_action 客户端回调 | — | [19_Action与通信机制.md](19_Action与通信机制.md) |
| ⚡ `AmclNode::laserReceived` | 激光回调：运动更新→观测更新→重采样→发布位姿与 TF | nav2_amcl/src/amcl_node.cpp（`laserReceived` 锚点，约 505 行处） | 激光订阅回调（message_filter） | `pf_update_action`（经运动模型）、传感器模型 `sensorUpdate`（内部调 `pf_update_sensor`）、`pf_update_resample` | [13_AMCL定位.md](13_AMCL定位.md) |
| `AmclNode::initialPoseReceived` | initialpose 话题回调：校验坐标系后转交处理 | 同上文件（`initialPoseReceived` 锚点，约 399 行处） | initialpose 订阅回调 | `handleInitialPose` | [13_AMCL定位.md](13_AMCL定位.md) |
| `AmclNode::handleInitialPose` | 用给定位姿+协方差重新初始化粒子群 | 同上文件（`handleInitialPose` 锚点，约 440 行处） | `initialPoseReceived` | `pf_init` | [13_AMCL定位.md](13_AMCL定位.md) |
| `MapServer::getMapCallback` | GetMap 服务回调：返回当前地图 | nav2_map_server/src/map_server/map_server.cpp（`getMapCallback` 锚点，约 179 行处） | map 服务回调（executor） | — | [14_MapServer.md](14_MapServer.md) |
| `MapServer::loadMapCallback` | LoadMap 服务回调：运行期热切换地图 | 同上文件（`loadMapCallback` 锚点，约 195 行处） | load_map 服务回调 | `loadMapFromYaml`（经 `loadMapResponseFromYaml`） | [14_MapServer.md](14_MapServer.md) |
| `map_io::loadMapFromYaml` | 解析 YAML 元数据后加载地图图片 | nav2_map_server/src/map_io.cpp（`loadMapFromYaml` 锚点，约 349 行处） | `MapServer::on_configure`、`loadMapCallback`、CLI 工具 | `loadMapYaml`、`loadMapFromFile` | [14_MapServer.md](14_MapServer.md) |
| `map_io::loadMapFromFile` | 用图像库把图片像素翻译为占据栅格 | 同上文件（`loadMapFromFile` 锚点，约 208 行处） | `loadMapFromYaml` | 像素→占据率换算 | [14_MapServer.md](14_MapServer.md) |
| `map_io::saveMapToFile` | 把占据栅格写为图片+YAML | 同上文件（`saveMapToFile` 锚点，约 637 行处） | `MapSaver::saveMapTopicToFile` | `checkSaveParameters`、图像编码 | [14_MapServer.md](14_MapServer.md) |
| `RouteServer::findRoute` | 路线计算核心：起终点提取→图搜索→稠密化 | nav2_route/src/route_server.cpp（`findRoute` 锚点，约 209 行处） | `computeRoute`、`computeAndTrackRoute`（经模板辅助） | `GoalIntentExtractor::findStartandGoal`、`RoutePlanner::findRoute` | [17_Route路线规划.md](17_Route路线规划.md) |
| `RouteServer::computeRoute` | ComputeRoute Action 回调 | 同上文件（`computeRoute` 锚点，约 346 行处） | SimpleActionServer 工作线程 | `findRoutePlan`→`findRoute` | [17_Route路线规划.md](17_Route路线规划.md) |
| `RoutePlanner::findRoute` | 路网图上 Dijkstra 式最优路线搜索 | nav2_route/src/route_planner.cpp（`findRoute` 锚点，约 42 行处） | `RouteServer::findRoute` | 边代价评估（EdgeScorer）、优先队列扩展 | [17_Route路线规划.md](17_Route路线规划.md) |
| `RouteTracker::trackRoute` | 跟踪执行中的路线：进度估计与操作触发 | nav2_route/src/route_tracker.cpp（`trackRoute` 锚点，约 145 行处） | `RouteServer::computeAndTrackRoute` | `nav2_util::getCurrentPose`、`OperationsManager::process` | [17_Route路线规划.md](17_Route路线规划.md) |
| `GoalIntentExtractor::findStartandGoal` | 把请求中的位姿/节点 ID 解析为路网起终点 | nav2_route/src/goal_intent_extractor.cpp（`findStartandGoal` 锚点，约 94 行处） | `RouteServer::findRoute` | 最近节点搜索、`getCurrentPose` | [17_Route路线规划.md](17_Route路线规划.md) |
| `DockingServer::dockRobot` | DockRobot Action 回调：导航→旋转对准→逼近→充电确认 | nav2_docking/opennav_docking/src/docking_server.cpp（`dockRobot` 锚点，约 175 行处） | SimpleActionServer 工作线程 | `rotateToDock`、`approachDock`、`waitForCharge` | [18_Docking靠泊.md](18_Docking靠泊.md) |
| `DockingServer::approachDock` | 用检测位姿闭环逼近充电桩 | 同上文件（`approachDock` 锚点，约 453 行处） | `dockRobot` | 控制律计算、碰撞检查、`vel_publisher_->publish` | [18_Docking靠泊.md](18_Docking靠泊.md) |
| `DockingServer::undockRobot` | UndockRobot Action 回调：倒车脱离并确认脱桩 | 同上文件（`undockRobot` 锚点，约 621 行处） | SimpleActionServer 工作线程 | `rotateToDock`（按需）、控制律、`isCharging` 检查 | [18_Docking靠泊.md](18_Docking靠泊.md) |

## 附：双向查询小抄

- **谁在调 `createPlan`（插件接口）？** → 只有 `PlannerServer::getPlan` 一处（见第三节）。
- **谁在调 `computeVelocityCommands`（插件接口）？** → 只有 `ControllerServer::computeAndPublishVelocity` 一处（`RotationShimController` 内部再转发给主控制器，见第四节）。
- **谁在调 `updateBounds`/`updateCosts`（图层接口）？** → 只有 `LayeredCostmap::updateMap` 一处（见第五节）。
- **谁在调 `execute_callback_`（各服务器主回调）？** → 只有 `SimpleActionServer::work` 一处（见第二节）。
- 更多接口契约见 [04_nav2_core插件接口.md](04_nav2_core插件接口.md)，线程全景见 [20_线程模型.md](20_线程模型.md)。
