# 术语表（Glossary）

> 口径：Nav2 1.5.0（main 分支）、C++20。术语按**英文字母 / 中文拼音首字母**混合排序分节；"首次出现章节"指本书系统讲解该术语的章节（并非字面首提）。配套：[ClassIndex.md](ClassIndex.md)、[FileIndex.md](FileIndex.md)、[APIIndex.md](APIIndex.md)，导航枢纽 [Index.md](Index.md)。

## A

| 术语 | 英文 | 一句话定义 | 首次出现章节 | 相关概念 |
|---|---|---|---|---|
| Action | ROS 2 Action | 带反馈、可取消、可抢占的长任务通信原语，Nav2 全部任务接口的主体 | [19](19_Action与通信机制.md) | Service、SimpleActionServer |
| A*（A 星） | A-star | 带启发式的最短路搜索算法，Smac 系列与 NavFn A* 模式的核心 | [07](07_PlannerServer与全局规划.md) | Dijkstra、启发式 |
| admissible 启发式 | Admissible Heuristic | 永不高估剩余代价的启发式，是 A* 保证最优性的前提 | [08](08_SmacPlanner.md) | A*、双启发式 |
| AMCL | Adaptive Monte Carlo Localization | 自适应蒙特卡洛定位：用粒子滤波在已知地图中估计机器人位姿 | [13](13_AMCL定位.md) | 粒子滤波、似然场模型 |
| AnalyticExpansion（解析扩展） | Analytic Expansion | Hybrid-A* 接近目标时用 Dubins/RS 曲线直连目标的加速手段 | [08](08_SmacPlanner.md) | Dubins 曲线、Reeds-Shepp 曲线 |
| AssistedTeleop | Assisted Teleop | 辅助遥操作恢复行为：把人工速度指令投影到无碰撞方向 | [12](12_Behaviors恢复行为.md) | TimedBehavior、恢复行为 |
| autostart | autostart | lifecycle_manager 启动时是否自动拉起全部受管节点的开关参数 | [05](05_生命周期管理.md) | lifecycle_manager |
| AdjustSpeedLimit | Adjust Speed Limit | Route 跟踪期按边属性动态发布限速的 RouteOperation 插件 | [17](17_Route路线规划.md) | RouteOperation、限速 |

## B

| 术语 | 英文 | 一句话定义 | 首次出现章节 | 相关概念 |
|---|---|---|---|---|
| BackUp | Back Up | 后退恢复行为，DriveOnHeading 取反方向的特化 | [12](12_Behaviors恢复行为.md) | DriveOnHeading |
| BasicNavigator | Basic Navigator | nav2_simple_commander 提供的 Python 门面类，一行代码发起导航 | [24](24_扩展开发指南.md) | 外观模式 |
| BehaviorServer | Behavior Server | 恢复行为宿主服务器，每个行为插件对应一个独立 Action | [12](12_Behaviors恢复行为.md) | TimedBehavior |
| Behavior 接口 | nav2_core::Behavior | 恢复行为插件契约（configure/activate/run） | [04](04_nav2_core插件接口.md) | 插件机制 |
| BehaviorTree.CPP | BehaviorTree.CPP | Nav2 采用的第三方 C++ 行为树库（工厂、XML、tick 语义） | [11](11_行为树与BTNavigator.md) | 行为树、Groot |
| BehaviorTreeEngine | Behavior Tree Engine | 封装 BT 工厂与循环 tick 的引擎类 | [11](11_行为树与BTNavigator.md) | BtActionServer |
| BehaviorTreeNavigator | nav2_core::BehaviorTreeNavigator | Navigator 插件的模板基类，持有 BtActionServer | [11](11_行为树与BTNavigator.md) | NavigateToPoseNavigator |
| BinaryFilter | Binary Filter | 代价过滤器：进出掩码区域时发布布尔开关信号 | [06](06_Costmap2D.md) | CostmapFilter |
| bond（保活） | Bond | 服务器与 lifecycle_manager 间的心跳连接，断开即触发"一个倒下全体放倒 + 整体重启" | [05](05_生命周期管理.md) | lifecycle_manager、respawn |
| BtActionNode | BT Action Node | 把 ROS Action 客户端包装成 BT 叶节点的模板基类，几十个 BT 插件的共同骨架 | [11](11_行为树与BTNavigator.md) | BtServiceNode |
| BtActionServer | BT Action Server | bt_navigator 内加载 XML、tick 树、处理抢占的 Action 服务器骨架 | [11](11_行为树与BTNavigator.md) | BehaviorTreeEngine |
| BtNavigator | BT Navigator | 行为树导航器服务器，加载 Navigator 插件并互斥调度 | [11](11_行为树与BTNavigator.md) | NavigatorMuxer |
| BtServiceNode | BT Service Node | 把 ROS Service 客户端包装成 BT 叶节点的模板基类 | [11](11_行为树与BTNavigator.md) | BtActionNode |

## C

| 术语 | 英文 | 一句话定义 | 首次出现章节 | 相关概念 |
|---|---|---|---|---|
| ChargingDock | opennav_docking_core::ChargingDock | 坞站插件契约：检测坞站位姿、判断充电状态 | [18](18_Docking靠泊.md) | SimpleChargingDock |
| ClassLoader | pluginlib::ClassLoader | 运行时按注册名动态加载插件类的加载器 | [04](04_nav2_core插件接口.md) | pluginlib、插件描述 XML |
| ClearCostmapService | Clear Costmap Service | 清除代价地图（全部/局部/指定区域）的服务三件套 | [06](06_Costmap2D.md) | Costmap2DROS |
| cmd_vel | cmd_vel | 底盘速度指令话题，三级安全链的最终输出与底盘唯一入口 | [01](01_整体架构.md) | 速度指令三级链路 |
| cmd_vel_nav | cmd_vel_nav | 控制器与恢复行为并联输出的原始速度话题（链路第一级） | [01](01_整体架构.md) | cmd_vel_smoothed |
| cmd_vel_smoothed | cmd_vel_smoothed | velocity_smoother 平滑后的中间速度话题（链路第二级） | [16](16_CollisionMonitor与VelocitySmoother.md) | VelocitySmoother |
| CollisionMonitor | Collision Monitor | 直连原始传感器、对速度指令做最终安全裁决的独立节点 | [16](16_CollisionMonitor与VelocitySmoother.md) | 停止区/减速区、Polygon |
| Composition（组件化） | Component Composition | 多个节点装入同一进程共享执行器的部署方式 | [20](20_线程模型.md) | 执行器、进程内通信 |
| ComputePathToPose | Compute Path To Pose | 全局规划 Action 接口：起终点进、Path 出 | [07](07_PlannerServer与全局规划.md) | PlannerServer |
| ComputeRoute | Compute Route | Route 图搜索 Action：一次性输出 Route 与 Path | [17](17_Route路线规划.md) | ComputeAndTrackRoute |
| ComputeAndTrackRoute | Compute And Track Route | Route 边走边跟踪的 Action，途中触发 Operations | [17](17_Route路线规划.md) | RouteTracker |
| condition_variable | std::condition_variable | 线程间等待/唤醒原语，Nav2 用于循环节流与关停通知 | [23](23_现代C++专题.md) | mutex、atomic |
| constexpr | constexpr | 把常量计算移到编译期的 C++ 关键字 | [23](23_现代C++专题.md) | 现代 C++ |
| ControllerServer | Controller Server | 局部控制宿主服务器，唯一的高频实时循环服务器 | [09](09_ControllerServer与局部控制.md) | FollowPath、控制频率 |
| Controller 接口 | nav2_core::Controller | 控制器插件契约（setPlan/computeVelocityCommands 等） | [04](04_nav2_core插件接口.md) | 插件机制 |
| Costmap2D | Costmap 2D | 纯数据结构的二维代价栅格（无 ROS 依赖） | [06](06_Costmap2D.md) | Costmap2DROS |
| Costmap2DROS | Costmap 2D ROS | 代价地图的 ROS 封装：生命周期、更新线程、TF、插件装载 | [06](06_Costmap2D.md) | LayeredCostmap、mapUpdateLoop |
| CostmapFilter | Costmap Filter | 掩码类过滤层的模板方法基类（Keepout/Speed/Binary 的父类） | [06](06_Costmap2D.md) | filter mask |
| CostmapLayer | Costmap Layer | 自带私有栅格的层基类，提供 updateWithMax 等合并策略 | [06](06_Costmap2D.md) | Layer |
| critic（评分器） | Critic | DWB/MPPI 中对候选轨迹打分的可插拔评价器，责任链式求和 | [09](09_ControllerServer与局部控制.md) | CriticManager、责任链模式 |
| CriticManager | Critic Manager | MPPI 中加载并逐一调用 critic 插件的管理器 | [10](10_MPPI与RPP控制器.md) | Optimizer |

## D

| 术语 | 英文 | 一句话定义 | 首次出现章节 | 相关概念 |
|---|---|---|---|---|
| 代价地图 | Costmap | 融合静态地图与传感数据、以 0~255 代价表达可通行性的栅格 | [06](06_Costmap2D.md) | 分层代价地图、膨胀 |
| 单例（被拒绝的模式） | Singleton | Nav2 刻意不采用的全局状态模式，代之以依赖注入 | [22](22_设计模式专题.md) | 依赖注入 |
| DDS | Data Distribution Service | ROS 2 底层的发布订阅通信中间件标准 | [19](19_Action与通信机制.md) | QoS、Topic |
| DenoiseLayer | Denoise Layer | 用图像形态学算子去除代价地图孤立噪点的层 | [06](06_Costmap2D.md) | Layer |
| DifferentialMotionModel | Differential Motion Model | 差速底盘的里程计运动模型（alpha1~4 噪声参数） | [13](13_AMCL定位.md) | 粒子滤波 |
| Dijkstra | Dijkstra | 无启发式的最短路搜索，NavFn 的默认模式 | [07](07_PlannerServer与全局规划.md) | 势场、A* |
| DockingServer | Docking Server | 靠泊服务器：预备位姿 → 检测 → 接近 → 充电确认的状态机宿主 | [18](18_Docking靠泊.md) | ChargingDock、DockRobot |
| DockRobot / UndockRobot | Dock / Undock Robot | 靠泊与脱离坞站的两个 Action 接口 | [18](18_Docking靠泊.md) | DockingServer |
| dock 数据库 | Dock Database | 已知坞站实例（类型 + 位姿）的注册表 | [18](18_Docking靠泊.md) | ChargingDock |
| DriveOnHeading | Drive On Heading | 沿当前朝向直线行驶的恢复行为模板 | [12](12_Behaviors恢复行为.md) | BackUp |
| Dubins 曲线 | Dubins Curve | 只允许前进的最短曲线族，Hybrid-A* 前向模式的解析扩展 | [08](08_SmacPlanner.md) | Reeds-Shepp 曲线 |
| DWA | Dynamic Window Approach | 动态窗口法：在可达速度窗口内采样评分，DWB 的算法前身 | [09](09_ControllerServer与局部控制.md) | DWB |
| DWB | DWB Controller | 基于速度采样 + critic 链评分的可定制控制器（DWA 重构版） | [09](09_ControllerServer与局部控制.md) | critic、TrajectoryGenerator |
| 动态参数 | Dynamic Parameters | 运行时热更新参数的机制（add_on_set_parameters_callback） | [21](21_配置系统.md) | 配置系统 |

## E / F

| 术语 | 英文 | 一句话定义 | 首次出现章节 | 相关概念 |
|---|---|---|---|---|
| EdgeCostFunction（边评分器） | Edge Cost Function / Edge Scorer | Route 图搜索中评估每条边代价的插件族 | [17](17_Route路线规划.md) | RoutePlanner |
| error_code | Error Code | Nav2 各 Action result 中标准化的失败原因编码，分层排障的依据 | [19](19_Action与通信机制.md) | feedback |
| Executor（执行器） | Executor | 调度节点回调的事件循环（单线程/多线程） | [20](20_线程模型.md) | 回调组、spin |
| Fallback 节点 | Fallback (Selector) | 依次尝试子节点直到一个成功的 BT 控制节点 | [11](11_行为树与BTNavigator.md) | Sequence 节点 |
| feedback | Feedback | Action 执行期间的周期性进度回报 | [19](19_Action与通信机制.md) | goal handle |
| filter mask（过滤掩码） | Filter Mask | 与地图对齐、编码禁区/限速等语义的辅助栅格 | [06](06_Costmap2D.md) | CostmapFilter、CostmapFilterInfoServer |
| FollowPath | Follow Path | 局部控制 Action 接口：Path 进、持续速度指令出 | [09](09_ControllerServer与局部控制.md) | ControllerServer |
| FollowWaypoints | Follow Waypoints | 航点巡航 Action：逐点调用 NavigateToPose | [19](19_Action与通信机制.md) | WaypointFollower |
| footprint（足迹） | Footprint | 机器人外轮廓多边形，碰撞检查与膨胀的基准 | [06](06_Costmap2D.md) | 内切/外接半径 |
| FootprintCollisionChecker | Footprint Collision Checker | 按足迹多边形在代价地图上做碰撞检查的工具类 | [06](06_Costmap2D.md) | GridCollisionChecker |
| FREE_SPACE | Free Space | 代价值 0：自由空间 | [06](06_Costmap2D.md) | LETHAL_OBSTACLE |
| 分层代价地图 | Layered Costmap | 静态/障碍/膨胀等多层按序叠合的代价地图组织方式 | [06](06_Costmap2D.md) | LayeredCostmap |
| 服务 | Service | 一问一答的同步 RPC 通信原语，适合快速查询类接口 | [19](19_Action与通信机制.md) | Action、Topic |

## G / H

| 术语 | 英文 | 一句话定义 | 首次出现章节 | 相关概念 |
|---|---|---|---|---|
| GeoJSON | GeoJSON | Route 路网图文件的默认格式 | [17](17_Route路线规划.md) | GraphFileLoader |
| GlobalPlanner 接口 | nav2_core::GlobalPlanner | 全局规划插件契约（createPlan） | [04](04_nav2_core插件接口.md) | PlannerServer |
| goal handle | Goal Handle | Action 目标的句柄对象，承载接受/取消/结果状态 | [19](19_Action与通信机制.md) | SimpleActionServer |
| GoalChecker 接口 | nav2_core::GoalChecker | 判定"是否到达目标"的插件契约 | [04](04_nav2_core插件接口.md) | SimpleGoalChecker |
| GoalIntentExtractor | Goal Intent Extractor | Route 中把自由位姿映射到路网起终点的组件（空间索引加速） | [17](17_Route路线规划.md) | RouteServer |
| GracefulController | Graceful Controller | 基于平滑控制律的姿态收敛控制器 | [09](09_ControllerServer与局部控制.md) | Controller 接口 |
| GridCollisionChecker | Grid Collision Checker | Smac 在 SE(2) 栅格上按朝向做足迹碰撞检查的组件 | [08](08_SmacPlanner.md) | footprint |
| Groot | Groot | BehaviorTree.CPP 配套的行为树可视化/监控编辑器 | [11](11_行为树与BTNavigator.md) | XML 行为树 |
| 黑板 | Blackboard | BT 各节点共享数据的键值存储，Nav2 用它传递 goal/path 等 | [11](11_行为树与BTNavigator.md) | BehaviorTree.CPP |
| 恢复行为 | Recovery Behavior | 导航失败时的脱困动作（清图/旋转/后退/等待） | [12](12_Behaviors恢复行为.md) | RecoveryNode |
| 回调组 | Callback Group | 控制回调能否并发执行的分组机制（互斥/可重入） | [20](20_线程模型.md) | Executor |
| 话题 | Topic | 发布/订阅式异步流数据通信原语 | [19](19_Action与通信机制.md) | QoS、DDS |
| Hybrid-A* | Hybrid A-star | 连续朝向 + 运动基元扩展的 A* 变体，天然满足运动学约束 | [08](08_SmacPlanner.md) | 运动基元、解析扩展 |

## I / J / K

| 术语 | 英文 | 一句话定义 | 首次出现章节 | 相关概念 |
|---|---|---|---|---|
| InflationLayer | Inflation Layer | 从致命障碍向外按指数衰减扩散代价的层 | [06](06_Costmap2D.md) | 膨胀、cost_scaling_factor |
| InputAtWaypoint | Input At Waypoint | 航点处等待外部输入的任务执行器插件 | [19](19_Action与通信机制.md) | WaypointTaskExecutor |
| INSCRIBED_INFLATED_OBSTACLE | Inscribed Inflated Obstacle | 代价值 253：机器人中心落入即必然碰撞（内切圆内） | [06](06_Costmap2D.md) | 内切半径 |
| invoke | invoke | nav2::ServiceClient 的同步调用封装（内部独立 executor 等待） | [19](19_Action与通信机制.md) | ServiceClient |
| 加速度约束 | Acceleration Limits | velocity_smoother 做速度平滑的加/减速度上限依据 | [16](16_CollisionMonitor与VelocitySmoother.md) | VelocitySmoother |
| 降采样 | Downsampling | Smac2D 降低搜索分辨率以加速、再插值还原的手段 | [08](08_SmacPlanner.md) | SmacPlanner2D |
| 节点 | Node | ROS 2 的基本计算单元，拥有自己的接口与参数 | [01](01_整体架构.md) | Executor、Lifecycle |
| KeepoutFilter | Keepout Filter | 按掩码把禁区写为致命代价的过滤层 | [06](06_Costmap2D.md) | filter mask |
| KLD 采样 | KLD Sampling | AMCL 按分布复杂度自适应调整粒子数量的方法 | [13](13_AMCL定位.md) | 自适应重采样 |
| 控制频率 | controller_frequency | 控制循环的目标频率，决定实时性与算力预算 | [09](09_ControllerServer与局部控制.md) | 软实时 |

## L

| 术语 | 英文 | 一句话定义 | 首次出现章节 | 相关概念 |
|---|---|---|---|---|
| latched QoS | Transient Local QoS | 让迟到订阅者也能收到最后一条消息的持久性 QoS（地图话题标配） | [14](14_MapServer.md) | QoS |
| Layer 接口 | nav2_costmap_2d::Layer | 代价地图层插件契约（updateBounds/updateCosts 两阶段） | [06](06_Costmap2D.md) | LayeredCostmap |
| LayeredCostmap | Layered Costmap | 持有层插件列表、执行两阶段合成的聚合器 | [06](06_Costmap2D.md) | Costmap2DROS |
| LETHAL_OBSTACLE | Lethal Obstacle | 代价值 254：栅格内确有障碍 | [06](06_Costmap2D.md) | INSCRIBED_INFLATED_OBSTACLE |
| lifecycle_manager | Lifecycle Manager | 按序驱动受管节点状态转换并 bond 监活的管理节点 | [05](05_生命周期管理.md) | bond、LifecycleNode |
| LifecycleNode（nav2::） | nav2::LifecycleNode | 在 rclcpp_lifecycle 之上加 bond 创建与 rcl 预关闭回调的基类，1.5.0 位于 nav2_ros_common 包 | [05](05_生命周期管理.md) | 生命周期、nav2_ros_common |
| LikelihoodFieldModel（似然场模型） | Likelihood Field Model | AMCL 默认激光观测模型：按最近障碍距离场评分 | [13](13_AMCL定位.md) | 粒子滤波 |
| loopback_sim | Loopback Simulator | 无物理引擎的回环模拟器，最低成本跑通导航栈 | [25](25_学习路线图.md) | nav2_bringup |
| 粒子滤波 | Particle Filter | 用带权粒子集近似位姿后验分布的贝叶斯滤波 | [13](13_AMCL定位.md) | AMCL、重采样 |
| 里程计 | Odometry | 由轮速/IMU 积分得到的相对位姿估计（odom 坐标系来源） | [01](01_整体架构.md) | TF、OdomSmoother |
| 路网 | Route Graph | 预定义的节点 + 边导航图，Route 域的搜索空间 | [17](17_Route路线规划.md) | GeoJSON |

## M / N

| 术语 | 英文 | 一句话定义 | 首次出现章节 | 相关概念 |
|---|---|---|---|---|
| MapSaver | Map Saver | 把地图话题落盘为 yaml + 图片的工具节点 | [14](14_MapServer.md) | MapServer |
| MapServer | Map Server | 加载 map.yaml 与图片、常驻提供 OccupancyGrid 的服务器 | [14](14_MapServer.md) | latched QoS |
| map→odom 变换 | map to odom Transform | AMCL 发布的定位校正量：全局位姿 = map→odom ∘ odom→base_link | [13](13_AMCL定位.md) | TF |
| move_base | move_base | ROS 1 时代的单体导航节点，Nav2 的前身 | [00](00_项目概览.md) | Nav2 |
| MPPI | Model Predictive Path Integral | 采样型模型预测控制：千条带噪轨迹经 critic 评分后加权平均 | [10](10_MPPI与RPP控制器.md) | Optimizer、critic |
| mutex（互斥锁） | std::mutex | 保护共享数据的互斥原语，Nav2 热路径上尽量短持有 | [20](20_线程模型.md) | 数据竞争 |
| nav2_core | nav2_core | 只含纯虚接口的"接口宪法"包，所有算法插件的契约来源 | [04](04_nav2_core插件接口.md) | 插件机制 |
| nav2_msgs | nav2_msgs | Nav2 全部 Action/msg/srv 接口定义包 | [19](19_Action与通信机制.md) | Action |
| nav2_ros_common | nav2_ros_common | 基础封装包：LifecycleNode、SimpleActionServer、ServiceClient 等（nav2:: 命名空间，1.5.0 起替代 nav2_util 承载这些类） | [05](05_生命周期管理.md) | LifecycleNode |
| nav2_simple_commander | Nav2 Simple Commander | Python 高层 API 包，BasicNavigator 所在地 | [24](24_扩展开发指南.md) | BasicNavigator |
| NavFn | NavFn | 基于势场传播（Dijkstra/A*）的经典全局规划器 | [07](07_PlannerServer与全局规划.md) | 势场 |
| NavigateToPose | Navigate To Pose | 单点导航顶层 Action，bt_navigator 的主入口 | [01](01_整体架构.md) | BtNavigator |
| NavigateThroughPoses | Navigate Through Poses | 多途经点导航顶层 Action，一次规划穿多点 | [11](11_行为树与BTNavigator.md) | NavigateToPose |
| NavigatorMuxer | Navigator Muxer | 保证同一时刻只有一个 Navigator 插件活动的互斥器 | [11](11_行为树与BTNavigator.md) | 中介者模式 |
| NodeHybrid / Node2D / NodeLattice | Smac Node Types | Smac 三种搜索节点：SE(2) 运动基元 / 2D 邻域 / lattice 基元 | [08](08_SmacPlanner.md) | AStarAlgorithm |
| NodeThread | Node Thread | 构造即为节点开一条独立 spin 线程的工具类 | [20](20_线程模型.md) | Executor |
| NoiseGenerator | Noise Generator | MPPI 后台线程预生成控制噪声矩阵的组件 | [10](10_MPPI与RPP控制器.md) | Optimizer |
| NO_INFORMATION | No Information | 代价值 255：未知区域 | [06](06_Costmap2D.md) | track_unknown_space |
| 内切/外接半径 | Inscribed / Circumscribed Radius | 足迹的内切圆与外接圆半径，代价语义分界的几何依据 | [06](06_Costmap2D.md) | footprint、膨胀 |

## O / P / Q

| 术语 | 英文 | 一句话定义 | 首次出现章节 | 相关概念 |
|---|---|---|---|---|
| ObservationBuffer | Observation Buffer | 缓存传感观测并做 TF 变换/超时管理的缓冲区 | [06](06_Costmap2D.md) | ObstacleLayer |
| ObstacleLayer | Obstacle Layer | 按传感观测标记障碍（marking）与射线清除（clearing）的层 | [06](06_Costmap2D.md) | raytrace 清除 |
| OccupancyGrid | Occupancy Grid | ROS 占用栅格地图消息（-1 未知 / 0 空闲 / 100 占用） | [14](14_MapServer.md) | MapServer |
| OdomSmoother | Odom Smoother | 对里程计滑动窗口平滑、供控制器取当前速度的工具 | [09](09_ControllerServer与局部控制.md) | 里程计 |
| on_configure / on_activate | Lifecycle Callbacks | 生命周期转换回调：申请资源在前者、对外生效在后者 | [05](05_生命周期管理.md) | 生命周期 |
| Optimizer（MPPI） | MPPI Optimizer | MPPI 核心：生成噪声轨迹→评分→加权更新控制序列 | [10](10_MPPI与RPP控制器.md) | critic、NoiseGenerator |
| 膨胀 | Inflation | 把障碍代价按距离衰减函数向外扩散，给规划器留安全梯度 | [06](06_Costmap2D.md) | InflationLayer |
| pending goal | Pending Goal | SimpleActionServer 中排队等待抢占切换的新目标 | [19](19_Action与通信机制.md) | 抢占 |
| PhotoAtWaypoint | Photo At Waypoint | 航点处拍照存图的任务执行器插件 | [19](19_Action与通信机制.md) | WaypointTaskExecutor |
| PipelineSequence | Pipeline Sequence | 前序子节点成功后仍持续重 tick 的 BT 控制节点（边规划边走的关键） | [11](11_行为树与BTNavigator.md) | Sequence 节点 |
| PlannerServer | Planner Server | 全局规划宿主服务器，持有 global_costmap 与规划器插件映射 | [07](07_PlannerServer与全局规划.md) | ComputePathToPose |
| pluginlib | pluginlib | ROS 2 插件注册/发现/动态加载框架 | [04](04_nav2_core插件接口.md) | ClassLoader |
| Polygon（碰撞监视） | Collision Monitor Polygon | CollisionMonitor 中定义停止/减速/限制/接近区域的多边形 | [16](16_CollisionMonitor与VelocitySmoother.md) | 停止区/减速区/限制区 |
| PoseProgressChecker | Pose Progress Checker | 平移 + 角度双判据的进度检查器 | [09](09_ControllerServer与局部控制.md) | ProgressChecker 接口 |
| PositionGoalChecker | Position Goal Checker | 只看位置不看朝向的目标检查器 | [09](09_ControllerServer与局部控制.md) | GoalChecker 接口 |
| ProgressChecker 接口 | nav2_core::ProgressChecker | 判定"是否仍在取得进展"的插件契约，卡死检测的来源 | [04](04_nav2_core插件接口.md) | SimpleProgressChecker |
| 前瞻距离 | Lookahead Distance | RPP 在路径上选取追踪点的距离（可随速度调节） | [10](10_MPPI与RPP控制器.md) | RPP |
| 抢占 | Preemption | 新目标到达时替换执行中目标的语义；Nav2 为协作式（执行循环轮询切换） | [19](19_Action与通信机制.md) | pending goal |
| 桥接模式 | Bridge Pattern | 算法核心与 ROS 封装分离（NavFn/pf 库与其 ROS 外壳） | [22](22_设计模式专题.md) | 设计模式 |
| QoS | Quality of Service | 通信质量策略（可靠性/历史/持久性），收发两端必须兼容 | [19](19_Action与通信机制.md) | DDS、latched QoS |

## R

| 术语 | 英文 | 一句话定义 | 首次出现章节 | 相关概念 |
|---|---|---|---|---|
| RAII | Resource Acquisition Is Initialization | 资源生命周期绑定对象作用域的 C++ 惯用法 | [23](23_现代C++专题.md) | 智能指针 |
| RateController | Rate Controller | 限制子树 tick 频率的 BT 装饰节点（周期性重规划的实现者） | [11](11_行为树与BTNavigator.md) | 装饰器模式 |
| raytrace 清除 | Raytrace Clearing | 沿传感器到命中点的射线把途经栅格清为空闲 | [06](06_Costmap2D.md) | ObstacleLayer |
| rclcpp_action | rclcpp_action | ROS 2 Action 的 C++ 客户端/服务端底层库 | [19](19_Action与通信机制.md) | SimpleActionServer |
| rclcpp_lifecycle | rclcpp_lifecycle | ROS 2 生命周期节点标准状态机实现 | [05](05_生命周期管理.md) | LifecycleNode |
| RecoveryNode | Recovery Node | "主动作失败→执行恢复→重试"的 Nav2 自定义 BT 控制节点 | [11](11_行为树与BTNavigator.md) | RoundRobin |
| Reeds-Shepp 曲线 | Reeds-Shepp Curve | 允许倒车的最短曲线族，Hybrid-A* 双向模式的解析扩展 | [08](08_SmacPlanner.md) | Dubins 曲线 |
| ReroutingService | Rerouting Service | 跟踪途中请求重新路由的 RouteOperation 插件 | [17](17_Route路线规划.md) | RouteOperation |
| respawn | Respawn | launch 层面进程崩溃自动重启机制，与 bond 配合完成整体自愈 | [05](05_生命周期管理.md) | bond |
| rolling window（滚动窗口） | Rolling Window | local costmap 以机器人为中心随动的模式 | [06](06_Costmap2D.md) | local_costmap |
| RotationShimController | Rotation Shim Controller | 先原地转向对齐路径再移交内部真实控制器的装饰器控制器 | [10](10_MPPI与RPP控制器.md) | 装饰器模式 |
| RoundRobin | Round Robin | 轮转尝试子节点的 Nav2 自定义 BT 控制节点（恢复子树用） | [11](11_行为树与BTNavigator.md) | RecoveryNode |
| RoutePlanner | Route Planner | 路网上的 Dijkstra/A* 搜索器，边代价由评分器插件合成 | [17](17_Route路线规划.md) | EdgeCostFunction |
| RouteServer | Route Server | 图路由服务器：加载路网、搜索、跟踪三合一宿主 | [17](17_Route路线规划.md) | ComputeRoute |
| RouteTracker | Route Tracker | 跟踪机器人在路线上的进度并触发 Operations 的组件 | [17](17_Route路线规划.md) | RouteOperation |
| RouteOperation | Route Operation | 路线跟踪期间按节点/边触发的操作插件族 | [17](17_Route路线规划.md) | AdjustSpeedLimit |
| RPP | Regulated Pure Pursuit | 调节式纯追踪：按曲率/障碍/速度调节线速度的追踪控制器 | [10](10_MPPI与RPP控制器.md) | 前瞻距离 |
| 容差 | Tolerance | 规划/控制中允许的目标偏差（xy/yaw 等） | [07](07_PlannerServer与全局规划.md) | GoalChecker 接口 |
| RViz | RViz | ROS 可视化工具，Nav2 提供专属面板与工具插件 | [00](00_项目概览.md) | nav2_rviz_plugins |

## S

| 术语 | 英文 | 一句话定义 | 首次出现章节 | 相关概念 |
|---|---|---|---|---|
| SavitzkyGolaySmoother | Savitzky-Golay Smoother | 用 SG 滤波窗口做路径平滑的插件 | [15](15_Smoother路径平滑.md) | Smoother 接口 |
| Sequence 节点 | Sequence | 依次执行子节点、全部成功才成功的 BT 控制节点 | [11](11_行为树与BTNavigator.md) | Fallback 节点 |
| ServiceClient | nav2::ServiceClient | 带独立 executor 的同步服务客户端封装（nav2_ros_common） | [19](19_Action与通信机制.md) | invoke |
| shared_ptr | std::shared_ptr | 共享所有权智能指针，Nav2 中表达"共同持有"关系 | [23](23_现代C++专题.md) | unique_ptr、weak_ptr |
| SimpleActionServer | nav2::SimpleActionServer | Nav2 自研的单目标 + pending 抢占模型 Action 服务器（1.5.0 位于 nav2_ros_common 包、nav2:: 命名空间） | [19](19_Action与通信机制.md) | 抢占、goal handle |
| SimpleChargingDock / SimpleNonChargingDock | Simple (Non)Charging Dock | 官方坞站插件：基于检测话题/固定位姿的充电与非充电实现 | [18](18_Docking靠泊.md) | ChargingDock |
| SimpleGoalChecker | Simple Goal Checker | xy + yaw 双容差的默认目标检查器 | [09](09_ControllerServer与局部控制.md) | StoppedGoalChecker |
| SimpleProgressChecker | Simple Progress Checker | "时间窗内至少移动指定半径"的默认进度检查器 | [09](09_ControllerServer与局部控制.md) | ProgressChecker 接口 |
| SimpleSmoother | Simple Smoother | 加权平均迭代的默认路径平滑器 | [15](15_Smoother路径平滑.md) | Smoother 接口 |
| SmacPlanner | Smac Planner | 模板化 A* 规划器家族：Hybrid / 2D / Lattice 三种前端共用一个内核 | [08](08_SmacPlanner.md) | AStarAlgorithm |
| Smoother 接口 | nav2_core::Smoother | 路径平滑插件契约（smooth） | [04](04_nav2_core插件接口.md) | SmootherServer |
| SmootherServer | Smoother Server | 路径平滑宿主服务器 | [15](15_Smoother路径平滑.md) | SmoothPath |
| SpeedFilter | Speed Filter | 按掩码区域发布限速指令的过滤层 | [06](06_Costmap2D.md) | filter mask |
| Spin | Spin | 原地旋转恢复行为 | [12](12_Behaviors恢复行为.md) | TimedBehavior |
| staging pose（预备位姿） | Staging Pose | 靠泊前先导航到的粗定位点，由坞站插件给出 | [18](18_Docking靠泊.md) | DockRobot |
| State Lattice | State Lattice | 离线生成运动基元表、在线查表扩展的规划方法 | [08](08_SmacPlanner.md) | NodeLattice |
| StaticLayer | Static Layer | 订阅 /map 并写入静态障碍的层 | [06](06_Costmap2D.md) | MapServer |
| StoppedGoalChecker | Stopped Goal Checker | 位置到达且速度低于阈值才算完成的目标检查器 | [09](09_ControllerServer与局部控制.md) | SimpleGoalChecker |
| 势场 | Potential Field | NavFn 从目标向外传播的代价累积场，梯度下降即路径 | [07](07_PlannerServer与全局规划.md) | Dijkstra |
| 生命周期 | Lifecycle | unconfigured→inactive→active 的受控节点状态机 | [05](05_生命周期管理.md) | LifecycleNode |
| 数据源（碰撞监视） | Collision Monitor Source | CollisionMonitor 的传感输入抽象（scan/pointcloud/range 等） | [16](16_CollisionMonitor与VelocitySmoother.md) | Polygon |
| 速度指令三级链路 | Velocity Command Pipeline | cmd_vel_nav → velocity_smoother → cmd_vel_smoothed → collision_monitor → cmd_vel | [01](01_整体架构.md) | CollisionMonitor、VelocitySmoother |
| 索引包（三索引） | Book Indexes | 本书的类/文件/API 三份双向索引 | [ClassIndex.md](ClassIndex.md) | FileIndex、APIIndex |

## T / U / V

| 术语 | 英文 | 一句话定义 | 首次出现章节 | 相关概念 |
|---|---|---|---|---|
| TF / TF2 | Transform Framework | ROS 坐标变换体系，map/odom/base_link 树的维护与查询 | [01](01_整体架构.md) | map→odom 变换 |
| Theta* | Theta-star | 带视线检查的任意角度 A* 变体 | [07](07_PlannerServer与全局规划.md) | A* |
| tick | Tick | 行为树的周期性激励：根到叶传播执行信号并回收状态 | [11](11_行为树与BTNavigator.md) | 行为树 |
| TimedBehavior | Timed Behavior | 恢复行为的模板方法基类：固定频率循环调用 onCycleUpdate | [12](12_Behaviors恢复行为.md) | Behavior 接口 |
| track_unknown_space | Track Unknown Space | 是否把未知区域当作不可通行的开关参数 | [06](06_Costmap2D.md) | NO_INFORMATION |
| transform_tolerance | Transform Tolerance | 允许 TF 数据滞后的时间容差 | [06](06_Costmap2D.md) | TF |
| TwistPublisher / TwistSubscriber | Twist Pub/Sub | 兼容 Twist 与 TwistStamped 双格式的速度收发封装 | [09](09_ControllerServer与局部控制.md) | TwistStamped |
| TwistStamped | Twist Stamped | 带时间戳与坐标系的速度消息，1.5.0 默认速度格式 | [09](09_ControllerServer与局部控制.md) | cmd_vel |
| unique_ptr | std::unique_ptr | 独占所有权智能指针，宿主持有插件的标准姿势 | [23](23_现代C++专题.md) | shared_ptr |
| updateBounds / updateCosts | Two-Phase Update | 层更新两阶段：先各层申报脏区包围盒，再按序写代价 | [06](06_Costmap2D.md) | Layer 接口 |
| use_sim_time | use_sim_time | 使用仿真时钟 /clock 的全局参数 | [21](21_配置系统.md) | 配置系统 |
| VelocitySmoother | Velocity Smoother | 按加速度/死区约束平滑速度指令的独立节点（链路第二级） | [16](16_CollisionMonitor与VelocitySmoother.md) | cmd_vel_smoothed |
| VoxelLayer | Voxel Layer | 用 3D 体素网格做标记/清除再投影到 2D 的障碍层 | [06](06_Costmap2D.md) | ObstacleLayer |

## W / X / Y / Z

| 术语 | 英文 | 一句话定义 | 首次出现章节 | 相关概念 |
|---|---|---|---|---|
| Wait | Wait | 原地等待恢复行为 | [12](12_Behaviors恢复行为.md) | TimedBehavior |
| WaitAtWaypoint | Wait At Waypoint | 航点处停留指定时长的任务执行器插件 | [19](19_Action与通信机制.md) | WaypointTaskExecutor |
| WaypointFollower | Waypoint Follower | 航点巡航服务器，逐点调用 NavigateToPose 并执行航点任务 | [19](19_Action与通信机制.md) | FollowWaypoints |
| WaypointTaskExecutor | nav2_core::WaypointTaskExecutor | 到达航点后执行任务的插件契约 | [04](04_nav2_core插件接口.md) | WaitAtWaypoint |
| weak_ptr | std::weak_ptr | 不拥有对象的观察指针，插件回指宿主时打破引用环 | [23](23_现代C++专题.md) | shared_ptr |
| worldToMap / mapToWorld | World/Map Conversion | 世界坐标（米）与栅格坐标（格）互转的基础方法 | [06](06_Costmap2D.md) | Costmap2D |
| 外观模式 | Facade Pattern | 把子系统折叠成单一门面（BasicNavigator、Costmap2DROS） | [22](22_设计模式专题.md) | 设计模式 |
| 系统测试 | System Tests | nav2_system_tests 的端到端集成测试集 | [02](02_目录设计.md) | CI |
| 现代 C++ | Modern C++ | C++17/20 语言特性在 Nav2 中的工程化使用 | [23](23_现代C++专题.md) | RAII、智能指针 |
| 线程模型 | Threading Model | 各服务器的线程分布、共享数据与锁的全景 | [20](20_线程模型.md) | Executor、mutex |
| 协作式取消 | Cooperative Cancellation | 取消/抢占依赖执行循环主动轮询标志位，而非强制杀线程 | [19](19_Action与通信机制.md) | 抢占 |
| 行为树 | Behavior Tree | 用树形控制流组织任务逻辑的编排模型，逻辑外置为 XML | [11](11_行为树与BTNavigator.md) | tick、黑板 |
| XML 行为树 | BT XML | 行为树的声明式描述文件，改逻辑不改代码的落点 | [11](11_行为树与BTNavigator.md) | Groot |
| 依赖注入 | Dependency Injection | 插件所需资源全部经 configure 参数注入而非自行获取 | [22](22_设计模式专题.md) | 单例（被拒绝） |
| 运动基元 | Motion Primitive | Hybrid-A*/Lattice 扩展邻居时使用的短运动片段 | [08](08_SmacPlanner.md) | Hybrid-A* |
| 责任链模式 | Chain of Responsibility | critic 评分管线的模式本质：逐个处理者累加意见 | [22](22_设计模式专题.md) | critic |
| 中介者模式 | Mediator Pattern | lifecycle_manager 与 NavigatorMuxer 集中协调多方的模式落点 | [22](22_设计模式专题.md) | lifecycle_manager |
| 智能指针 | Smart Pointers | unique/shared/weak 三种所有权语义的指针家族 | [23](23_现代C++专题.md) | RAII |
| 装饰器模式 | Decorator Pattern | 不改原类叠加职责（RotationShim、BT 装饰节点） | [22](22_设计模式专题.md) | RotationShimController |
| 状态模式 | State Pattern | 生命周期状态机与靠泊阶段推进的模式本质 | [22](22_设计模式专题.md) | 生命周期 |
| 组合模式 | Composite Pattern | BT 容器与叶节点同构嵌套的模式本质 | [22](22_设计模式专题.md) | 行为树 |
| 自适应重采样 | Adaptive Resampling | AMCL 按权重退化程度决定是否重采样、按 KLD 决定粒子数 | [13](13_AMCL定位.md) | KLD 采样 |
| 坐标系约定 | Frame Conventions | map（全局）/ odom（连续）/ base_link（本体）三级坐标系 | [01](01_整体架构.md) | TF |
