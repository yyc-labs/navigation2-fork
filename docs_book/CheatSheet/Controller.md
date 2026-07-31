# 速查卡：局部控制器选型

> 口径：Nav2 1.5.0（main 分支）。参数名核实自各控制器参数声明源码与 `nav2_bringup/params/nav2_params.yaml`。详解见 [第 09 章](../09_ControllerServer与局部控制.md)、[第 10 章](../10_MPPI与RPP控制器.md)。

## 什么时候打开本速查

- 新项目选控制器，想快速对比五个内置实现的风格与算力开销；
- 改 YAML 想不起 `lookahead_dist`、`vx_std`、`k_phi` 这类参数属于谁；
- 机器人到点后原地画龙/不肯转身，想确认旋转收尾该配哪个开关；
- 想给 MPPI 配 critics 列表却记不全名字；
- 拿不准 Rotation Shim 怎么"套"在主控制器外面。

## 1. 五控制器对比

均实现 `nav2_core::Controller`，挂载键 `controller_plugins`（controller_server）。插件类名核实自各包 plugin xml：

| 控制器 | 插件类名 | 算法风格 | 适用 | 算力 | 特点 |
|---|---|---|---|---|---|
| DWB | `dwb_core::DWBLocalPlanner` | 速度采样 + Critic 打分 | 差动/全向 | 中 | 评分器全插件化，可解释性强 |
| RPP | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` | 纯追踪 + 曲率/代价调速 | 差动/阿克曼 | 低 | 精确贴线，工业场景常用 |
| MPPI | `nav2_mppi_controller::MPPIController` | 采样型模型预测（批量轨迹 + softmax 加权） | 差动/全向/阿克曼 | 高 | 动态避障与约束表达最强，1.5.0 默认 |
| Graceful | `nav2_graceful_controller::GracefulController` | 李雅普诺夫收敛控制律 | 差动 | 低 | 平滑到点，靠泊 controller 同源 |
| Rotation Shim | `nav2_rotation_shim_controller::RotationShimController` | 装饰器：先原地转向再移交 | 套任意主控制器 | 低 | 解决"起步背对路径"画龙问题 |

ControllerServer 主循环（ProgressChecker/GoalChecker/PathHandler 协作）见 [第 09 章](../09_ControllerServer与局部控制.md)。

## 2. 关键参数速查（含源码核实的默认值）

### MPPI（默认档核实自 `nav2_bringup/params/nav2_params.yaml`）

| 参数 | 默认档 | 含义 |
|---|---|---|
| `time_steps` / `model_dt` | 56 / 0.05 | 预测时域 = 56×0.05 s = 2.8 s |
| `batch_size` | 2000 | 每轮采样轨迹数 |
| `vx_std` / `vy_std` / `wz_std` | 0.2 / 0.2 / 0.4 | 采样噪声标准差 |
| `vx_max` / `vx_min` / `wz_max` | 0.5 / -0.35 / 1.9 | 速度盒约束 |
| `iteration_count` / `temperature` / `gamma` | 1 / 0.3 / 0.015 | 优化迭代与 softmax 温度 |
| `motion_model` | "diff_drive" | diff_drive / omni / ackermann 等模型插件 |
| `critics` | 8 项列表 | `ConstraintCritic` `CostCritic` `GoalCritic` `GoalAngleCritic` `PathAlignCritic` `PathFollowCritic` `PathAngleCritic` `PreferForwardCritic`（另有 `TwirlingCritic` 可选） |

调参思路（先权重后噪声）见 [第 10 章](../10_MPPI与RPP控制器.md)。

### RPP（`nav2_regulated_pure_pursuit_controller/src/parameter_handler.cpp`，约 37 行起）

| 参数 | 默认 | 含义 |
|---|---|---|
| `max_linear_vel`（旧名 `desired_linear_vel`） | — | 巡航线速度 |
| `lookahead_dist` | 0.6 | 前视距离 (m) |
| `min_lookahead_dist` / `max_lookahead_dist` / `lookahead_time` | 0.3 / 0.9 / 1.5 | `use_velocity_scaled_lookahead_dist: true` 时按速度伸缩 |
| `use_regulated_linear_velocity_scaling` | true | 曲率调速（弯道减速） |
| `use_cost_regulated_linear_velocity_scaling` | true | 近障调速 |
| `regulated_linear_scaling_min_radius` | 0.90 | 触发曲率调速的半径 |
| `use_rotate_to_heading` / `rotate_to_heading_min_angle` | true / 0.785 | 起步先原地转向 |
| `rotate_to_heading_angular_vel` | 1.8 | 原地转向角速度 |
| `min_approach_linear_velocity` / `approach_velocity_scaling_dist` | 0.05 / 0.6 | 到点减速 |
| `allow_reversing` | false | 允许倒车段 |
| `max_allowed_time_to_collision_up_to_carrot` | 1.0 | 前视碰撞检查时域 (s) |

### DWB（参数散布于 dwb_plugins/dwb_core，见 `kinematic_parameters.cpp` 约 73 行起、`xy_theta_iterator.cpp`、`standard_traj_generator.cpp`、`dwb_local_planner.cpp`）

| 参数 | 默认 | 含义 |
|---|---|---|
| `min_vel_x` / `max_vel_x` / `max_vel_theta` | 0.0 | 速度界（必配） |
| `min_speed_xy` / `max_speed_xy` | 0.0 | 平移速率界 |
| `acc_lim_x` 等 | 0.0 | 加速度界（必配） |
| `vx_samples` / `vtheta_samples` | 20 / 20 | 采样密度 |
| `sim_time` | 1.7 | 轨迹前推时长 (s) |
| `critics` | 必配 | 评分器列表（如 `RotateToGoal` `Oscillation` `ObstacleFootprint` `GoalAlign` `PathAlign` `PathDist` `GoalDist`） |

### Graceful（`nav2_graceful_controller/src/parameter_handler.cpp`，约 37 行起）

| 参数 | 默认 | 含义 |
|---|---|---|
| `k_phi` / `k_delta` / `beta` / `lambda` | 2.0 / 1.0 / 0.4 / 2.0 | 控制律增益 |
| `v_linear_min` / `v_linear_max` / `v_angular_max` | 0.1 / 0.5 / 1.0 | 速度界 |
| `min_lookahead` / `max_lookahead` | 0.25 / 1.0 | 前视目标窗口 |
| `slowdown_radius` | 1.5 | 减速半径 (m) |
| `initial_rotation` / `initial_rotation_tolerance` | true / 0.75 | 起步原地转向 |
| `prefer_final_rotation` | true | 到点后再对朝向 |

### Rotation Shim（`nav2_rotation_shim_controller/src/parameter_handler.cpp`，约 40 行起）

| 参数 | 默认 | 含义 |
|---|---|---|
| `primary_controller.plugin` | 必配 | 被包裹的主控制器类型字符串 |
| `angular_dist_threshold` | 0.785 | 偏航差超过即进入原地转向 (rad) |
| `angular_disengage_threshold` | 0.3925 | 退出转向阈值 |
| `rotate_to_heading_angular_vel` / `max_angular_accel` | 1.8 / — | 转向速度/加速度 |
| `forward_sampling_distance` | 0.5 | 取路径采样点估计朝向 (m) |
| `rotate_to_goal_heading` | false | 到点后也接管旋转对准 |
| `simulate_ahead_time` | 1.0 | 旋转碰撞预演时长 (s) |

## 3. 选型口诀

> **贴线巡逻 RPP 稳，动态人群 MPPI 神；**
> **老派可调选 DWB，靠泊到点 Graceful 温；**
> **起步背对先转身——Shim 一套谁都行；**
> **算力紧张往前排，算力富余往后奔。**

- 仓储固定路线、低算力 → RPP（配 Rotation Shim 更稳）；
- 动态障碍多、底盘约束复杂 → MPPI（默认选择）；
- 需要逐项定制评分逻辑 → DWB；
- 差动到点姿态要优雅 → Graceful；
- 任何主控制器 + 原地转向补丁 → Rotation Shim 装饰器（[第 22 章](../22_设计模式专题.md) 装饰器模式一节）。

MPPI/RPP 深拆见 [第 10 章](../10_MPPI与RPP控制器.md)；输出链路下游（速度平滑与碰撞看门狗）见 [第 16 章](../16_CollisionMonitor与VelocitySmoother.md)。
