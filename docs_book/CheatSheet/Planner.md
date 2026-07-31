# 速查卡：全局规划器选型

> 口径：Nav2 1.5.0（main 分支）。参数名核实自各规划器参数声明源码。详解见 [第 07 章](../07_PlannerServer与全局规划.md)、[第 08 章](../08_SmacPlanner.md)。

## 什么时候打开本速查

- 新项目选规划器，想快速对比五个内置实现的适用车型；
- 改 YAML 想不起 `motion_model_for_search`、`minimum_turning_radius` 等参数名；
- 规划耗时/失败，想核对 `max_iterations`、`max_planning_time` 这类预算参数默认值；
- 差动改阿克曼底盘，拿不准该从 NavFn 换到哪个 Smac 变体。

## 1. 五规划器对比

均实现 `nav2_core::GlobalPlanner`，挂载键 `planner_plugins`（planner_server）。插件类名核实自各包 plugin xml：

| 规划器 | 插件类名 | 算法 | 搜索空间 | 适用车型 | 特点 |
|---|---|---|---|---|---|
| NavFn | `nav2_navfn_planner::NavfnPlanner` | Dijkstra / A*（`use_astar`） | 2D 栅格 | 圆形近似的差动/全向 | 最经典稳妥，路径不带朝向 |
| Smac 2D | `nav2_smac_planner::SmacPlanner2D` | 代价感知 A* | 2D 栅格 | 差动/全向 | 支持降采样加速、代价乘子调形 |
| Smac Hybrid | `nav2_smac_planner::SmacPlannerHybrid` | Hybrid-A*（Dubin/Reeds-Shepp 原语） | SE2（x, y, θ） | 阿克曼/类车、需运动学可行 | 带朝向的平滑可行路径 |
| Smac Lattice | `nav2_smac_planner::SmacPlannerLattice` | 状态点阵 A*（离线原语文件） | SE2 | 差动/全向/任意底盘（非圆形足迹） | `lattice_filepath` 指定原语，支持原地旋转惩罚 |
| Theta* | `nav2_theta_star_planner::ThetaStarPlanner` | 任意角度（any-angle）A* + 视线检查 | 2D 栅格 | 差动/全向 | 折线少、路径更直 |

PlannerServer 的调度与异常→错误码链路见 [第 07 章](../07_PlannerServer与全局规划.md)。

## 2. 关键参数速查（含源码核实的默认值）

### NavFn（`nav2_navfn_planner/src/parameter_handler.cpp`，约 38 行起）

| 参数 | 默认 | 含义 |
|---|---|---|
| `tolerance` | 0.5 | 目标不可达时的就近容差 (m) |
| `use_astar` | false | true 用 A*，false 用 Dijkstra |
| `allow_unknown` | true | 允许穿越未知区 |
| `use_final_approach_orientation` | false | 末段朝向按接近方向取值 |

### Smac 2D（`nav2_smac_planner/include/nav2_smac_planner/smac_planner_2d_impl.hpp`，configure 参数段约 73 行起）

| 参数 | 默认 | 含义 |
|---|---|---|
| `tolerance` | 0.125 | 就近容差 (m) |
| `downsample_costmap` / `downsampling_factor` | false / 1 | 降采样加速 |
| `cost_travel_multiplier` | 1.0 | 代价项权重（越大越躲墙） |
| `max_iterations` | 1000000 | 搜索迭代上限（-1 不限） |
| `max_on_approach_iterations` | 1000 | 进入容差圈后的收尾迭代 |
| `max_planning_time` | 2.0 | 时间预算 (s) |

### Smac Hybrid(`smac_planner_hybrid_impl.hpp`，约 80 行起）

| 参数 | 默认 | 含义 |
|---|---|---|
| `motion_model_for_search` | "DUBIN" | 原语模型；倒车选 "REEDS_SHEPP" |
| `minimum_turning_radius` | 0.4 | 最小转弯半径 (m)，**必按车改** |
| `angle_quantization_bins` | 72 | 朝向离散 bin 数 |
| `reverse_penalty` | 2.0 | 倒车惩罚（REEDS_SHEPP 生效） |
| `change_penalty` / `non_straight_penalty` | 0.0 / 1.2 | 换向/转弯惩罚 |
| `smooth_path` | true | 内置平滑器后处理 |
| `cache_obstacle_heuristic` | false | 静态环境可开启加速 |
| `max_planning_time` | 5.0 | 时间预算 (s) |

### Smac Lattice（`smac_planner_lattice_impl.hpp`，约 75 行起）

| 参数 | 默认 | 含义 |
|---|---|---|
| `lattice_filepath` | 内置示例原语 | 离线生成的原语 JSON，**分辨率须与 costmap 匹配** |
| `rotation_penalty` | 5.0 | 原地旋转惩罚（差动特有） |
| `retrospective_penalty` | 0.015 | 后视惩罚（偏好靠后代价） |
| `cost_penalty` | 2.0 | 代价项权重 |
| `analytic_expansion_ratio` | 3.5 | 解析扩张频率 |
| `change_penalty` / `non_straight_penalty` | 0.05 / 1.05 | 换向/转弯惩罚 |

### Theta*（`nav2_theta_star_planner/src/parameter_handler.cpp`，约 38 行起）

| 参数 | 默认 | 含义 |
|---|---|---|
| `how_many_corners` | 8 | 邻域连通数（4 或 8） |
| `w_euc_cost` | 1.0 | 欧氏距离权重 |
| `w_traversal_cost` | 2.0 | 通行代价权重 |
| `allow_unknown` | true | 允许穿越未知区 |

Smac 家族搜索框架（A* 模板 + Node 模型 + 解析扩张 + 平滑）逐层拆解见 [第 08 章](../08_SmacPlanner.md)。

## 3. 选型口诀

> **圆车省心用 NavFn，躲墙调形 Smac 2D；**
> **类车倒库 Hybrid 转（Dubin 前进、RS 倒车）；**
> **异形底盘上 Lattice，走廊直线 Theta\* 见；**
> **先按车型定家族，再拿半径、惩罚细调线。**

- 只有圆形足迹 + 不在乎路径朝向 → NavFn / Smac 2D / Theta*；
- 有最小转弯半径约束（阿克曼、牵引车）→ Smac Hybrid；
- 非圆形足迹或要精确差动/全向运动学 → Smac Lattice；
- 路径经常被平滑器救场 → 直接换带朝向的 Smac 家族，配套平滑见 [第 15 章](../15_Smoother路径平滑.md)。
