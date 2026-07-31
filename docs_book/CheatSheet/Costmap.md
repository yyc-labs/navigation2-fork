# 速查卡：Costmap2D

> 口径：Nav2 1.5.0（main 分支）。层清单核实自 `nav2_costmap_2d/costmap_plugins.xml`，代价值核实自 `nav2_costmap_2d/include/nav2_costmap_2d/cost_values.hpp`。详解见 [第 06 章](../06_Costmap2D.md)。

## 什么时候打开本速查

- 改 YAML 想不起某个 Layer 的插件类名或参数名；
- 看到栅格值 253/254/255 拿不准各自语义；
- 新机器人接入，要快速拼一份 local/global costmap 配置骨架；
- 分不清 `plugins` 与 `filters` 两个挂载键该放什么；
- 想确认膨胀层参数（`inflation_radius`/`cost_scaling_factor`）名字与默认量级。

## 1. 分层结构一图流

```
观测源(scan/pointcloud) ─> 各 Layer 独立维护 ─> LayeredCostmap 逐层 updateCosts 合成 master 栅格
                                                          │
Costmap2DROS（生命周期宿主/TF/更新线程） ──────────────────┘─> 发布 costmap / costmap_raw
```

- 挂载键 `plugins`：常规层，参与 master 合成；挂载键 `filters`：代价地图滤镜（Costmap Filter），基于滤镜掩膜（mask）后处理。
- 更新循环：`update_frequency` 频率下先 `updateBounds` 再 `updateCosts`；读写 master 栅格须持 `Costmap2D` 的互斥锁。详见 [第 06 章](../06_Costmap2D.md)。

## 2. 常用 Layer 与关键参数

插件类名一律带 `nav2_costmap_2d::` 前缀（核实自 costmap_plugins.xml）：

| 层 | 插件类名 | 用途 | 关键参数（YAML 内已核实） |
|---|---|---|---|
| 静态层 | `StaticLayer` | 吸收 map_server 地图 | `map_subscribe_transient_local` |
| 障碍层 | `ObstacleLayer` | 2D 标记/清除障碍 | `observation_sources` 及每源 `topic` `data_type` `marking` `clearing` `obstacle_max_range` `raytrace_max_range` |
| 体素层 | `VoxelLayer` | 3D 体素投影障碍 | 障碍层参数 + `z_resolution` `z_voxels` `mark_threshold` `publish_voxel_map` |
| 膨胀层 | `InflationLayer` | 代价梯度膨胀 | `inflation_radius`（默认档 0.7）`cost_scaling_factor`（默认档 3.0） |
| 传统膨胀层 | `LegacyInflationLayer` | 旧版 CellData 算法 | 同上 |
| 非对称膨胀层 | `AsymmetricInflationLayer` | 沿全局路径非对称膨胀 | 同上系 |
| 距离传感层 | `RangeSensorLayer` | 声呐/红外 | — |
| 去噪层 | `DenoiseLayer` | 滤除孤立噪点障碍 | — |
| 容器层 | `PluginContainerLayer` | 层分组复用 | — |
| 禁行滤镜 | `KeepoutFilter` | 禁行区 | `filter_info_topic` `override_lethal_cost` `lethal_override_cost` |
| 限速滤镜 | `SpeedFilter` | 限速区 | `filter_info_topic` `speed_limit_topic` |
| 开关滤镜 | `BinaryFilter` | 区域二值开关 | — |

各层算法细节见 [第 06 章](../06_Costmap2D.md)；SpeedFilter 与控制器 `setSpeedLimit` 的联动见 [第 04 章](../04_nav2_core插件接口.md)。

## 3. 代价值语义（cost_values.hpp，约 70 行处）

| 值 | 常量 | 语义 |
|---|---|---|
| 255 | `NO_INFORMATION` | 未知区域 |
| 254 | `LETHAL_OBSTACLE` | 致命障碍（栅格内有障碍物本体） |
| 253 | `INSCRIBED_INFLATED_OBSTACLE` | 内切致命（机器人中心到此必碰撞） |
| 252 | `MAX_NON_OBSTACLE` | 非障碍代价最大值 |
| 1~252 | — | 膨胀衰减梯度（离障碍越远越小） |
| 0 | `FREE_SPACE` | 自由空间 |

记忆锚点：**254 是"物体在这"，253 是"中心在这必撞"，255 是"不知道"**。

## 4. 常用 YAML 骨架（摘自 `nav2_bringup/params/nav2_params.yaml`）

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true          # 局部图跟随机器人滚动
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22            # 或用 footprint: "[[x,y],...]"
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.70

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      track_unknown_space: true     # 未知区按 NO_INFORMATION 记账
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      filters: ["keepout_filter", "speed_filter"]
```

双 costmap 命名空间嵌套（`local_costmap.local_costmap`）是 Costmap2DROS 作为子节点的产物，解释见 [第 06 章](../06_Costmap2D.md)；参数系统全貌见 [第 21 章](../21_配置系统.md)。
