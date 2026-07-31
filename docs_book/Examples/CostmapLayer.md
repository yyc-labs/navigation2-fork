# 实战示例：从零实现一个自定义禁区层（Costmap Layer）

> 测试口径：Nav2 1.5.0（main 分支），C++20。
> 相关章节：[06 Costmap2D](../06_Costmap2D.md) · [04 nav2_core 插件接口](../04_nav2_core插件接口.md) · [24 扩展开发指南](../24_扩展开发指南.md)

## 1. 目标

做什么：实现一个禁区层插件 `my_nav2_examples::NoGoZoneLayer`——从 YAML 参数读取若干矩形禁区（世界坐标 `[x_min, y_min, x_max, y_max]`），把它们以 `LETHAL_OBSTACLE` 写入代价地图，让规划器绕开这些区域。等价于一个"低配版" KeepoutFilter（官方 KeepoutFilter 用 mask 地图 + filter_info 话题，本例用静态参数，聚焦层插件机制本身）。

学到什么：

- `nav2_costmap_2d::Layer` 的 4 个纯虚函数（`reset` / `isClearable` / `updateBounds` / `updateCosts`）与 `onInitialize` 钩子；
- `CostmapLayer` 中间基类的价值：自带一块与主图同尺寸的私有 `Costmap2D`，配合 `updateWithMax` 系列方法合并到主图；
- 分层代价地图的两阶段更新协议（bounds 汇总 → costs 写入），出自 Lu et al., IROS 2014 分层代价地图论文。

## 2. 最小实现面：必须 override 的纯虚函数

以下签名逐字核实自 `nav2_costmap_2d/include/nav2_costmap_2d/layer.hpp`（约 209 行）：

| 纯虚函数 | 说明 |
|---|---|
| `virtual void reset() = 0;` | 整层复位（如清空并重画） |
| `virtual bool isClearable() = 0;` | 恢复行为 ClearCostmap 是否允许清除本层；禁区层必须返回 `false` |
| `virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x, double * max_y) = 0;` | 每周期第一阶段：扩张本层需要重绘的世界坐标包围盒 |
| `virtual void updateCosts(Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) = 0;` | 每周期第二阶段：把本层代价写入主图的指定栅格窗口 |

非纯虚、常用 override（同头文件）：`protected: virtual void onInitialize() {}`（初始化钩子，`tf_`、`name_`、`layered_costmap_` 此时已就绪）、`virtual void matchSize() {}`（主图尺寸变化时回调；`CostmapLayer` 已提供实现 `virtual void matchSize();`，见 `nav2_costmap_2d/include/nav2_costmap_2d/costmap_layer.hpp`，约 203 行）、`virtual void activate() {}`、`virtual void deactivate() {}`、`virtual void onFootprintChanged() {}`。

基类现成可用的保护成员（核实自 layer.hpp）：`layered_costmap_`、`name_`、`tf_`（`nav2::TransformBuffer *`）、`node_`（`nav2::LifecycleNode::WeakPtr`）、`logger_`、`clock_`、`current_`、`enabled_`。

## 3. 完整代码

工程布局（独立 ament 包 `my_nav2_examples_costmap`）：

```
my_nav2_examples_costmap/
├── include/my_nav2_examples_costmap/no_go_zone_layer.hpp
├── src/no_go_zone_layer.cpp
├── no_go_zone_layer_plugin.xml
├── CMakeLists.txt
└── package.xml
```

### 3.1 no_go_zone_layer.hpp

```cpp
#ifndef MY_NAV2_EXAMPLES_COSTMAP__NO_GO_ZONE_LAYER_HPP_
#define MY_NAV2_EXAMPLES_COSTMAP__NO_GO_ZONE_LAYER_HPP_

#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_layer.hpp"   // 中间基类：Layer + 私有 Costmap2D
#include "nav2_costmap_2d/cost_values.hpp"     // LETHAL_OBSTACLE / NO_INFORMATION / FREE_SPACE
#include "rclcpp/rclcpp.hpp"

namespace my_nav2_examples
{

/// 一个矩形禁区（世界坐标，单位米）
struct Zone
{
  double x_min, y_min, x_max, y_max;
};

/**
 * @class NoGoZoneLayer
 * @brief 教学用禁区层：把参数配置的矩形区域标为致命障碍。
 *        继承 CostmapLayer 以获得私有层内 Costmap2D 与 updateWithMax 合并能力。
 */
class NoGoZoneLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  NoGoZoneLayer() = default;
  ~NoGoZoneLayer() override = default;

  // ---- Layer 的 4 个纯虚函数 ----
  void reset() override;
  bool isClearable() override {return false;}   // 禁区不允许被清障行为擦除

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  // ---- 常用钩子 ----
  void onInitialize() override;   // 读参数、首次画禁区
  void matchSize() override;      // 主图尺寸变化时：调整本层尺寸并重画

private:
  /// 把所有禁区矩形光栅化到本层私有 costmap（costmap_ 继承自 Costmap2D）
  void rasterizeZones();

  std::vector<Zone> zones_;
  bool need_recalculation_{true};  // 首帧或尺寸变化后需要全图重绘
};

}  // namespace my_nav2_examples

#endif  // MY_NAV2_EXAMPLES_COSTMAP__NO_GO_ZONE_LAYER_HPP_
```

### 3.2 no_go_zone_layer.cpp

```cpp
#include <algorithm>
#include <vector>

#include "my_nav2_examples_costmap/no_go_zone_layer.hpp"

namespace my_nav2_examples
{

void NoGoZoneLayer::onInitialize()
{
  // 基类约定：onInitialize 被调用时 node_ / name_ / layered_costmap_ 均已就绪
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"NoGoZoneLayer: 无法锁定宿主节点"};
  }

  // 参数全名形如 global_costmap.global_costmap.no_go_zone_layer.enabled
  // 官方层插件（如 StaticLayer）同样用 declare_or_get_parameter 读取
  enabled_ = node->declare_or_get_parameter(name_ + "." + "enabled", true);

  // 禁区用扁平数组表达：每 4 个数一个矩形 [x_min, y_min, x_max, y_max]
  const std::vector<double> flat = node->declare_or_get_parameter(
    name_ + "." + "zones", std::vector<double>{});
  if (flat.size() % 4 != 0) {
    throw std::runtime_error{"NoGoZoneLayer: zones 长度必须是 4 的倍数"};
  }
  for (size_t i = 0; i + 3 < flat.size(); i += 4) {
    zones_.push_back({flat[i], flat[i + 1], flat[i + 2], flat[i + 3]});
  }

  // 本层底色设为 NO_INFORMATION：配合 updateWithMax 不会污染主图。
  // 注意 Costmap2D 构造默认值是 0（FREE_SPACE），必须显式改（签名核实自
  // costmap_2d.hpp：void setDefaultValue(unsigned char c)）
  setDefaultValue(nav2_costmap_2d::NO_INFORMATION);

  // CostmapLayer::matchSize 让本层私有 costmap 与主图对齐（尺寸/分辨率/原点）
  NoGoZoneLayer::matchSize();
  current_ = true;   // 本层数据不依赖传感器，恒为最新

  RCLCPP_INFO(logger_, "禁区层 %s 加载了 %zu 个矩形禁区", name_.c_str(), zones_.size());
}

void NoGoZoneLayer::matchSize()
{
  nav2_costmap_2d::CostmapLayer::matchSize();  // 重设本层网格尺寸
  rasterizeZones();                            // 尺寸变了必须重画
  need_recalculation_ = true;                  // 下个周期把全图范围报给 updateBounds
}

void NoGoZoneLayer::reset()
{
  rasterizeZones();
  need_recalculation_ = true;
  current_ = true;
}

void NoGoZoneLayer::rasterizeZones()
{
  resetMaps();   // Costmap2D 提供：全图置 default_value_（已在 onInitialize 设为 NO_INFORMATION）

  for (const auto & z : zones_) {
    unsigned int mx0, my0, mx1, my1;
    // worldToMap 越界返回 false：禁区可能部分在图外，做夹取处理
    // 签名核实自 costmap_2d.hpp：bool worldToMap(double, double, unsigned int &, unsigned int &) const
    if (!worldToMap(z.x_min, z.y_min, mx0, my0)) {
      mx0 = 0; my0 = 0;   // 左下角越界则夹到图内（教学简化，严谨做法逐边裁剪）
    }
    if (!worldToMap(z.x_max, z.y_max, mx1, my1)) {
      mx1 = getSizeInCellsX() - 1;
      my1 = getSizeInCellsY() - 1;
    }
    for (unsigned int my = my0; my <= my1; ++my) {
      for (unsigned int mx = mx0; mx <= mx1; ++mx) {
        setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }
}

void NoGoZoneLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  if (!enabled_) {
    return;
  }

  if (need_recalculation_) {
    // 首帧/重置后：把所有禁区角点并入包围盒，保证 updateCosts 覆盖到
    for (const auto & z : zones_) {
      // touch 是 CostmapLayer 辅助函数：扩张 min/max 包围盒
      touch(z.x_min, z.y_min, min_x, min_y, max_x, max_y);
      touch(z.x_max, z.y_max, min_x, min_y, max_x, max_y);
    }
    need_recalculation_ = false;
  }
  // 静态禁区之后无需再扩张 bounds；动态层（如障碍层）每周期都要 touch 新观测
}

void NoGoZoneLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  // 取本层与主图逐格最大值：LETHAL 必胜出，NO_INFORMATION 不覆盖主图
  // （继承自 CostmapLayer 的保护方法，签名见 costmap_layer.hpp）
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

}  // namespace my_nav2_examples

// pluginlib 导出宏：注意基类是 nav2_costmap_2d::Layer（不是 CostmapLayer！）
// 官方 StaticLayer / KeepoutFilter 均如此导出
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_nav2_examples::NoGoZoneLayer, nav2_costmap_2d::Layer)
```

## 4. 插件导出：plugins.xml + CMakeLists.txt + package.xml

### 4.1 no_go_zone_layer_plugin.xml

写法对照官方 `nav2_costmap_2d/costmap_plugins.xml`（`base_class_type` 一律是 `nav2_costmap_2d::Layer`）：

```xml
<class_libraries>
  <library path="my_nav2_examples_costmap">
    <class type="my_nav2_examples::NoGoZoneLayer"
           base_class_type="nav2_costmap_2d::Layer">
      <description>教学用矩形禁区层</description>
    </class>
  </library>
</class_libraries>
```

### 4.2 CMakeLists.txt 关键段

```cmake
cmake_minimum_required(VERSION 3.20)
project(my_nav2_examples_costmap)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

nav2_package()

add_library(${PROJECT_NAME} SHARED src/no_go_zone_layer.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_ros_common::nav2_ros_common
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle)
target_link_libraries(${PROJECT_NAME} PRIVATE pluginlib::pluginlib)

# 关键：层插件注册到 nav2_costmap_2d 包名下（对照官方
# nav2_costmap_2d/CMakeLists.txt 的
# pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)）
pluginlib_export_plugin_description_file(nav2_costmap_2d no_go_zone_layer_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(FILES no_go_zone_layer_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(nav2_costmap_2d nav2_ros_common rclcpp rclcpp_lifecycle)
ament_export_targets(${PROJECT_NAME})
ament_package()
```

### 4.3 package.xml 关键段

```xml
<name>my_nav2_examples_costmap</name>
<buildtool_depend>ament_cmake</buildtool_depend>
<build_depend>nav2_common</build_depend>
<depend>nav2_costmap_2d</depend>
<depend>nav2_ros_common</depend>
<depend>pluginlib</depend>
<depend>rclcpp</depend>
<depend>rclcpp_lifecycle</depend>
<export><build_type>ament_cmake</build_type></export>
```

## 5. YAML 挂载配置

把层名追加进 `plugins` 列表（顺序即叠放顺序，本层放在 inflation 之前，禁区才会被膨胀）：

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "obstacle_layer", "no_go_zone_layer", "inflation_layer"]
      no_go_zone_layer:
        plugin: "my_nav2_examples::NoGoZoneLayer"
        enabled: true
        # 两个禁区：每 4 个数一个矩形 [x_min, y_min, x_max, y_max]（map 坐标，米）
        zones: [1.0, 1.0, 2.0, 2.5,
                -3.0, 0.5, -1.5, 1.5]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
```

局部代价地图同理（`local_costmap.local_costmap.ros__parameters.plugins`）。

## 6. 运行验证

```bash
# 1. 编译
colcon build --packages-select my_nav2_examples_costmap --symlink-install
source install/setup.bash

# 2. 带自定义参数启动
ros2 launch nav2_bringup tb3_simulation_launch.py params_file:=/path/to/my_params.yaml

# 3. RViz 中把 Map 显示的话题切到 /global_costmap/costmap，
#    应看到两块紫色（致命）矩形，且外圈有膨胀渐变

# 4. 命令行确认层已挂载
ros2 param get /global_costmap/global_costmap plugins

# 5. 下发一个"穿越禁区"的目标，观察全局路径绕行
ros2 topic echo /global_costmap/costmap --once | head -20
```

## 7. 常见错误排查

1. `does not exist ... nav2_costmap_2d::Layer`——plugins.xml 的 `base_class_type` 写成了 `nav2_costmap_2d::CostmapLayer`，或 CMake 中 `pluginlib_export_plugin_description_file` 的第一个参数不是 `nav2_costmap_2d`。层插件的 pluginlib 基类只有 `Layer`。
2. 禁区显示了但机器人仍穿越——层顺序放在 `static_layer` 之前被 StaticLayer 的写入策略覆盖，或用了 `updateWithOverwrite` 让后续层覆写。教学层用 `updateWithMax` 最稳。
3. 禁区不显示——本层底色误设为 `FREE_SPACE`（0）：`updateWithMax` 下 0 永远输给主图其他值没问题，但若误用 TrueOverwrite 会把主图整块刷白；另查 `zones` 参数是否被声明为字符串（YAML 里要写成数字数组）。
4. 尺寸不匹配崩溃（越界写）——忘了在 `matchSize()` 里调用 `CostmapLayer::matchSize()`；滚动窗口的 local costmap 每次原点移动都可能触发。
5. `ClearCostmapExceptRegion` 把禁区清掉了——`isClearable()` 返回了 `true`。禁区类语义层必须返回 `false`（官方 StaticLayer 同样不可清除）。

## 8. 本示例对应源码（映射链）

| 环节 | 位置 |
|---|---|
| 示例类 | `my_nav2_examples::NoGoZoneLayer`（本文） |
| 对应官方实现 | `nav2_costmap_2d/plugins/costmap_filters/keepout_filter.cpp`（约 420 行，锚点 `KeepoutFilter::process`）；层机制更典型的参照是 `nav2_costmap_2d/plugins/static_layer.cpp`（约 590 行，锚点 `StaticLayer::onInitialize`、`StaticLayer::updateCosts`） |
| 接口头文件 | `nav2_costmap_2d/include/nav2_costmap_2d/layer.hpp`（约 209 行，4 个纯虚函数）与 `nav2_costmap_2d/include/nav2_costmap_2d/costmap_layer.hpp`（约 203 行，`updateWithMax`/`touch`/`matchSize`） |
| pluginlib 注册处 | `static_layer.cpp` 头部 `PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::StaticLayer, nav2_costmap_2d::Layer)`、`keepout_filter.cpp` 末尾 `PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::KeepoutFilter, nav2_costmap_2d::Layer)` + `nav2_costmap_2d/costmap_plugins.xml` + `nav2_costmap_2d/CMakeLists.txt` 中 `pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)` |
| 官方 YAML 例子 | `nav2_bringup/params/nav2_params.yaml` 中 `global_costmap`/`local_costmap` 段（锚点 `plugins: ["static_layer", "obstacle_layer", "inflation_layer"]` 与 `filters: ["keepout_filter"]`、`keepout_filter.plugin: "nav2_costmap_2d::KeepoutFilter"`） |

闭环路线：layer.hpp 确认两阶段更新契约 → StaticLayer 看"带私有网格的层"的完整写法（订阅、锁、滚动窗口）→ KeepoutFilter 看生产级禁区方案（mask 地图 + CostmapFilterInfo 话题）→ 用本示例骨架做自己的语义层。
