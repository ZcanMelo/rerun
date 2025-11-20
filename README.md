# DoraXRerun: 自动驾驶可视化

基于Rerun C++ SDK的自动驾驶数据可视化项目，支持点云、路径规划、车辆位姿等多种数据的实时3D可视化。

## 功能特性

- 🚗 **车辆位姿可视化** - 显示自车位置和朝向的3D箭头
- 📡 **激光雷达点云** - 实时显示雷达点云数据
- 🛣️ **路径规划显示** - 支持中心参考线、车道线等路径可视化  
- 📊 **文本信息日志** - 实时显示车辆状态信息
- 🧊 **目标聚类显示** - 3D立方体显示聚类目标

## 安装依赖

### Rerun C++ SDK

请参考安装教程：[从零开始学Rerun(C++)](https://blog.csdn.net/candygua/article/details/146050143)


## 代码示例

### 点云显示（雷达点云）

```bash
std::vector<rerun::Position3D> rerun_points; // 存储雷达点云的3D坐标
// （需自行填充点云数据到rerun_points）
rec.log("live_points", rerun::Points3D(rerun_points)
    .with_colors(0x00FF00FF) // 颜色：绿色（RGBA）
    .with_radii({0.02f}) // 点半径
);
```

### 路径线显示

```bash
std::vector<rerun::Position3D> points; // 存储线路的节点坐标
// （需自行填充线路节点数据到points）
std::vector<rerun::LineStrip3D> line_strips = {rerun::LineStrip3D(points)};
rec.log("path_points", rerun::LineStrips3D(line_strips)
    .with_colors(0x0000FFFF) // 颜色：蓝色（RGBA）
    .with_radii({0.08f}) // 线半径
);
```

### 车辆位姿信息

```bash
// pose为车辆位姿数据结构体（需自行定义并赋值）
rec.log(
    "vehicle/position_log",
    rerun::TextLog("x: " + std::to_string(pose.x) 
        + "  y: " + std::to_string(pose.y) 
        + "  theta: " + std::to_string(pose.theta) 
        + "  s: " + std::to_string(pose.s))
    .with_level(rerun::TextLogLevel::Info) // 日志级别：信息
);
```

### 自车位姿箭头

```bash
std::vector<rerun::Position3D> origins; // 箭头起点（车辆位置）
std::vector<rerun::Vector3D> vectors; // 箭头方向向量
// pose为车辆位姿数据结构体（需自行定义并赋值）
float angle = pose.theta * (M_PI / 180.0f); // 角度转换：度→弧度
float length = 1.5f; // 箭头长度
origins.push_back({pose.x, pose.y, -1.0}); // 车辆3D位置（z轴固定为-1.0）
vectors.push_back({length * cosf(angle), length * sinf(angle), 0.0}); // 箭头朝向
rec.log(
    "arrows",
    rerun::Arrows3D::from_vectors(vectors)
        .with_origins(origins)
        .with_colors(0xFF00FFFF) // 颜色：紫色（RGBA）
);
```

### 目标聚类立方体

```bash
std::vector<rerun::Vec3D> centers; // 立方体中心点坐标
std::vector<rerun::Vec3D> half_sizes; // 立方体长宽高的一半（需自行计算赋值）
std::vector<rerun::Quaternion> quaternions; // 立方体旋转四元数（需自行计算赋值）
// （需自行填充聚类目标的中心、尺寸、旋转数据）
rec.log(
    "lidar/clusters",
    rerun::Boxes3D::from_centers_and_half_sizes(centers, half_sizes)
        .with_quaternions(quaternions) // 应用旋转
        .with_fill_mode(rerun::FillMode::MajorWireframe) // 显示模式：主要线框
);
```

## 效果展示

[Dora×Rerun：自动驾驶可视化](https://www.bilibili.com/video/BV1pEyFBjEx1/)

## 详细教程

完整的使用教程和API说明请参考：
- [从零开始学Rerun(C++) - CSDN博客](https://blog.csdn.net/candygua/category_12994576.html)


