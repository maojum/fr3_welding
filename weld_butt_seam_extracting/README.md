# weld_butt_seam_extracting - 对接焊缝提取功能包

## 简介

本功能包实现了基于PCL点云处理的对接焊缝自动识别和轨迹规划算法，已从ROS1成功迁移至ROS2 Jazzy。

## 功能特性

- ✅ 自动平面分割与聚类（RANSAC）
- ✅ 基于法向量的平面分类
- ✅ 焊缝平面智能识别（距离判定）
- ✅ 平面交线计算与过滤
- ✅ 焊缝轨迹自动生成
- ✅ 焊枪位姿序列输出
- ✅ ROS2标准节点实现

## 依赖项

- ROS2 Jazzy
- PCL (Point Cloud Library)
- Eigen3
- OpenCV
- rclcpp
- sensor_msgs
- geometry_msgs
- tf2_geometry_msgs
- pcl_ros
- pcl_conversions

## 编译

```bash
cd ~/ros2_ws_jazzy
colcon build --packages-select weld_butt_seam_extracting
source install/setup.bash
```

## 使用方法

### 启动节点

```bash
ros2 run weld_butt_seam_extracting weld_butt_seam_extracting_node --ros-args -p pcd_file_path:="./fr3_description/pointcloud/weldcomponent.pcd"
```

### 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `pcd_file_path` | string | "" | PCD点云文件的绝对路径 |

### 话题

#### 发布的话题

| 话题名 | 消息类型 | 频率 | 说明 |
|--------|----------|------|------|
| `/butt_torch_pose` | geometry_msgs/msg/Pose | 1000Hz | 焊枪位姿序列 |
| `/butt_cloud_point` | sensor_msgs/msg/PointCloud2 | - | 工件点云（可选） |

## 算法流程

```
PCD点云加载
    ↓
坐标变换
    ↓
平面分割(RANSAC)
    ↓
平面聚类(法向量)
    ↓
降采样
    ↓
焊缝平面识别
    ↓
相邻平面查找
    ↓
平面旋转
    ↓
交线计算
    ↓
交线过滤
    ↓
交点计算
    ↓
轨迹生成
    ↓
位姿序列输出
```

## 算法参数

| 参数 | 值 | 说明 |
|------|-----|------|
| RANSAC距离阈值 | 0.2mm | 平面拟合容差 |
| 法向量相似阈值 | 0.1 | 平面聚类判定 |
| 降采样半径 | 3mm | 均匀采样半径 |
| 焊缝距离阈值 | 7mm | 焊缝平面判定 |
| 相邻距离阈值 | 2mm | 相邻平面判定 |
| 交点距离阈值 | 0.9mm | 交点识别容差 |

## 代码结构

```
weld_butt_seam_extracting/
├── CMakeLists.txt              # CMake构建配置
├── package.xml                 # ROS2功能包清单
├── README.md                   # 本文件
├── ROS2_MIGRATION_NOTES.md     # ROS2迁移说明
├── include/
│   └── weld_butt_seam_extracting/
│       └── weld_butt_seam_extracting_headfile.h  # 主头文件
└── src/
    ├── weld_butt.cpp           # 主程序(ROS2版本)
    ├── weld_butt_ros1_backup.cpp  # ROS1备份
    ├── SeamSepAllPlane.cpp     # 平面分割
    ├── SeamCluAllPlane.cpp     # 平面聚类
    ├── SeamDownSmape.cpp       # 降采样
    ├── SeamTwoPlane.cpp        # 焊缝平面选择
    ├── SeamDownSmapeT.cpp      # 目标平面降采样
    ├── SeamFourPlane.cpp       # 相邻平面查找
    ├── SeamFindSelf.cpp        # 自身平面排除
    ├── SeamRevolve.cpp         # 点云旋转
    ├── SeamCoefficient.cpp     # 平面系数计算
    ├── SeamIntersectLine.cpp   # 交线计算
    ├── SeamDrawLine.cpp        # 直线绘制
    ├── SeamAllIntersectPoint.cpp  # 交点计算
    ├── SeamEndShow.cpp         # 最终展示
    └── SeamLinkTwo.cpp         # 双侧连接
```

## 核心函数说明

### SeamSepAllPlane
使用RANSAC算法从点云中分割所有平面，提取法向量。

### SeamCluAllPlane
根据平面法向量的相似性进行聚类。

### SeamTwoPlane
通过点间距离判断找到焊缝的两个侧面。

### SeamRevolve
将点云旋转到标准坐标系，避免坐标系影响。

### SeamIntersectLine
计算两个平面的交线方程。

### SeamEndShow
生成最终的焊缝轨迹展示点云。

### SeamLinkTwo
连接两个焊缝平面的对应点，生成焊缝中心线。

## 输出文件

算法运行时会在 `/tmp/` 目录生成多个中间结果PCD文件：

- `Plane*.pcd` - 分割出的各个平面
- `Target_Plane_*.pcd` - 识别的焊缝平面
- `Link_*_*.pcd` - 相邻平面
- `IntersectLine_*_*.pcd` - 交线
- `End_Show*.pcd` - 最终轨迹
- `weld_point.pcd` - 焊缝中心点云
- `transformed_weld_point_cloud.pcd` - 变换后的焊缝点云

## 示例

```bash
# 使用示例PCD文件运行
ros2 run weld_butt_seam_extracting weld_butt_seam_extracting_node \
  --ros-args -p pcd_file_path:="/path/to/weldcomponent.pcd"
```

## 调试

```bash
# 查看日志
ros2 run weld_butt_seam_extracting weld_butt_seam_extracting_node \
  --ros-args -p pcd_file_path:="/path/to/file.pcd" --log-level debug

# 可视化中间结果
pcl_viewer /tmp/Target_Plane_1.pcd
pcl_viewer /tmp/weld_point.pcd
```

## 常见问题

### Q: 提示找不到PCD文件
A: 确保使用绝对路径，并检查文件权限。

### Q: 提取的焊缝位置不准确
A: 调整transformPointCloud函数中的坐标变换参数，需要根据实际相机-机器人标定结果修改。

### Q: 编译失败
A: 检查是否安装了所有依赖项，特别是PCL、Eigen3和OpenCV。

### Q: 运行时没有输出
A: 检查输入点云质量，确保包含清晰的平面结构。

## 版本历史

- v1.0.0 (2026-01-05) - ROS2 Jazzy版本，完整迁移自ROS1

## 作者

vegyo (2448128273@qq.com)

## 许可证

Apache-2.0

## 相关链接

- [ROS2迁移详细说明](ROS2_MIGRATION_NOTES.md)
- [PCL官方文档](https://pointclouds.org/)
- [ROS2 Jazzy文档](https://docs.ros.org/en/jazzy/)
