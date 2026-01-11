# 工业智能机器人的规划与控制算法课程实验

## 项目概述

本项目是一个基于ROS2和MoveIt2的FR3焊接机器人实验平台，包含机器人描述、焊接执行、MoveIt2配置以及焊接缝提取等模块。用于大学生工业智能机器人的规划与控制算法课程实验。

## 环境配置

### 1. 操作系统安装

推荐使用Ubuntu 24.04 LTS（支持ROS2 Jazzy）。如果您的计算机运行其他操作系统，可以使用虚拟机安装Ubuntu。

#### 虚拟机安装（可选）
- 下载VirtualBox或VMware Workstation。
- 下载Ubuntu 24.04 ISO镜像（https://ubuntu.com/download/desktop）。
- 创建新虚拟机，分配至少4GB RAM、2个CPU核心、20GB硬盘。
- 安装Ubuntu，按照向导完成设置。

### 2. ROS2安装

使用鱼香ROS一键安装脚本安装ROS2 Jazzy（适用于Ubuntu 24.04）：

```bash
# 下载并运行鱼香ROS安装脚本
wget http://fishros.com/install -O fishros && . ./fishros

# 选择安装ROS2 Jazzy桌面版
# 按照脚本提示选择相应选项
```

安装完成后，设置环境变量（添加到~/.bashrc）：

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. 安装构建工具与依赖管理工具

使用 `rosdepc` (rosdep 的国内优化版) 和 `colcon` 构建工具管理项目依赖。

```bash
sudo apt update

# 安装 colcon 构建工具
sudo apt install python3-colcon-common-extensions

# 安装 pip (如果尚未安装).
sudo apt install python3-pip

# 安装 rosdepc
sudo pip3 install -U rosdepc --break-system-packages

# 初始化 rosdepc
sudo rosdepc init
rosdepc update
```

### 4. 项目获取与依赖安装

1. **获取项目代码**

   ```bash
   git clone https://github.com/maojum/fr3_welding.git
   cd fr3_welding
   ```

2. **一键安装依赖**

   利用 `rosdepc` 根据代码中的 `package.xml` 自动安装所需依赖：

   ```bash
   rosdepc install --from-paths . --ignore-src -r -y
   ```

   安装额外包

   ```bash
   sudo apt install \
     ros-jazzy-moveit \
     ros-jazzy-moveit-visual-tools \
     ros-jazzy-ros2-control \
     ros-jazzy-ros2-controllers \
     ros-jazzy-joint-state-broadcaster \
     ros-jazzy-joint-trajectory-controller
   ```

### 5. 构建项目

```bash
colcon build --symlink-install
source install/setup.bash
```

## Docker环境快速搭建
docker快速部署环境

### 使用Docker快速环境

1. **构建完整环境镜像**：
   ```bash
   ./setup_docker.sh
   ```

Docker镜像已预安装：
- Ubuntu 24.04 + ROS2 Jazzy
- MoveIt2 + 相关工具
- 项目代码 + 所有依赖
- 预构建的项目

## 项目结构

- `fr3_bringup/`: 机器人启动配置
- `fr3_description/`: 机器人URDF描述
- `fr3_weld_executing/`: 焊接执行节点
- `fr3_weld_moveit2_config/`: MoveIt2配置
- `weld_butt_seam_extracting/`: 焊接缝提取算法

## 运行实验

### 1. 启动机器人仿真

```bash
ros2 launch fr3_bringup fr3.launch.py
```

### 2. 运行焊接缝提取

```bash
ros2 run weld_butt_seam_extracting weld_butt_seam_extracting_node \
  --ros-args -p pcd_file_path:="./fr3_description/pointcloud/weldcomponent.pcd"
```


## 实验任务

1. **环境搭建**: 完成上述环境配置。
2. **集成测试**: 实现完整的焊接流程，并记录机械臂运行轨迹视频


## 注意事项

- 确保所有依赖正确安装，否则构建会失败。
- 如果遇到权限问题，使用sudo。
- 虚拟机中可能需要调整显卡设置以运行RViz。
- 实验过程中注意保存代码修改记录。

## 参考资料

- ROS2官方文档: https://docs.ros.org/en/jazzy/
- MoveIt2教程: https://moveit.picknik.ai/main/index.html
- PCL文档: https://pointclouds.org/
- 鱼香ROS: https://fishros.com/d2lros2/#/