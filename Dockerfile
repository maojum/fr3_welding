# 使用Ubuntu 24.04作为基础镜像
FROM ubuntu:24.04

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive

# 更新系统并安装基本工具
RUN apt update && apt upgrade -y && \
    apt install -y \
        curl \
        wget \
        gnupg \
        lsb-release \
        software-properties-common \
        locales \
        git \
        vim \
        nano \
        python3 \
        python3-pip \
        build-essential \
        && rm -rf /var/lib/apt/lists/*

# 设置locale
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

# 添加ROS2 GPG密钥和源
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装ROS2 Jazzy桌面版
RUN apt update && apt install -y ros-jazzy-desktop && \
    rm -rf /var/lib/apt/lists/*

# 设置ROS2环境变量
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# 安装colcon构建工具
RUN apt update && apt install -y python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*

# 安装rosdepc
RUN pip3 install -U rosdepc --break-system-packages

# 安装MoveIt和相关ROS2包
RUN apt update && apt install -y \
        ros-jazzy-moveit \
        ros-jazzy-moveit-visual-tools \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-joint-state-broadcaster \
        ros-jazzy-joint-trajectory-controller \
        && rm -rf /var/lib/apt/lists/*

# 初始化rosdepc
RUN rosdepc init && rosdepc update

# 克隆项目
RUN git clone https://github.com/maojum/fr3_welding.git

# 设置工作目录到项目
WORKDIR /workspace/fr3_welding

# 安装项目依赖
RUN rosdepc install --from-paths . --ignore-src -r -y

# 构建项目
RUN colcon build --symlink-install