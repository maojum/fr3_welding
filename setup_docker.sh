#!/bin/bash

# 构建Docker镜像（包含完整环境）
echo "构建Docker镜像（包含完整ROS2和项目环境）..."
docker build -t fr3_welding_test .

# 运行容器
echo "运行Docker容器..."
docker run -it --name fr3_welding_container fr3_welding_test
