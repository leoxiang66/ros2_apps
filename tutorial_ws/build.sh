#!/bin/bash

# 确保脚本在任何错误发生时立即退出
set -e

# 切换到工作区根目录
cd ~/git/ros2_apps/tutorial_ws

# 加载 ROS 2 Jazzy 环境
source /opt/ros/jazzy/setup.bash

# 构建工作区
colcon build

# 加载构建结果
source install/setup.bash

echo "Build completed successfully."