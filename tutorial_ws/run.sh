#!/bin/bash

# 确保脚本在任何错误发生时立即退出
set -e

# 切换到工作区根目录
cd ~/git/ros2_apps/tutorial_ws

# 加载构建结果
source install/setup.bash

# 运行节点
ros2 run my_cpp_pkg hello_world_publisher