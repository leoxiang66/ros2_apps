#!/bin/bash

# 确保脚本在任何错误发生时立即退出
set -e

# 切换到工作区根目录
cd ~/git/ros2_apps/tutorial_ws

# 移除 build、install 和 log 目录
rm -rf build/ install/ log/

echo "Workspace cleaned successfully."