#!/bin/bash
# YOLO追踪系统快速启动脚本

echo "=== YOLO11 目标追踪系统 ==="
echo "正在启动系统..."

# 切换到工作空间
cd /home/micoair/drone_vision_ws

# 编译工作空间
echo "编译工作空间..."
catkin_make

# 设置环境
source devel/setup.bash

# 启动系统
echo "启动追踪系统..."
roslaunch cam_tracker cam_tracker.launch

echo "系统已启动！"
echo "使用以下命令控制追踪器："
echo "  启动: rostopic pub /tracker_action std_msgs/Bool \"data: true\""
echo "  停止: rostopic pub /tracker_action std_msgs/Bool \"data: false\""