#!/bin/bash
# -*- coding: utf-8 -*-

# 快速测试和启动脚本
# 使用方法: ./quick_test_and_run.sh

echo "=== CAM TRACKER NODE 快速测试和运行脚本 ==="
echo

# 检查工作空间
echo "1. 检查工作空间..."
cd /home/micoair/drone_vision_ws
if [ $? -eq 0 ]; then
    echo "✓ 工作空间路径正确"
else
    echo "✗ 工作空间路径错误"
    exit 1
fi

# 检查模型文件
echo "2. 检查YOLO模型文件..."
if [ -f "/home/micoair/UAVGP25/yolo11n.pt" ]; then
    echo "✓ 找到YOLO模型文件"
else
    echo "✗ 未找到YOLO模型文件: /home/micoair/UAVGP25/yolo11n.pt"
    echo "请确保模型文件存在"
    exit 1
fi

# 编译工作空间
echo "3. 编译工作空间..."
catkin_make
if [ $? -eq 0 ]; then
    echo "✓ 编译成功"
else
    echo "✗ 编译失败"
    exit 1
fi

# 设置环境
echo "4. 设置环境..."
source devel/setup.bash
echo "✓ 环境设置完成"

# 检查摄像头设备
echo "5. 检查摄像头设备..."
if [ -e "/dev/video0" ]; then
    echo "✓ 找到摄像头设备 /dev/video0"
else
    echo "⚠ 警告: 未找到摄像头设备 /dev/video0"
    echo "请确保摄像头已连接"
fi

echo
echo "=== 准备工作完成 ==="
echo
echo "启动选项:"
echo "1. 完整启动 (推荐): ./quick_test_and_run.sh full"
echo "2. 仅测试环境: ./quick_test_and_run.sh test"
echo "3. 手动启动步骤:"
echo "   a) roscore"
echo "   b) roslaunch cam_tracker cam_tracker.launch"
echo "   c) rostopic echo /yolo_identify"
echo

# 检查启动参数
if [ "$1" = "full" ]; then
    echo "启动完整系统..."
    echo "按 Ctrl+C 停止"
    sleep 2
    roslaunch cam_tracker cam_tracker.launch
elif [ "$1" = "test" ]; then
    echo "运行环境测试..."
    python3 test_cam_tracker.py
else
    echo "使用 './quick_test_and_run.sh full' 启动完整系统"
    echo "使用 './quick_test_and_run.sh test' 运行环境测试"
fi