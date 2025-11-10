#!/bin/bash

# Cam Tracker 快速启动脚本

echo "=================================="
echo "Cam Tracker 快速启动"
echo "=================================="

# 设置工作空间路径
WORKSPACE_PATH=~/drone_ws

echo "✅ 切换到工作空间: $WORKSPACE_PATH"
cd $WORKSPACE_PATH

# 刷新环境
echo "✅ 刷新ROS环境..."
source devel/setup.bash

# 添加可执行权限
echo "✅ 设置脚本可执行权限..."
chmod +x src/cam_tracker/scripts/cam_tracker_node.py
chmod +x src/cam_tracker/scripts/visualizer_node_new.py
chmod +x src/cam_tracker/scripts/test_controller.py

# 检查摄像头节点是否运行
echo "✅ 检查摄像头节点..."
if rostopic list | grep -q "/usb_cam/image_raw"; then
    echo "   ✓ 摄像头节点正在运行"
else
    echo "   ⚠️  摄像头节点未运行，请先启动摄像头！"
    echo "   提示: roslaunch usb_cam usb_cam-test.launch"
fi

echo ""
echo "=================================="
echo "启动选项："
echo "=================================="
echo "1. 启动检测和可视化节点"
echo "2. 仅启动检测节点"
echo "3. 仅启动可视化节点"
echo "4. 启动控制器测试"
echo "5. 编译并启动"
echo "6. 查看帮助"
echo "q. 退出"
echo "=================================="

read -p "请选择 (1-6/q): " choice

case $choice in
    1)
        echo "🚀 启动检测和可视化节点..."
        roslaunch cam_tracker cam_tracker_new.launch
        ;;
    2)
        echo "🚀 启动检测节点..."
        rosrun cam_tracker cam_tracker_node.py
        ;;
    3)
        echo "🚀 启动可视化节点..."
        rosrun cam_tracker visualizer_node_new.py
        ;;
    4)
        echo "🎮 启动控制器测试..."
        rosrun cam_tracker test_controller.py
        ;;
    5)
        echo "🔨 编译工作空间..."
        catkin_make
        source devel/setup.bash
        echo "✅ 编译完成！"
        echo "🚀 启动节点..."
        roslaunch cam_tracker cam_tracker_new.launch
        ;;
    6)
        echo ""
        echo "=================================="
        echo "Cam Tracker 使用说明"
        echo "=================================="
        echo ""
        echo "📋 控制命令:"
        echo "   0 - 停止检测"
        echo "   1 - 抓取区模式（只检测ball）"
        echo "   2 - 释放区模式（只检测car）"
        echo ""
        echo "📡 话题接口:"
        echo "   订阅: /cam_tracker/tracker_control (UInt8)"
        echo "   发布: /cam_tracker/info (CamTrack)"
        echo ""
        echo "🎮 可视化快捷键:"
        echo "   q - 退出"
        echo "   p - 暂停/继续"
        echo "   s - 保存截图"
        echo "   h - 显示/隐藏帮助"
        echo "   d - 显示/隐藏详细信息"
        echo ""
        echo "📚 详细文档: src/cam_tracker/NEW_VERSION_README.md"
        echo "=================================="
        ;;
    q|Q)
        echo "👋 退出"
        exit 0
        ;;
    *)
        echo "❌ 无效选择"
        exit 1
        ;;
esac
