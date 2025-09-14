#!/bin/bash

echo "=== CAM TRACKER 快速启动测试 ==="
echo

# 设置环境
cd /home/micoair/drone_vision_ws
source devel/setup.bash

echo "1. 测试OpenCV日志设置..."
python3 -c "
import cv2
try:
    cv2.setLogLevel(2)
    print('✅ OpenCV日志设置成功')
except Exception as e:
    print(f'❌ OpenCV日志设置失败: {e}')
    exit(1)
"

echo
echo "2. 测试模型文件..."
if [ -f "/home/micoair/drone_vision_ws/src/cam_tracker/config/yolo11n.pt" ]; then
    echo "✅ 找到模型文件"
elif [ -f "/home/micoair/UAVGP25/yolo11n.pt" ]; then
    echo "✅ 找到模型文件 (UAVGP25目录)"
else
    echo "❌ 未找到模型文件"
    echo "请确保模型文件存在于正确位置"
fi

echo
echo "3. 测试节点导入..."
python3 -c "
import sys
sys.path.append('/home/micoair/drone_vision_ws/src/cam_tracker/scripts')
try:
    from cam_tracker_node import setup_opencv_logging
    print('✅ 节点导入成功')
    setup_opencv_logging()
    print('✅ OpenCV日志函数测试成功')
except Exception as e:
    print(f'❌ 节点导入失败: {e}')
    exit(1)
"

echo
echo "4. 检查ROS环境..."
if [ -n "$ROS_MASTER_URI" ]; then
    echo "✅ ROS环境变量已设置"
    echo "   ROS_MASTER_URI: $ROS_MASTER_URI"
else
    echo "❌ ROS环境变量未设置"
fi

echo
echo "=== 测试完成 ==="
echo "如果所有测试都通过，现在可以安全启动节点："
echo "roslaunch cam_tracker cam_tracker.launch"