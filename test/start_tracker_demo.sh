#!/bin/bash

# YOLO11 追踪器控制演示脚本
# 使用 /tracker_action 话题来控制追踪器的启动和停止

echo "=========================================="
echo "YOLO11 追踪器控制演示"
echo "=========================================="

# 检查ROS是否运行
if ! pgrep -x "roscore" > /dev/null; then
    echo "错误: 请先启动 roscore"
    exit 1
fi

echo "可用的控制命令:"
echo ""
echo "1. 启动追踪器:"
echo "   rostopic pub /tracker_action std_msgs/Bool \"data: true\""
echo ""
echo "2. 停止追踪器:"
echo "   rostopic pub /tracker_action std_msgs/Bool \"data: false\""
echo ""
echo "3. 查看检测结果:"
echo "   rostopic echo /yolo_identify"
echo ""
echo "4. 查看追踪器状态:"
echo "   rostopic list | grep tracker"
echo ""

# 函数: 启动追踪器
start_tracker() {
    echo "🟢 启动追踪器..."
    rostopic pub -1 /tracker_action std_msgs/Bool "data: true"
    echo "✅ 启动命令已发送"
}

# 函数: 停止追踪器
stop_tracker() {
    echo "🔴 停止追踪器..."
    rostopic pub -1 /tracker_action std_msgs/Bool "data: false"
    echo "✅ 停止命令已发送"
}

# 函数: 查看状态
show_status() {
    echo "📊 当前话题状态:"
    rostopic list | grep -E "(tracker|yolo|usb_cam)" | sort
    echo ""
    echo "📈 检测结果频率:"
    timeout 3 rostopic hz /yolo_identify 2>/dev/null || echo "暂无检测结果发布"
}

# 交互式菜单
while true; do
    echo ""
    echo "请选择操作:"
    echo "  1) 启动追踪器"
    echo "  2) 停止追踪器"
    echo "  3) 查看状态"
    echo "  4) 监控检测结果"
    echo "  5) 退出"
    echo ""
    read -p "输入选择 (1-5): " choice
    
    case $choice in
        1)
            start_tracker
            ;;
        2)
            stop_tracker
            ;;
        3)
            show_status
            ;;
        4)
            echo "📡 监控检测结果 (按 Ctrl+C 停止):"
            rostopic echo /yolo_identify
            ;;
        5)
            echo "👋 再见!"
            break
            ;;
        *)
            echo "❌ 无效选择，请输入 1-5"
            ;;
    esac
done

echo "脚本结束"