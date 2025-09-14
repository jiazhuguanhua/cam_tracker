#!/bin/bash
# ROS日志级别设置脚本
# 使用方法: source set_log_level.sh [debug|info|warn|error]

LOG_LEVEL=${1:-"info"}

echo "设置ROS日志级别为: $LOG_LEVEL"

case $LOG_LEVEL in
    "debug")
        export ROSCONSOLE_CONFIG_FILE=""
        export ROSCONSOLE_FORMAT='[${severity}] [${time}] [${node}] [${function}:${line}]: ${message}'
        rosparam set /cam_tracker_node/log_level DEBUG
        echo "✅ 已设置为DEBUG级别 - 显示所有详细信息"
        ;;
    "info")
        export ROSCONSOLE_CONFIG_FILE=""
        export ROSCONSOLE_FORMAT='[${severity}] [${time}]: ${message}'
        rosparam set /cam_tracker_node/log_level INFO
        echo "✅ 已设置为INFO级别 - 显示一般信息"
        ;;
    "warn")
        rosparam set /cam_tracker_node/log_level WARN
        echo "✅ 已设置为WARN级别 - 只显示警告和错误"
        ;;
    "error")
        rosparam set /cam_tracker_node/log_level ERROR
        echo "✅ 已设置为ERROR级别 - 只显示错误信息"
        ;;
    *)
        echo "❌ 无效的日志级别: $LOG_LEVEL"
        echo "支持的级别: debug, info, warn, error"
        exit 1
        ;;
esac

echo ""
echo "使用方法示例:"
echo "source set_log_level.sh debug   # 最详细的日志"
echo "source set_log_level.sh info    # 标准日志级别"
echo "source set_log_level.sh warn    # 只显示警告"
echo "source set_log_level.sh error   # 只显示错误"