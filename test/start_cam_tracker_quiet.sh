#!/bin/bash
# 启动cam_tracker时抑制FFmpeg警告的脚本

# 设置环境变量来抑制FFmpeg日志
export FFMPEG_HIDE_BANNER=1
export AV_LOG_FORCE_NOCOLOR=1
export OPENCV_LOG_LEVEL=ERROR
export OPENCV_FFMPEG_LOGLEVEL=-8

# 设置ROS环境
cd /home/micoair/drone_vision_ws
source devel/setup.bash

echo "=== 启动CAM TRACKER (无FFmpeg警告版本) ==="
echo "已设置环境变量抑制FFmpeg警告输出"
echo "按Ctrl+C停止"
echo

# 启动launch文件
roslaunch cam_tracker cam_tracker.launch