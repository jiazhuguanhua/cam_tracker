# cam_tracker ROS包

## 🎯 功能概述

**高性能实时多目标追踪系统**，基于YOLO11+ByteTrack算法，专为ROS机器人系统设计。

### 核心特性
- 🔥 **YOLO11检测** + **ByteTrack追踪**：业界领先的检测追踪算法
- 🎮 **动态控制**：通过ROS话题实时启动/停止追踪器
- 📊 **实时可视化**：边界框、追踪ID、运动轨迹显示
- ⚡ **高性能**：支持CPU/GPU加速，优化的处理流程
- � **易集成**：标准ROS接口，支持多种硬件平台

## 🖥️ 运行环境

- **操作系统**: Ubuntu 20.04 LTS + ROS Noetic
- **Python**: 3.8+
- **硬件**: USB摄像头
- **推荐平台**: NVIDIA Jetson系列 (Orin Nano/NX/AGX)

## 🚀 快速开始

### 1. 环境配置
**NVIDIA Jetson用户** (推荐)：
```bash
# 参考官方完整配置教程
# https://docs.ultralytics.com/zh/guides/nvidia-jetson/
```

**通用平台**：
```bash
pip install ultralytics opencv-python
sudo apt install ros-noetic-usb-cam ros-noetic-cv-bridge
```

### 2. 启动系统
```bash
# 编译和启动
cd ~/drone_vision_ws
catkin_make && source devel/setup.bash
roslaunch cam_tracker cam_tracker.launch
```

### 3. 控制追踪器 🎮
```bash
# 启动追踪
rostopic pub /tracker_action std_msgs/Bool "data: true"

# 停止追踪  
rostopic pub /tracker_action std_msgs/Bool "data: false"

# 使用交互式控制脚本
~/drone_vision_ws/start_tracker_demo.sh
```

### 4. 监控结果
```bash
# 查看检测结果
rostopic echo /yolo_identify

# 查看系统状态
rostopic list | grep tracker
```

## ⚙️ 配置参数

### Launch文件参数
```xml
<!-- 模型配置 -->
<param name="model_path" value="path/to/yolo11n.pt" />
<param name="confidence_threshold" value="0.25" />

<!-- 显示配置 -->
<param name="show_image" value="false" />  <!-- 可视化开关 -->

<!-- 摄像头配置 -->
<param name="video_device" value="/dev/video0" />
<param name="image_width" value="640" />
<param name="image_height" value="480" />
```
默认使用CUDA，ultralytics系统会自动选择可用的设备。（CPU/GPU etc.）

### ROS参数动态配置
```bash
# 运行时修改参数
rosparam set /cam_tracker_node/confidence_threshold 0.3
rosparam set /cam_tracker_node/show_image true

# 查看当前参数
rosparam get /cam_tracker_node/confidence_threshold
rosparam list | grep cam_tracker
```

## 📡 ROS接口

### 话题接口
```bash
# 订阅话题
/usb_cam/image_raw          # 摄像头图像数据
/tracker_action             # 追踪器控制 (std_msgs/Bool)

# 发布话题  
/yolo_identify              # 检测结果 (cam_tracker/DetectionArray)
```

### 控制命令
```bash
# 启动/停止追踪器
rostopic pub /tracker_action std_msgs/Bool "data: true"   # 启动
rostopic pub /tracker_action std_msgs/Bool "data: false"  # 停止

# 参数动态配置
rosparam set /cam_tracker_node/confidence_threshold 0.3
rosparam set /cam_tracker_node/show_image true
rosparam list | grep cam_tracker  # 查看所有参数
```

## 🔧 日志系统

### 日志级别控制
```bash
# 正常运行 - 简洁日志
roslaunch cam_tracker cam_tracker.launch

# 调试模式 - 详细日志  
export ROSCONSOLE_CONFIG_FILE=$(rospack find cam_tracker)/config/debug_console.config
roslaunch cam_tracker cam_tracker.launch

# 实时调整日志级别
rosparam set /cam_tracker_node/log_level DEBUG
```

## 🎨 可视化界面

启用可视化显示：
```bash
# 方法1: Launch参数
<param name="show_image" value="true" />

# 方法2: 运行时设置
rosparam set /cam_tracker_node/show_image true
```

### 显示内容
- 🟩 **彩色边界框**: 不同类别使用不同颜色 (person: 绿色, car: 蓝色等)
- 🎯 **追踪ID**: 显示唯一的追踪标识符  
- 📍 **中心点**: 目标中心位置标记
- 📈 **运动轨迹**: 目标移动路径 (最近30个位置点)
- 📊 **实时信息**: FPS显示、目标总数、颜色图例

## ⚡ 性能优化

### 硬件加速
```bash
# 降低分辨率提升帧率
rosparam set /usb_cam/image_width 320
rosparam set /usb_cam/image_height 240
```

### 参数调优
```bash
# 置信度阈值 (降低检测更多目标)
rosparam set /cam_tracker_node/confidence_threshold 0.2

# 关闭可视化 (提升性能)
rosparam set /cam_tracker_node/show_image false
```

## 🛠️ 故障排除

### 常见问题
```bash
# 1. 摄像头无法打开
ls /dev/video*  # 检查设备
sudo usermod -a -G video $USER  # 添加权限

# 2. 模型加载失败  
find ~/drone_vision_ws -name "*.pt"  # 检查模型文件

# 3. 追踪器无响应
rostopic list | grep tracker  # 检查话题
rostopic pub /tracker_action std_msgs/Bool "data: true"  # 重新启动

# 4. 性能监控
rostopic hz /yolo_identify  # 检查发布频率
htop -p $(pgrep -f cam_tracker)  # 监控CPU使用
```

## 📂 项目结构

```
cam_tracker/
├── launch/cam_tracker.launch           # 主启动文件
├── scripts/cam_tracker_node.py         # 核心追踪节点
├── msg/                                # 消息定义
│   ├── Detection.msg                   # 单个检测结果
│   └── DetectionArray.msg              # 检测结果数组
├── models/yolo11n.pt                   # YOLO11模型文件
├── start_tracker_demo.sh               # 控制演示脚本 ⭐
└── README.md                           # 本文档
```

## 🤝 应用场景

- 🚁 **无人机视觉追踪**: 自动识别和跟踪地面目标
- 🤖 **移动机器人导航**: 实时检测行人、车辆、障碍物  
- 📹 **智能监控系统**: 多目标同时追踪
- 🚗 **自动驾驶辅助**: 检测道路目标
- 🏭 **工业自动化**: 物体识别、分拣、质量检测

## 📈 性能数据

**测试环境**: NVIDIA Jetson Orin + USB 3.0相机
- **检测延迟**: 10ms (GPU) yolov11
- **总体FPS**: 18-25 (640x480分辨率)
- **内存占用**: ~500MB

---
**如果这个项目对您有帮助，请给我一个Star！**