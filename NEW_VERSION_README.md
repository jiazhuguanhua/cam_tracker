# Cam Tracker - 新版使用指南

## 📋 概述

新版cam_tracker支持ball和car两种目标的检测与追踪，使用ByteTracker追踪器，支持通过控制信号切换检测模式。

## 🎯 功能特性

- ✅ 支持ball和car两种目标检测
- ✅ 模式切换：抓取区（只检测ball）/ 释放区（只检测car）
- ✅ 使用ByteTracker进行目标追踪
- ✅ 实时可视化显示
- ✅ 距离信息接口（预留，当前输出0）
- ✅ 夹爪状态接口（预留，当前输出False）

## 📦 消息定义

### CamTrack.msg

```msg
std_msgs/Header header      # 时间戳、坐标系
bool system_ok              # 系统状态

# Ball信息（抓取区）
int32 ball_num              # ball数量
float32 ball_x              # 目标ball的x坐标
float32 ball_y              # 目标ball的y坐标
float32 ball_dis            # 距离（预留，输出0）

# Car信息（释放区）
int32 car_num               # car数量
float32[] car_x             # car的x坐标数组
float32[] car_y             # car的y坐标数组
float32[] car_dis           # 距离数组（预留，输出0）

# 夹爪状态（预留）
bool in_gripper             # 是否抓取到球（输出False）
```

## 🚀 使用方法

### 1. 编译工作空间

```bash
cd ~/drone_uavgp_ws
catkin_make

# 或使用catkin build
catkin build cam_tracker

# 刷新环境
source devel/setup.bash
```

### 2. 启动节点

```bash
# 启动检测和可视化节点
roslaunch cam_tracker cam_tracker_new.launch
```

### 3. 发送控制信号

#### 方式1: 使用测试脚本（推荐）

```bash
# 新终端
cd ~/drone_uavgp_ws
source devel/setup.bash
rosrun cam_tracker test_controller.py

# 按提示输入:
# 0 - 停止检测
# 1 - 抓取区模式（只检测ball）
# 2 - 释放区模式（只检测car）
```

#### 方式2: 使用rostopic命令

```bash
# 停止检测
rostopic pub /cam_tracker/tracker_control std_msgs/UInt8 "data: 0" -1

# 抓取区模式（检测ball）
rostopic pub /cam_tracker/tracker_control std_msgs/UInt8 "data: 1" -1

# 释放区模式（检测car）
rostopic pub /cam_tracker/tracker_control std_msgs/UInt8 "data: 2" -1
```

### 4. 查看检测结果

```bash
# 查看检测信息
rostopic echo /cam_tracker/info

# 查看特定字段
rostopic echo /cam_tracker/info/ball_num
rostopic echo /cam_tracker/info/car_num
```

## 📊 话题接口

### 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/usb_cam/image_raw` | sensor_msgs/Image | 摄像头图像输入 |
| `/cam_tracker/tracker_control` | std_msgs/UInt8 | 控制信号 (0/1/2) |

### 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cam_tracker/info` | cam_tracker/CamTrack | 检测结果信息 |

## 🎮 可视化控制

运行可视化节点后，可使用以下键盘快捷键：

- `q`: 退出程序
- `p`: 暂停/继续显示
- `s`: 保存当前帧截图
- `h`: 显示/隐藏帮助信息
- `d`: 显示/隐藏详细信息

## 📝 示例输出

### 抓取区模式（control=1）

```yaml
header:
  seq: 105
  stamp:
    secs: 1731162345
    nsecs: 500123000
  frame_id: "camera_link"
system_ok: True

ball_num: 1
ball_x: 157.3
ball_y: 120.8
ball_dis: 0.0

car_num: 0
car_x: []
car_y: []
car_dis: []

in_gripper: False
```

### 释放区模式（control=2）

```yaml
header:
  seq: 112
  stamp:
    secs: 1731162350
    nsecs: 102456000
  frame_id: "camera_link"
system_ok: True

ball_num: 0
ball_x: 0.0
ball_y: 0.0
ball_dis: 0.0

car_num: 2
car_x: [85.3, 192.7]
car_y: [65.1, 142.0]
car_dis: [0.0, 0.0]

in_gripper: False
```

## ⚙️ 配置参数

在launch文件中可调整以下参数：

```xml
<arg name="camera_topic" default="/usb_cam/image_raw" />
<arg name="confidence_threshold" default="0.5" />
<arg name="iou_threshold" default="0.45" />
<arg name="max_det" default="100" />
<arg name="tracker_type" default="bytetrack.yaml" />
<arg name="publish_rate" default="20.0" />
```

## 🔧 模型配置

1. 确保您的YOLO模型包含ball和car两个类别
2. 将模型文件放在 `cam_tracker/models/` 目录下
3. 在 `cam_tracker_node.py` 中修改 `MODEL_NAME` 变量

```python
MODEL_NAME = "your_model.pt"  # 修改为您的模型名称
```

## 🐛 故障排查

### 问题1: 找不到消息类型

```bash
# 重新编译并刷新环境
cd ~/drone_uavgp_ws
catkin_make
source devel/setup.bash
```

### 问题2: 模型加载失败

- 检查模型文件路径是否正确
- 确保模型包含ball和car两个类别
- 查看日志输出的错误信息

### 问题3: 没有检测结果

- 检查是否发送了正确的控制信号（1或2）
- 确认摄像头图像正常
- 调整置信度阈值参数

## 📚 文件结构

```
cam_tracker/
├── scripts/
│   ├── cam_tracker_node.py          # 主检测节点（新）
│   ├── visualizer_node_new.py       # 可视化节点（新）
│   ├── test_controller.py           # 测试控制脚本（新）
│   ├── ball_tracker_node.py         # 旧版ball追踪节点
│   └── cam_tracker_node_old.py      # 旧版检测节点
├── launch/
│   └── cam_tracker_new.launch       # 新版launch文件
├── msg/
│   ├── CamTrack.msg                 # 新消息定义
│   ├── Detection.msg                # 旧消息
│   └── ...
└── models/
    └── your_model.pt                # YOLO模型文件
```

## 🔄 从旧版本迁移

旧版本使用的话题和消息已经改变：

| 旧版本 | 新版本 |
|--------|--------|
| `/tracker_control` (Bool) | `/cam_tracker/tracker_control` (UInt8) |
| `/detection/target` (Detection) | `/cam_tracker/info` (CamTrack) |
| `/detection/all_targets` | `/cam_tracker/info` |

## 📞 支持

如有问题，请查看日志输出或联系开发团队。

---

**版本**: 3.0.0  
**日期**: 2025-11-09  
**作者**: ROS Developer
