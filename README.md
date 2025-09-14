# cam_tracker ROS包

## 简介

`cam_tracker` 是一个简化的ROS包，用于USB摄像头的实时目标检测和追踪。使用YOLO11进行目标检测，支持ByteTracker或简化追踪器进行多目标追踪。

## 功能特性

- 🎯 **YOLO11目标检测**: 使用最新的Ultralytics YOLO11模型
- 📹 **USB摄像头支持**: 直接读取USB摄像头数据
- 🔄 **多目标追踪**: 支持ByteTracker或内置简化追踪器
- 📡 **ROS集成**: 完整的ROS话题发布订阅机制
- 🔧 **灵活配置**: 支持参数化配置模型和追踪器

## 系统要求

- ROS Noetic
- Python 3.8+
- OpenCV
- USB摄像头 (通常在 `/dev/video0`)

## 安装依赖

### Python依赖
```bash
pip install ultralytics opencv-python numpy
```

### 可选的ByteTracker (推荐)
```bash
pip install yolox
```

### ROS依赖
```bash
sudo apt install ros-noetic-usb-cam ros-noetic-image-view
```

## 消息类型

### Detection.msg
单个检测目标的信息：
```
int32 id                    # 追踪ID
string class_name           # 类别名称  
float32 confidence          # 置信度
float32[] xyxy              # 边界框 [x1, y1, x2, y2]
float32 center_x            # 中心点x坐标
float32 center_y            # 中心点y坐标
float32 width               # 宽度
float32 height              # 高度
```

### DetectionArray.msg
多个检测目标的集合：
```
std_msgs/Header header      # 消息头
Detection[] detections      # 检测目标数组
int32 image_width           # 图像宽度
int32 image_height          # 图像高度
int32 total_objects         # 总检测目标数量
float32 processing_time     # 处理时间(秒)
```

## 话题

- **订阅**: `/usb_cam/image_raw` (sensor_msgs/Image) - USB摄像头图像
- **发布**: `/yolo_identify` (cam_tracker/DetectionArray) - 检测和追踪结果

## 使用方法

### 1. 编译包
```bash
cd ~/drone_vision_ws
catkin_make
source devel/setup.bash
```

### 2. 启动追踪系统
```bash
roslaunch cam_tracker cam_tracker.launch
```

### 3. 测试订阅结果
在另一个终端中：
```bash
rosrun cam_tracker test_subscriber.py
```

### 4. 查看图像 (可选)
```bash
rosrun image_view image_view image:=/usb_cam/image_raw
```

## 配置参数

### launch文件参数
- `model_path`: YOLO模型路径 (默认: yolo11n.pt)
- `confidence_threshold`: 置信度阈值 (默认: 0.5)
- `device`: 运行设备 (默认: cpu)
- `max_age`: 目标最大消失帧数 (默认: 30)
- `min_hits`: 最小命中次数 (默认: 3)
- `iou_threshold`: IoU阈值 (默认: 0.3)

### 摄像头参数
- `video_device`: 摄像头设备路径 (默认: /dev/video0)
- `image_width`: 图像宽度 (默认: 640)
- `image_height`: 图像高度 (默认: 480)
- `framerate`: 帧率 (默认: 30)

## 故障排除

### 1. 摄像头无法访问
```bash
# 检查摄像头设备
ls /dev/video*

# 测试摄像头
cheese  # 或者使用其他摄像头测试工具
```

### 2. YOLO模型下载
如果模型自动下载失败：
```bash
# 手动下载YOLO11模型
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolo11n.pt
mv yolo11n.pt ~/drone_vision_ws/src/cam_tracker/config/
```

### 3. 依赖问题
```bash
# 检查Python包
python3 -c "import ultralytics; print('Ultralytics OK')"
python3 -c "import cv2; print('OpenCV OK')"
```

### 4. 常见错误处理
- **权限错误**: 确保脚本有执行权限 `chmod +x scripts/*.py`
- **摄像头占用**: 关闭其他使用摄像头的程序
- **内存不足**: 降低图像分辨率或使用较小的YOLO模型

## 开发与调试

### 查看话题
```bash
# 列出所有话题
rostopic list

# 查看检测结果
rostopic echo /yolo_identify

# 查看消息频率
rostopic hz /yolo_identify
```

### 性能监控
```bash
# 查看节点状态
rosnode info /cam_tracker_node

# 监控CPU和内存使用
htop
```

## 文件结构

```
cam_tracker/
├── CMakeLists.txt          # CMake构建文件
├── package.xml             # ROS包配置
├── README.md               # 说明文档
├── config/                 # 配置文件
│   └── tracker_config.yaml
├── launch/                 # 启动文件
│   └── cam_tracker.launch
├── msg/                    # 消息定义
│   ├── Detection.msg
│   └── DetectionArray.msg
└── scripts/                # Python脚本
    ├── cam_tracker_node.py # 主追踪节点
    └── test_subscriber.py  # 测试订阅者
```

## 许可证

本项目采用 TODO 许可证。

## 贡献

欢迎提交Issue和Pull Request！

## 更新日志

- v0.1.0: 初始版本，支持基本的目标检测和追踪功能