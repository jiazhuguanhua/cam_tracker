# CAM_TRACKER_NODE 更新说明

## 更新概述
根据 `multi-target-track.py` 示例代码，已全面更新 `cam_tracker_node.py`，实现了更强大的目标追踪功能，支持ROS1 Noetic。

## 主要变更

### 1. 删除模拟数据功能
- 移除了 `generate_mock_detections` 方法
- 删除了所有模拟数据相关的条件判断
- 现在完全依赖真实的YOLO检测结果

### 2. 更新YOLO检测和追踪逻辑
- 使用 `bytetrack.yaml` 追踪器 (更稳定的追踪算法)
- 改用 `model.track()` 方法进行检测和追踪
- 支持持久化追踪 (`persist=True`)

### 3. 实现智能目标选择机制
- **第一帧自动选择**: 在第一帧中自动选择置信度最高的car类型目标
- **目标类型过滤**: 专注于COCO数据集中的car类别 (ID=2)
- **置信度优先**: 选择检测置信度最高的目标作为追踪目标

### 4. 轨迹历史记录功能
- 使用 `defaultdict` 存储每个目标的运动轨迹
- 保留最近30个位置点，形成轨迹线
- 在显示界面中绘制目标运动轨迹

### 5. 增强的日志输出
- **FPS监控**: 实时计算和显示帧率
- **目标选择日志**: 记录自动选择的目标信息
- **追踪状态**: 显示目标是否正常追踪或已丢失
- **详细错误信息**: 包含完整的错误堆栈跟踪
- **统计信息**: 每30帧输出一次详细统计

### 6. 更新显示和发布逻辑
- **轨迹可视化**: 在图像上绘制目标运动轨迹
- **目标丢失处理**: 当目标丢失时显示警告信息
- **多样化显示**: 支持显示所有检测框或仅显示目标
- **状态信息**: 显示追踪器类型、FPS、目标ID等信息

## 新增功能特性

### 目标追踪特性
- 自动目标选择 (第一帧选择最佳car目标)
- 持久化追踪 (目标暂时被遮挡后能重新识别)
- 轨迹历史记录 (显示目标运动路径)
- 目标丢失检测和警告

### 性能监控
- 实时FPS计算和显示
- 处理时间统计
- 检测目标数量统计
- 追踪状态监控

### 用户界面改进
- 更清晰的目标标注
- 轨迹线可视化
- 状态信息显示
- 目标丢失警告

## 配置参数

### 核心参数
- `model_path`: YOLO模型路径 (默认: yolo11n.pt)
- `confidence_threshold`: 检测置信度阈值 (默认: 0.25)
- `device`: 计算设备 (默认: cpu)
- `show_image`: 是否显示图像窗口 (默认: false)

### 追踪参数
- 追踪器: ByteTrack (bytetrack.yaml)
- 轨迹历史长度: 30个点
- 目标选择: 自动选择最高置信度的car

## ROS话题

### 订阅话题
- `/usb_cam/image_raw` (sensor_msgs/Image): 摄像头图像

### 发布话题
- `/yolo_identify` (cam_tracker/DetectionArray): 检测和追踪结果

## 使用方法

### 1. 编译工作空间
```bash
cd /home/micoair/drone_vision_ws
catkin_make
source devel/setup.bash
```

### 2. 启动节点
```bash
# 方法1: 使用launch文件 (推荐)
roslaunch cam_tracker cam_tracker.launch

# 方法2: 分别启动
roscore
rosrun usb_cam usb_cam_node _video_device:=/dev/video0
rosrun cam_tracker cam_tracker_node.py
```

### 3. 监控输出
```bash
# 查看检测结果
rostopic echo /yolo_identify

# 查看节点日志
rosnode info /cam_tracker_node
```

## 日志输出示例

```
[INFO] 正在加载YOLO模型: /home/micoair/UAVGP25/yolo11n.pt
[INFO] YOLO模型加载成功
[INFO] 摄像头追踪节点已启动
[INFO] 第一帧检测到 3 个目标
[INFO] 选择追踪目标ID: 1, 置信度: 0.856
[INFO] 帧数: 30, 平均FPS: 12.3, 检测目标数: 2, 目标追踪状态: 正常
[INFO] 目标ID 1 追踪正常
[WARN] 目标ID 1 已丢失
```

## 故障排除

### 1. 模型加载失败
- 确保YOLO模型文件存在: `/home/micoair/UAVGP25/yolo11n.pt`
- 检查ultralytics是否正确安装
- 检查文件权限

### 2. 摄像头连接问题
- 确认摄像头设备路径: `/dev/video0`
- 检查usb_cam节点是否正常运行
- 验证摄像头硬件连接

### 3. 追踪性能问题
- 降低图像分辨率
- 调整confidence_threshold
- 考虑使用GPU加速 (设置device="cuda")

## 依赖要求

- ROS1 Noetic
- Python 3.8+
- OpenCV
- NumPy
- ultralytics (YOLO)
- usb_cam package
- cv_bridge