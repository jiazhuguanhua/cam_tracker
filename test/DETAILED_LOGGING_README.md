# CAM_TRACKER_NODE 详细日志系统说明

## 新增的详细日志功能

### 🎯 日志级别支持
现在支持ROS标准的4个日志级别：

- **DEBUG** 🔍: 最详细的调试信息
- **INFO** 📝: 一般状态信息  
- **WARN** ⚠️: 警告信息
- **ERROR** ❌: 错误信息

### 📊 详细日志内容

#### 启动阶段日志
```
[INFO] ============================================================
[INFO] 初始化YOLO11多目标追踪节点
[INFO] ============================================================
[INFO] 节点参数配置:
[INFO]   模型路径: /home/micoair/UAVGP25/yolo11n.pt
[INFO]   置信度阈值: 0.25
[INFO]   计算设备: cpu
[INFO]   显示图像: true
[DEBUG] CvBridge初始化完成
[INFO] ----------------------------------------
[INFO] 开始加载YOLO模型...
[INFO] 模型文件路径: /home/micoair/UAVGP25/yolo11n.pt
[INFO] 模型文件大小: 6.2 MB
[INFO] YOLO模型加载成功! 耗时: 2.34秒
[INFO] 模型类别数量: 80
[DEBUG] 支持的类别: ['person', 'bicycle', 'car', ...]
```

#### 运行时日志
```
[INFO] 收到第一帧图像数据
[DEBUG] 图像尺寸: 640x480
[DEBUG] 图像编码: bgr8
[DEBUG] 图像转换成功，形状: (480, 640, 3)
[DEBUG] YOLO推理耗时: 24.2ms
[INFO] ****************************************
[INFO] 第一帧检测结果:
[INFO]   检测到 1 个目标
[INFO]   推理耗时: 24.2ms
[INFO] 开始追踪所有目标...
[INFO] ****************************************
```

#### 统计信息日志
```
[INFO] ==================================================
[INFO] 帧数统计 - 第30帧
[INFO]   平均FPS: 23.3
[INFO]   最高FPS: 28.5
[INFO]   最低FPS: 18.2
[INFO]   平均处理时间: 35.2ms
[INFO]   当前检测目标数: 1
[INFO]   目标类别统计: person: 1
[INFO] ==================================================
```

#### 调试级别详细信息
```
[DEBUG] 处理第10帧, 当前FPS: 23, 检测目标: 1, 处理时间: 24.2ms
[DEBUG] 检测结果处理: 1个边界框
[DEBUG] 追踪ID列表: [1]
[DEBUG] 目标ID 1: person 置信度:0.856 位置:(320,240) 轨迹点数:10
[DEBUG] 本帧处理完成: 1个追踪目标
[DEBUG] 准备发布检测数组: 图像尺寸(640x480), 目标数量: 1, 处理时间: 24.2ms
[DEBUG] 检测对象1: ID=1, 类别=person, 置信度=0.856, 中心=(320,240)
[DEBUG] 消息发布完成，耗时: 0.15ms
[DEBUG] 成功发布 1 个追踪目标到话题 /yolo_identify
```

## 🛠️ 使用方法

### 1. 标准启动（INFO级别）
```bash
cd /home/micoair/drone_vision_ws
source devel/setup.bash
roslaunch cam_tracker cam_tracker.launch
```

### 2. 调试模式启动（DEBUG级别）
```bash
cd /home/micoair/drone_vision_ws
source set_log_level.sh debug
source devel/setup.bash
roslaunch cam_tracker cam_tracker.launch
```

### 3. 静默模式启动（WARN级别）
```bash
cd /home/micoair/drone_vision_ws
source set_log_level.sh warn
source devel/setup.bash
roslaunch cam_tracker cam_tracker.launch
```

### 4. 使用静默启动脚本（抑制FFmpeg警告）
```bash
cd /home/micoair/drone_vision_ws
./start_cam_tracker_quiet.sh
```

## 📈 日志输出频率

### 实时输出
- 错误信息：立即输出
- 警告信息：立即输出
- 致命错误：立即输出

### 定期输出
- **每帧**: DEBUG级别的处理详情
- **每10帧**: 简要统计信息（DEBUG级别）
- **每30帧**: 详细统计报告（INFO级别）

### 特殊时机
- **节点启动**: 完整的初始化信息
- **第一帧**: 特别详细的检测信息
- **异常情况**: 完整的错误堆栈

## 🎛️ 动态调整日志级别

### 运行时调整
```bash
# 设置为DEBUG级别（最详细）
rosparam set /cam_tracker_node/rosconsole/level DEBUG

# 设置为INFO级别（标准）
rosparam set /cam_tracker_node/rosconsole/level INFO

# 设置为WARN级别（只显示警告）
rosparam set /cam_tracker_node/rosconsole/level WARN

# 设置为ERROR级别（只显示错误）
rosparam set /cam_tracker_node/rosconsole/level ERROR
```

### 查看当前日志级别
```bash
rosparam get /cam_tracker_node/rosconsole/level
```

## 📋 日志信息分类

### 🔧 系统信息 (INFO)
- 节点启动/停止
- 模型加载状态
- 参数配置信息
- 话题连接状态
- 定期统计报告

### 🔍 调试信息 (DEBUG)
- 每帧处理详情
- 推理时间统计
- 目标检测详情
- 追踪ID变化
- 消息发布详情
- 图像转换信息

### ⚠️ 警告信息 (WARN)
- 追踪ID丢失
- 检测结果异常
- 性能下降提醒
- 资源使用警告

### ❌ 错误信息 (ERROR)
- 图像转换失败
- 模型推理错误
- 消息发布失败
- ROS通信异常

### 💀 致命错误 (FATAL)
- 模型文件缺失
- 节点初始化失败
- 系统资源不足
- 关键组件失效

## 🎨 日志格式说明

### 标准格式
```
[级别] [时间戳]: 消息内容
```

### 调试格式（包含函数和行号）
```
[级别] [时间戳] [节点名] [函数名:行号]: 消息内容
```

### 特殊标记
- 🚀 启动相关
- ✅ 成功完成
- ❌ 错误/失败
- ⚠️ 警告
- 🔄 处理中
- 📊 统计信息
- 🛑 停止/中断
- 🧹 清理资源
- 👋 正常退出

## 🔧 故障排除

### 如果日志太多
```bash
# 设置为较高级别
source set_log_level.sh warn
```

### 如果需要调试
```bash
# 设置为最详细级别
source set_log_level.sh debug
```

### 保存日志到文件
```bash
roslaunch cam_tracker cam_tracker.launch 2>&1 | tee cam_tracker.log
```

### 实时监控特定话题
```bash
# 监控检测结果
rostopic echo /yolo_identify

# 监控节点状态
rosnode info /cam_tracker_node
```

## 📊 性能监控

通过日志可以监控的性能指标：

1. **FPS**: 实时帧率和平均帧率
2. **处理时间**: 每帧的处理耗时
3. **推理时间**: YOLO模型推理耗时
4. **发布时间**: ROS消息发布耗时
5. **内存使用**: 轨迹历史占用
6. **检测质量**: 置信度分布和目标数量

现在您的cam_tracker_node将提供非常详细的日志信息，帮助您更好地了解系统运行状况！