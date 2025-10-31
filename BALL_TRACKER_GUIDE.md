# Ball Tracker Node - 小球追踪节点使用指南

## 📋 概述

这是专为无人机抓取小球任务设计的视觉追踪节点，相比旧版本进行了全面优化。

## ✨ 主要改进

### 1. 功能优化 🎯
- **智能目标选择**: 自动选择图像边缘位置的目标（便于抓取）
- **分阶段执行**: 
  - 阶段1: 拍照识别，选择边缘目标
  - 阶段2: 持续追踪选定目标
- **优雅的启停控制**: 
  - 接收True启动追踪
  - 接收False停止追踪（保持模型在GPU中）
- **目标丢失处理**: 超过阈值后自动重新选择目标

### 2. 从Person追踪到Ball追踪 🎾
- ✅ 移除Person追踪相关代码
- ✅ 专注于小球/物体检测
- ✅ 优化目标选择逻辑（边缘优先）
- ✅ 更贴切的命名和注释

### 3. 控制方式改进 🎮
- **简化接口**: 使用Bool消息控制（True=启动, False=停止）
- **状态反馈**: 实时发布追踪器状态
- **分阶段信息**: 通过状态话题获知当前阶段
- **可选自动启动**: 支持节点启动后自动开始追踪

### 4. 代码质量提升 📦
- ✅ 清晰的状态机设计
- ✅ 完善的类型提示
- ✅ 详细的文档注释
- ✅ 模块化的函数设计
- ✅ 统一的日志格式

## 🔧 配置参数

### launch文件示例

```xml
<launch>
    <node name="ball_tracker" pkg="cam_tracker" type="ball_tracker_node.py" output="screen">
        <!-- 相机话题 -->
        <param name="camera_topic" value="/usb_cam/image_raw"/>
        
        <!-- 检测参数 -->
        <param name="confidence_threshold" value="0.5"/>
        <param name="iou_threshold" value="0.45"/>
        <param name="max_det" value="100"/>
        
        <!-- 追踪参数 -->
        <param name="tracker_type" value="bytetrack.yaml"/>
        
        <!-- 边缘检测参数 (0.25 = 图像四周25%区域) -->
        <param name="edge_margin_ratio" value="0.25"/>
        
        <!-- 发布频率 -->
        <param name="min_publish_rate" value="20.0"/>
        
        <!-- 是否自动启动 (默认false，等待控制信号) -->
        <param name="auto_start" value="false"/>
    </node>
</launch>
```

## 📡 话题接口

### 订阅话题

| 话题名 | 消息类型 | 说明 |
|--------|---------|------|
| `/usb_cam/image_raw` | `sensor_msgs/Image` | 相机图像输入 |
| `/tracker_control` | `std_msgs/Bool` | 追踪控制 (True=启动, False=停止) |

### 发布话题

| 话题名 | 消息类型 | 频率 | 说明 |
|--------|---------|------|------|
| `/detection/target` | `cam_tracker/Detection` | ≥20Hz | 当前追踪目标位置 |
| `/detection/all_targets` | `cam_tracker/DetectionArray` | 实时 | 所有检测目标完整信息 |
| `/tracker_status` | `std_msgs/String` | 变化时 | 追踪器状态 |

## 📊 消息格式

### Detection (目标位置)

```python
# detection_id: 
#   1  = 有效目标 (正在追踪)
#   0  = 目标暂时丢失 (未超过30帧，使用最近数据)
#  -1  = 目标已丢失 (超过30帧，积极搜索中，使用最近数据)
int32 detection_id

# 目标中心点坐标 (像素)
float64 detection_x
float64 detection_y
```

**重要说明:**
- `detection_id = 1`: 目标正常追踪中，位置数据实时更新
- `detection_id = 0`: 目标暂时不可见（<30帧），位置为最近已知位置
- `detection_id = -1`: 目标丢失超过30帧，节点进入积极搜索模式
  - 继续输出最近的位置数据
  - 持续检测环境中的所有目标
  - 一旦重新识别到符合条件的目标，自动恢复追踪（ID变回1）

### 状态消息格式

状态消息格式: `{state}:{phase}:{description}`

**State (状态):**
- `idle` - 空闲
- `initializing` - 初始化拍照
- `tracking` - 正在追踪
- `target_lost` - 目标丢失
- `stopped` - 已停止

**Phase (阶段):**
- `waiting` - 等待启动
- `snapshot` - 拍照识别
- `target_selected` - 目标已选择
- `tracking_active` - 追踪进行中
- `completed` - 完成

## 🚀 使用流程

### 方式1: 手动控制（推荐用于pick_server集成）

```bash
# 1. 启动节点（默认等待控制信号）
roslaunch cam_tracker ball_tracker.launch

# 2. 无人机飞到目标区域上空后，发送启动信号
rostopic pub /tracker_control std_msgs/Bool "data: true" -1

# 节点会：
#   - 拍摄一张照片
#   - 识别所有小球
#   - 选择边缘位置的目标
#   - 开始持续追踪

# 3. 监控目标位置
rostopic echo /detection/target

# 4. 抓取完成后，发送停止信号
rostopic pub /tracker_control std_msgs/Bool "data: false" -1

# 节点会：
#   - 停止追踪
#   - 保持模型在GPU中（下次启动更快）
```

### 方式2: 自动启动

```xml
<!-- launch文件中设置 -->
<param name="auto_start" value="true"/>
```

节点启动后立即开始追踪流程。

## 🔍 工作原理

### 目标选择策略

```
1. 拍照并检测所有目标
2. 过滤出目标类型（sports ball）
3. 计算每个目标到边缘的距离
4. 优先级排序：
   ① 在边缘区域的目标（边缘25%区域）
   ② 如无边缘目标，选择距离边缘最近的
   ③ 相同条件下，选择置信度最高的
5. 锁定选中目标，开始追踪
```

### 边缘区域示意图

```
图像边缘区域 (edge_margin_ratio = 0.25)

┌────────────────────────────────────┐
│ ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ │ 
│ ▓▓┌────────────────────────┐▓▓▓▓ │ ← 边缘区域(25%)
│ ▓▓│                        │▓▓▓▓ │
│ ▓▓│      中心区域           │▓▓▓▓ │
│ ▓▓│                        │▓▓▓▓ │
│ ▓▓└────────────────────────┘▓▓▓▓ │
│ ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ │
└────────────────────────────────────┘
```

### 追踪流程图

```
启动信号 (True)
    ↓
【初始化阶段】
    ├─ 拍照
    ├─ YOLO检测
    ├─ 过滤目标类型
    ├─ 选择边缘目标
    └─ 锁定目标ID
    ↓
【追踪阶段】
    ├─ YOLO追踪 (ByteTrack)
    ├─ 查找锁定ID
    ├─ 发布位置 (≥20Hz)
    └─ 检测丢失
        ├─ 未丢失 → 继续追踪
        └─ 丢失 → 重新选择
    ↓
停止信号 (False)
    ↓
【停止阶段】
    ├─ 停止追踪
    └─ 保持模型加载
```

## 🧪 测试方法

### 1. 基础功能测试

```bash
# 启动节点
roslaunch cam_tracker ball_tracker.launch

# 查看状态
rostopic echo /tracker_status

# 启动追踪
rostopic pub /tracker_control std_msgs/Bool "data: true" -1

# 观察日志，应该看到：
# - 📸 拍摄快照并进行目标检测...
# - 🎯 已选择目标: ID=X
# - 📡 阶段2: 开始持续追踪...

# 查看目标位置
rostopic echo /detection/target

# 停止追踪
rostopic pub /tracker_control std_msgs/Bool "data: false" -1
```

### 2. 边缘选择测试

```bash
# 准备测试场景：
# - 在图像中心放置一个球
# - 在图像边缘放置另一个球

# 启动追踪
rostopic pub /tracker_control std_msgs/Bool "data: true" -1

# 检查日志，应该选择边缘位置的球
# 输出类似：✅ 选择边缘区域目标 (置信度: 0.XXX)
```

### 3. 重启测试

```bash
# 测试停止后重新启动的响应速度

# 1. 启动
rostopic pub /tracker_control std_msgs/Bool "data: true" -1
# 等待几秒

# 2. 停止
rostopic pub /tracker_control std_msgs/Bool "data: false" -1

# 3. 再次启动（应该很快，模型已在GPU中）
rostopic pub /tracker_control std_msgs/Bool "data: true" -1
```

## 🔌 与pick_server集成

### pick_server端伪代码

```python
import rospy
from std_msgs.msg import Bool
from cam_tracker.msg import Detection

class PickServer:
    def __init__(self):
        # 控制cam_tracker
        self.tracker_control_pub = rospy.Publisher(
            '/tracker_control', Bool, queue_size=1
        )
        
        # 订阅目标位置
        self.target_sub = rospy.Subscriber(
            '/detection/target', Detection, self.target_callback
        )
        
        # 订阅状态
        self.status_sub = rospy.Subscriber(
            '/tracker_status', String, self.status_callback
        )
    
    def execute_pick(self):
        # 1. 无人机飞到目标区域上空
        self.fly_to_target_area()
        
        # 2. 启动视觉追踪
        rospy.loginfo("启动视觉追踪")
        self.tracker_control_pub.publish(Bool(data=True))
        
        # 3. 等待目标选择完成（通过状态回调判断）
        self.wait_for_target_lock()
        
        # 4. 根据位置数据调整无人机姿态
        while not self.aligned:
            # target_callback中更新target_x, target_y
            self.adjust_position(self.target_x, self.target_y)
        
        # 5. 执行抓取
        self.gripper_close()
        
        # 6. 抓取完成，停止追踪
        rospy.loginfo("停止视觉追踪")
        self.tracker_control_pub.publish(Bool(data=False))
        
        # 7. 返回
        self.fly_home()
    
    def target_callback(self, msg):
        if msg.detection_id == 1:  # 有效目标
            self.target_x = msg.detection_x
            self.target_y = msg.detection_y
        # detection_id == 0 时使用最近数据（目标暂时丢失）
    
    def status_callback(self, msg):
        # 解析状态: "state:phase:description"
        parts = msg.data.split(':')
        state = parts[0]
        phase = parts[1] if len(parts) > 1 else ''
        
        if phase == 'target_selected':
            self.target_locked = True
```

## ⚙️ 高级配置

### 修改目标类型

编辑 `ball_tracker_node.py`:

```python
# 修改这一行
TARGET_CLASS = "sports ball"  # 改为你需要的类别

# YOLO支持的常见类别：
# - "sports ball" (运动球类)
# - "orange" (橙子)
# - "apple" (苹果)
# - "bottle" (瓶子)
# 等等...
```

### 调整边缘区域大小

```xml
<!-- 在launch文件中调整 -->
<!-- 0.25 = 边缘25%区域 -->
<!-- 0.3 = 边缘30%区域 (更宽) -->
<!-- 0.2 = 边缘20%区域 (更窄) -->
<param name="edge_margin_ratio" value="0.25"/>
```

### 使用自定义模型

```xml
<!-- 方式1: 在launch中指定 -->
<param name="model_path" value="/path/to/your/model.pt"/>

<!-- 方式2: 放到标准位置 -->
<!-- ~/catkin_ws/src/cam_tracker/models/your_model.pt -->
```

然后修改代码中的 `MODEL_NAME`:

```python
MODEL_NAME = "your_model.pt"
```

## 📊 性能对比

| 特性 | 旧版本 (ct_clibgrip.py) | 新版本 (ball_tracker_node.py) |
|------|------------------------|----------------------------|
| 目标类型 | Person (已过时) | 可配置 (默认sports ball) |
| 目标选择 | 置信度最高 | 边缘优先 + 智能选择 |
| 启动方式 | Bool控制 | Bool控制 (保持) |
| 停止方式 | Bool控制 | Bool控制 + 优雅关闭 |
| 初始化 | 直接追踪 | 拍照识别 → 选择目标 → 追踪 |
| 状态反馈 | 无 | 完整状态机 + 实时状态 |
| 代码可读性 | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| 文档完整性 | ⭐⭐ | ⭐⭐⭐⭐⭐ |

## 🐛 故障排除

### 问题1: 未检测到目标

**可能原因:**
- 目标类型配置错误
- 置信度阈值太高
- 光线条件不佳

**解决方法:**
```bash
# 1. 检查目标类型
# 确认YOLO模型支持你要检测的类别

# 2. 降低置信度阈值
<param name="confidence_threshold" value="0.3"/>  <!-- 从0.5降到0.3 -->

# 3. 查看所有检测结果
rostopic echo /detection/all_targets
```

### 问题2: 总是选择中心目标

**可能原因:**
- 边缘区域设置太小
- 没有边缘目标

**解决方法:**
```xml
<!-- 增大边缘区域 -->
<param name="edge_margin_ratio" value="0.35"/>  <!-- 从0.25增到0.35 -->
```

### 问题3: 追踪不稳定

**可能原因:**
- 目标移动太快
- 遮挡严重

**解决方法:**
```xml
<!-- 调整追踪器参数 -->
<param name="tracker_type" value="botsort.yaml"/>  <!-- 尝试其他追踪器 -->
<param name="iou_threshold" value="0.3"/>  <!-- 降低IOU阈值 -->
```

## 📝 开发日志

### v2.0.0 (2025-10-31)
- ✅ 完全重构代码
- ✅ 从Person追踪改为通用物体追踪
- ✅ 添加边缘目标优先选择
- ✅ 实现分阶段执行（拍照→选择→追踪）
- ✅ 优化控制接口
- ✅ 完善状态机和错误处理
- ✅ 提升代码质量和文档

### v1.0.0
- ❌ Person追踪（已废弃）
- ❌ 简单的启停控制
- ❌ 无目标选择策略

## 📞 支持

如有问题，请检查：
1. ROS日志输出
2. 话题数据 (`rostopic echo`)
3. 节点状态 (`rosnode info ball_tracker`)
4. 本文档的故障排除章节

## 🎉 最佳实践

1. **测试环境验证**: 先在测试环境充分测试再部署
2. **参数调优**: 根据实际场景调整边缘区域和置信度
3. **日志监控**: 初期使用时保持日志监控
4. **状态检查**: 集成时始终检查状态话题
5. **优雅控制**: 使用完毕后发送停止信号（节省计算资源）
