# 夹取区域判断功能更新说明

## 📋 更新内容

### 1. 新增消息字段

在 `Detection.msg` 中新增了 `in_gripper_zone` 字段：

```msg
uint8 detection_id
float32 detection_x
float32 detection_y
bool in_gripper_zone  # true=在夹取区域，false=不在夹取区域
```

### 2. 夹取区域判断逻辑

- **分界线定义**：水平线，位置由参数 `gripper_zone_y_ratio` 控制（默认 0.6，即图像高度的 60% 位置）
- **判断规则**：
  - 目标中心点在分界线**下方** → `in_gripper_zone = true` ✓ 在夹取区域
  - 目标中心点在分界线**上方** → `in_gripper_zone = false` ✗ 不在夹取区域

### 3. 可视化节点重写

全新的 `visualizer_node.py` 提供：

#### 实时显示功能
- ✅ 追踪目标位置（圆圈 + 十字准星）
- ✅ 夹爪区域分界线（橙色水平线）
- ✅ 所有检测目标边界框
- ✅ 追踪器状态信息
- ✅ 性能指标（FPS、延迟）
- ✅ 区域状态标识（IN ZONE / OUT ZONE）

#### 颜色编码
- 🟢 **绿色**：在夹取区域
- 🔴 **红色**：不在夹取区域
- 🟡 **黄色**：暂时丢失（ID=0）
- ⚪ **灰色**：目标丢失（ID=-1）
- 🟣 **紫色**：其他检测目标
- 🟠 **橙色**：夹爪分界线

#### 键盘控制
- `q` - 退出程序
- `p` - 暂停/继续
- `s` - 保存截图（保存到 `~/tracker_screenshots/`）
- `h` - 显示/隐藏帮助信息
- `d` - 显示/隐藏详细信息（检测数量、处理时间等）
- `b` - 显示/隐藏所有检测框

---

## 🚀 使用方法

### 步骤 1: 重新编译工作空间

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 步骤 2: 启动相机节点

```bash
roslaunch usb_cam usb_cam-test.launch
```

### 步骤 3: 启动追踪节点

```bash
# 使用默认参数（分界线在60%位置）
rosrun cam_tracker ball_tracker_node.py

# 或自定义分界线位置（例如50%）
rosrun cam_tracker ball_tracker_node.py _gripper_zone_y_ratio:=0.5
```

### 步骤 4: 启动可视化节点

```bash
# 使用默认参数
rosrun cam_tracker visualizer_node.py

# 或自定义分界线位置（必须与追踪节点一致）
rosrun cam_tracker visualizer_node.py _gripper_zone_y_ratio:=0.5
```

### 步骤 5: 控制追踪器

```bash
# 启动追踪
rostopic pub /tracker_control std_msgs/Bool "data: true"

# 停止追踪
rostopic pub /tracker_control std_msgs/Bool "data: false"
```

---

## 📊 话题接口

### 发布话题：`/detection/target`

```bash
rostopic echo /detection/target
```

**消息内容示例**：
```yaml
detection_id: 1        # 1=追踪中, 0=暂时丢失, -1=已丢失
detection_x: 320.5     # 目标中心X坐标
detection_y: 450.2     # 目标中心Y坐标
in_gripper_zone: true  # true=在夹取区域, false=不在夹取区域
```

---

## ⚙️ 参数配置

### ball_tracker_node 参数

```yaml
~gripper_zone_y_ratio: 0.6  # 夹爪分界线位置（0.0-1.0）
~camera_topic: /usb_cam/image_raw
~confidence_threshold: 0.5
~edge_margin_ratio: 0.25
```

**分界线位置示例**：
- `0.3` → 分界线在图像上方 30% 位置（夹取区域较大）
- `0.5` → 分界线在图像中间
- `0.6` → 分界线在图像下方 60% 位置（默认，夹取区域较小）
- `0.8` → 分界线在图像下方 80% 位置（夹取区域很小）

### visualizer_node 参数

```yaml
~gripper_zone_y_ratio: 0.6  # 必须与追踪节点一致
~camera_topic: /usb_cam/image_raw
```

---

## 🔧 集成示例

### 在 pick_server 中使用

```python
#!/usr/bin/env python3
import rospy
from cam_tracker.msg import Detection

def target_callback(msg):
    """处理目标位置信息"""
    if msg.detection_id == 1:
        # 正常追踪
        x, y = msg.detection_x, msg.detection_y
        in_zone = msg.in_gripper_zone
        
        if in_zone:
            rospy.loginfo(f"✓ 目标在夹取区域: ({x:.1f}, {y:.1f})")
            # 执行夹取动作
            execute_grab(x, y)
        else:
            rospy.loginfo(f"✗ 目标不在夹取区域: ({x:.1f}, {y:.1f})")
            # 调整无人机位置
            adjust_position(x, y)
    
    elif msg.detection_id == 0:
        rospy.logwarn("⚠️ 目标暂时丢失，保持当前位置")
        # 保持位置，等待恢复
        
    elif msg.detection_id == -1:
        rospy.logwarn("⚠️ 目标已丢失，搜索中...")
        # 停止夹取，等待重新捕获
        abort_grab()

# 订阅目标话题
rospy.Subscriber('/detection/target', Detection, target_callback)
```

---

## 📸 可视化界面说明

### 左上角信息面板
```
FPS: 28.5              # 实时帧率
Frame: 1234            # 帧计数
Status: tracking...    # 追踪器状态
Detections: 3          # 检测目标数量（按'd'切换）
Process Time: 45.2ms   # 处理时间（按'd'切换）
Target ID: 1           # 当前目标ID
```

### 右上角帮助面板（按 'h' 切换）
```
Keyboard Controls:
  Q - Quit
  P - Pause/Resume
  S - Save Screenshot
  H - Toggle Help
  D - Toggle Details
  B - Toggle Boxes
```

### 图像标注
- **橙色水平线**：夹爪区域分界线
  - 上方标注：`Out of Zone`
  - 下方标注：`In Gripper Zone`
  - 线上标签：`Gripper Zone Line (60%)`

- **目标标注**：
  - 圆圈 + 十字准星（颜色根据状态变化）
  - 状态标签：`TRACKING | ✓ IN ZONE` 或 `TRACKING | ✗ OUT ZONE`
  - 坐标信息：`(320, 450)`

- **其他目标**（按 'b' 切换）：
  - 紫色边界框
  - 标签：`tennis-ball ID:2 0.85`

---

## 🎯 典型应用场景

### 1. 无人机球抓取

```bash
# 使用较小的夹取区域（分界线在70%位置）
rosrun cam_tracker ball_tracker_node.py _gripper_zone_y_ratio:=0.7
rosrun cam_tracker visualizer_node.py _gripper_zone_y_ratio:=0.7
```

**原理**：
- 无人机从上方接近目标
- 当目标进入图像下方 30% 区域时（`in_gripper_zone=true`）
- 触发夹爪抓取动作

### 2. 机械臂抓取

```bash
# 使用较大的夹取区域（分界线在50%位置）
rosrun cam_tracker ball_tracker_node.py _gripper_zone_y_ratio:=0.5
rosrun cam_tracker visualizer_node.py _gripper_zone_y_ratio:=0.5
```

**原理**：
- 机械臂视角固定
- 当目标进入图像下半部分时（`in_gripper_zone=true`）
- 表示目标在机械臂可达范围内

---

## 🧪 测试方法

### 测试 1: 验证分界线显示

```bash
# 启动可视化节点
rosrun cam_tracker visualizer_node.py

# 检查：
# ✓ 是否显示橙色水平线
# ✓ 分界线位置是否正确（默认60%）
# ✓ 区域标签是否正确
```

### 测试 2: 验证区域判断

```bash
# 启动追踪节点
rostopic pub /tracker_control std_msgs/Bool "data: true"

# 监听目标话题
rostopic echo /detection/target

# 移动目标：
# 1. 将目标放在图像上方 → in_gripper_zone: false（红色标注）
# 2. 将目标放在图像下方 → in_gripper_zone: true（绿色标注）
```

### 测试 3: 验证键盘控制

```bash
# 在可视化窗口中测试：
# - 按 'p' 暂停/继续
# - 按 's' 保存截图（检查 ~/tracker_screenshots/）
# - 按 'h' 隐藏帮助
# - 按 'd' 隐藏详细信息
# - 按 'b' 隐藏检测框
# - 按 'q' 退出
```

---

## 📝 注意事项

1. **参数一致性**：
   - `ball_tracker_node` 和 `visualizer_node` 的 `gripper_zone_y_ratio` **必须一致**
   - 否则可视化显示的分界线与实际判断不符

2. **坐标系说明**：
   - 图像坐标系：原点在左上角
   - Y 轴向下增加
   - 分界线 Y 坐标 = `图像高度 × gripper_zone_y_ratio`

3. **性能考虑**：
   - 可视化节点运行在 50 Hz
   - 不会影响追踪节点性能
   - 可随时关闭可视化节点

4. **截图保存**：
   - 截图保存在 `~/tracker_screenshots/`
   - 文件名格式：`screenshot_YYYYMMDD_HHMMSS.png`
   - 包含所有可视化标注

---

## 🐛 故障排除

### 问题 1: 可视化窗口不显示

```bash
# 检查话题是否正常发布
rostopic list | grep detection
rostopic hz /detection/target

# 检查图像话题
rostopic hz /usb_cam/image_raw
```

### 问题 2: 分界线位置不对

```bash
# 检查参数值
rosparam get /ball_tracker_node/gripper_zone_y_ratio
rosparam get /ball_tracker_visualizer/gripper_zone_y_ratio

# 确保两个值相同
```

### 问题 3: 区域判断不准确

```bash
# 验证目标坐标
rostopic echo /detection/target

# 手动计算：
# 分界线 Y = 图像高度 × gripper_zone_y_ratio
# 例如：480 × 0.6 = 288
# 如果 detection_y > 288 → in_gripper_zone = true
```

---

## 📚 相关文档

- [BALL_TRACKER_GUIDE.md](./BALL_TRACKER_GUIDE.md) - 球追踪节点完整使用指南
- [BUG_FIX_TARGET_LOST.md](./BUG_FIX_TARGET_LOST.md) - 目标丢失修复说明
- [README.md](./README.md) - 项目总体说明

---

**更新时间**: 2025-11-01  
**版本**: 2.0.0
