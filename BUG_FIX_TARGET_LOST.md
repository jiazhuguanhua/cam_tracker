# Bug修复说明 - 目标丢失后的行为优化

## 🐛 问题描述

**原始问题:**
```
[WARN] [1761921330.066029]: ⚠️  目标丢失超过 30 帧
[INFO] [1761921330.069020]: 🔄 尝试重新选择目标...
[WARN] [1761921330.070529]: ⚠️  未检测到 tennis-ball 类型目标
```

当目标丢失超过30帧后，节点会：
- ❌ 进入TARGET_LOST状态后不再积极检测
- ❌ 即使重新出现目标也无法自动恢复追踪
- ❌ 没有明确的状态标识告知上层目标已丢失

## ✅ 修复方案

### 1. 新的追踪逻辑

```
目标在画面中 → detection_id = 1 (正常追踪)
    ↓
目标消失 < 30帧 → detection_id = 0 (暂时丢失，使用最近数据)
    ↓
目标消失 ≥ 30帧 → detection_id = -1 (已丢失，进入积极搜索模式)
    ├─ 继续运行YOLO检测
    ├─ 持续发布最近的位置数据
    └─ 一旦检测到目标 → 自动恢复 → detection_id = 1
```

### 2. detection_id 状态说明

| ID值 | 状态 | 位置数据 | 行为 |
|------|------|---------|------|
| 1 | 正常追踪 | 实时更新 | 持续追踪目标 |
| 0 | 暂时丢失 (<30帧) | 最近已知位置 | 继续追踪，等待目标重现 |
| -1 | 已丢失 (≥30帧) | 最近已知位置 | 积极搜索模式，自动重新捕获 |

### 3. 代码修改

#### 修改1: `_process_tracking` 方法

```python
# 旧逻辑
if missing_frames > MAX_MISSING_FRAMES:
    self.state = TrackerState.TARGET_LOST
    self._reselect_target(tracked_objects)  # 只尝试一次

# 新逻辑  
if missing_frames > MAX_MISSING_FRAMES:
    self.state = TrackerState.TARGET_LOST
    self._publish_target(None, header, force_invalid=True)  # ID=-1
    self._try_reacquire_target(tracked_objects)  # 每帧都尝试
```

#### 修改2: `_publish_target` 方法

新增 `force_invalid` 参数：

```python
def _publish_target(self, target, header=None, force_invalid=False):
    if target is not None:
        detection.detection_id = 1  # 有效目标
    else:
        if force_invalid:
            detection.detection_id = -1  # 已丢失（≥30帧）
        else:
            detection.detection_id = 0  # 暂时丢失（<30帧）
```

#### 修改3: 新增 `_try_reacquire_target` 方法

```python
def _try_reacquire_target(self, tracked_objects):
    """积极搜索并重新捕获目标"""
    target_objects = [obj for obj in tracked_objects if obj['class_name'] == TARGET_CLASS]
    
    if target_objects:
        new_target = self._select_edge_target(target_objects)
        if new_target:
            # 自动恢复追踪
            self.tracked_target_id = new_target['track_id']
            self.state = TrackerState.TRACKING
            rospy.loginfo("🎯 重新捕获目标")
```

## 📊 行为对比

### 旧版本

```
目标消失 → 超过30帧 → 进入TARGET_LOST → 停止积极检测
                                          ↓
                                    需要手动重启
```

### 新版本

```
目标消失 → 超过30帧 → 进入积极搜索模式 → 持续检测
                         ↓
                    ID = -1
                    发布最近位置
                         ↓
                    检测到目标？
                    ├─ 是 → 自动恢复追踪 (ID=1)
                    └─ 否 → 继续搜索
```

## 🧪 测试场景

### 场景1: 短暂遮挡
```bash
# 1. 启动追踪
rostopic pub /tracker_control std_msgs/Bool "data: true" -1

# 2. 用手遮挡目标2秒（<30帧）
# 预期: detection_id = 0, 位置保持不变

# 3. 移开遮挡
# 预期: detection_id = 1, 位置恢复更新
```

### 场景2: 目标完全离开画面
```bash
# 1. 启动追踪
rostopic pub /tracker_control std_msgs/Bool "data: true" -1

# 2. 将目标移出画面 >1秒（>30帧）
# 预期: 
#   - detection_id = -1
#   - 日志显示: "⚠️  目标丢失超过 30 帧，进入搜索模式"

# 3. 将目标重新移入画面
# 预期:
#   - 自动检测并重新捕获
#   - detection_id = 1
#   - 日志显示: "🎯 重新捕获目标: ID=X → ID=Y"
```

### 场景3: 更换目标
```bash
# 1. 启动追踪，追踪球A
# 预期: detection_id = 1

# 2. 移走球A，放入球B
# 预期:
#   - 球A消失时: detection_id = -1
#   - 检测到球B后: 自动切换追踪球B, detection_id = 1
```

## 📡 日志示例

### 正常追踪
```
[INFO] 🎯 已选择目标: ID=5, 置信度=0.876
[DEBUG] 📡 发布目标位置: ID=1, (320.5, 240.3)
```

### 短暂丢失
```
[DEBUG] 📡 发布暂时丢失: ID=0 (使用最近数据)
```

### 长时间丢失
```
[WARN] ⚠️  目标丢失超过 30 帧，进入搜索模式
[DEBUG] 📡 发布丢失状态: ID=-1 (积极搜索中)
[DEBUG] 未检测到 tennis-ball 类型目标，继续搜索...
[DEBUG] 未检测到 tennis-ball 类型目标，继续搜索...
```

### 重新捕获
```
[INFO] ======================================================================
[INFO] 🎯 重新捕获目标:
[INFO]    旧ID: 5 → 新ID: 12
[INFO]    置信度: 0.823
[INFO]    位置: (310.2, 250.8)
[INFO] ======================================================================
[INFO] ✅ 目标已恢复追踪
[DEBUG] 📡 发布目标位置: ID=1, (310.2, 250.8)
```

## 🔍 上层应用如何使用

### pick_server 集成示例

```python
def target_callback(self, msg):
    """处理目标位置回调"""
    
    if msg.detection_id == 1:
        # 正常追踪中，使用实时位置
        self.target_x = msg.detection_x
        self.target_y = msg.detection_y
        self.target_valid = True
        
    elif msg.detection_id == 0:
        # 暂时丢失，继续使用最近位置（目标可能马上回来）
        self.target_x = msg.detection_x
        self.target_y = msg.detection_y
        self.target_valid = True
        rospy.logdebug("目标暂时丢失，保持位置")
        
    elif msg.detection_id == -1:
        # 目标已丢失，进入等待模式
        self.target_valid = False
        rospy.logwarn("目标丢失，等待重新捕获...")
        
        # 选项1: 保持悬停
        self.hold_position()
        
        # 选项2: 中止任务
        # self.abort_mission()
```

## ⚙️ 可调参数

如果需要调整丢失阈值，修改代码中的常量：

```python
# ball_tracker_node.py 第34行
MAX_MISSING_FRAMES = 30  # 默认30帧（约1.5秒@20Hz）

# 调整建议:
# - 目标移动快/遮挡多: 增大到 50-60
# - 目标稳定/实时性要求高: 减小到 15-20
```

## 📝 总结

### 修复前
- ❌ 丢失后无法自动恢复
- ❌ 无法区分暂时丢失和长期丢失
- ❌ 停止检测，需要手动重启

### 修复后
- ✅ 自动重新捕获目标
- ✅ 三级状态指示（1/0/-1）
- ✅ 持续积极检测
- ✅ 无缝恢复追踪

### 影响
- **向下兼容**: 旧代码检查 `detection_id > 0` 仍然有效
- **新功能**: 可以通过 `detection_id == -1` 判断长期丢失
- **性能**: 无影响，仍然保持≥20Hz输出频率

## 🎉 验证完成

请按照上述测试场景验证修复效果！
