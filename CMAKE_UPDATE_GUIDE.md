# CMakeLists.txt 更新说明

## 需要添加的内容

在您的 `cam_tracker/CMakeLists.txt` 文件中，需要确保包含新的消息定义。

### 1. 找到 `add_message_files` 部分

找到类似这样的部分：

```cmake
add_message_files(
  FILES
  Detection.msg
  CompleteDetection.msg
  DetectionArray.msg
)
```

### 2. 添加新消息

在现有消息列表中添加 `CamTrack.msg`：

```cmake
add_message_files(
  FILES
  Detection.msg
  CompleteDetection.msg
  DetectionArray.msg
  CamTrack.msg              # <-- 添加这一行
)
```

### 3. 确保依赖正确

确保以下部分存在：

```cmake
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  message_generation      # 确保有这个
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  sensor_msgs
)
```

### 4. 完整的 add_message_files 示例

```cmake
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Detection.msg
  CompleteDetection.msg
  DetectionArray.msg
  CamTrack.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)
```

## 手动操作步骤

1. 用文本编辑器打开 CMakeLists.txt：
   ```bash
   gedit ~/drone_uavgp_ws/src/cam_tracker/CMakeLists.txt
   ```

2. 找到 `add_message_files` 部分

3. 在 `FILES` 列表中添加 `CamTrack.msg`

4. 保存文件

5. 重新编译：
   ```bash
   cd ~/drone_uavgp_ws
   catkin_make
   source devel/setup.bash
   ```

## 自动化脚本

或者运行以下命令自动添加：

```bash
cd ~/drone_uavgp_ws/src/cam_tracker

# 备份原文件
cp CMakeLists.txt CMakeLists.txt.backup

# 在 add_message_files 的 FILES 部分添加 CamTrack.msg
# 注意：这个命令假设 Detection.msg 在消息列表中
sed -i '/Detection.msg/a\  CamTrack.msg' CMakeLists.txt

# 查看更改
diff CMakeLists.txt.backup CMakeLists.txt

# 如果看起来正确，编译
cd ~/drone_uavgp_ws
catkin_make
```

## 验证安装

编译成功后，验证消息是否可用：

```bash
# 刷新环境
source ~/drone_uavgp_ws/devel/setup.bash

# 检查消息定义
rosmsg show cam_tracker/CamTrack

# 应该看到完整的消息定义
```

## 可能的错误及解决方案

### 错误1: "Could not find cam_tracker/CamTrack"

**原因**: 消息未正确编译

**解决**:
```bash
cd ~/drone_uavgp_ws
catkin_make clean
catkin_make
source devel/setup.bash
```

### 错误2: "No rule to make target"

**原因**: CMakeLists.txt 配置错误

**解决**: 检查 `add_message_files` 部分的缩进和语法

### 错误3: Python 无法导入消息

**原因**: 环境变量未刷新

**解决**:
```bash
source ~/drone_uavgp_ws/devel/setup.bash
# 或者在 ~/.bashrc 中添加：
echo "source ~/drone_uavgp_ws/devel/setup.bash" >> ~/.bashrc
```

## 需要的文件列表

确保以下文件存在：

- ✅ `msg/CamTrack.msg`
- ✅ `scripts/cam_tracker_node.py`
- ✅ `scripts/visualizer_node_new.py`
- ✅ `scripts/test_controller.py`
- ✅ `launch/cam_tracker_new.launch`

## 下一步

完成CMakeLists.txt更新后：

1. 编译工作空间
2. 刷新环境
3. 运行快速启动脚本：
   ```bash
   cd ~/drone_uavgp_ws/src/cam_tracker
   chmod +x quick_start_new.sh
   ./quick_start_new.sh
   ```
