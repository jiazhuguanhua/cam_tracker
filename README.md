
## 可视化节点

### 无GUI模式（适用于SSH远程环境）

可视化节点会将检测结果绘制在图像上并发布到 `/cam_tracker/visualization` 话题。

**查看方式：**

1. **使用rqt_image_view（推荐）**
```bash
rqt_image_view /cam_tracker/visualization
```

2. **使用rviz**
```bash
rviz
# 添加 Image 显示，选择话题 /cam_tracker/visualization
```

3. **使用web_video_server**
```bash
# 安装（如果没有）
sudo apt-get install ros-noetic-web-video-server

# 启动
rosrun web_video_server web_video_server

# 浏览器访问：http://<your_ip>:8080
```

### 可视化效果

- **Ball（黄色）**: 圆圈 + 十字 + 坐标
- **Car（紫色）**: 矩形框 + 编号 + 坐标  
- **状态栏**: 系统状态、数量统计、夹爪状态
- **FPS**: 每5秒输出一次帧率统计

