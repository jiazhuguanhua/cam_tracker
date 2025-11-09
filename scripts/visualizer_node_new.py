#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Cam Tracker Visualizer Node - 检测可视化节点
===========================================

功能说明：
1. 实时显示ball和car的检测结果
2. 显示检测框、类别标签、追踪ID、置信度
3. 显示系统状态和性能指标
4. 支持键盘控制

话题订阅：
- /usb_cam/image_raw: 摄像头图像
- /cam_tracker/info: 检测结果信息

键盘控制：
- 'q': 退出
- 'p': 暂停/继续
- 's': 保存截图
- 'h': 显示/隐藏帮助信息
- 'd': 显示/隐藏详细信息

作者: ROS Developer
日期: 2025-11-09
版本: 3.0.0
"""

import rospy
import cv2
import numpy as np
import os
import time
from typing import Optional
from collections import deque

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from cam_tracker.msg import CamTrack

# ==================== 配置参数 ====================

# 显示参数
WINDOW_NAME = "Cam Tracker Visualizer"
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720

# 颜色定义 (BGR格式)
COLOR_BALL = (0, 255, 0)            # 绿色 - Ball
COLOR_CAR = (255, 0, 0)             # 蓝色 - Car
COLOR_TARGET_BALL = (0, 255, 255)   # 黄色 - 目标Ball
COLOR_TEXT = (255, 255, 255)        # 白色 - 文字
COLOR_BG = (0, 0, 0)                # 黑色 - 背景
COLOR_OK = (0, 255, 0)              # 绿色 - 正常状态
COLOR_ERROR = (0, 0, 255)           # 红色 - 错误状态

# 绘制参数
BOX_THICKNESS = 2
CIRCLE_RADIUS = 8
TEXT_FONT = cv2.FONT_HERSHEY_SIMPLEX
TEXT_SCALE = 0.6
TEXT_THICKNESS = 2

# 性能监控
FPS_BUFFER_SIZE = 30


# ==================== 工具函数 ====================

def draw_text_with_background(img, text, pos, font_scale=TEXT_SCALE, thickness=TEXT_THICKNESS,
                               text_color=COLOR_TEXT, bg_color=COLOR_BG, padding=5):
    """绘制带背景的文字"""
    x, y = pos
    (text_width, text_height), baseline = cv2.getTextSize(
        text, TEXT_FONT, font_scale, thickness
    )
    
    cv2.rectangle(
        img,
        (x - padding, y - text_height - padding),
        (x + text_width + padding, y + baseline + padding),
        bg_color,
        -1
    )
    
    cv2.putText(
        img, text, (x, y),
        TEXT_FONT, font_scale, text_color, thickness, cv2.LINE_AA
    )
    
    return text_height + baseline + 2 * padding


# ==================== 主节点类 ====================

class CamTrackerVisualizer:
    """检测可视化节点"""
    
    def __init__(self):
        """初始化节点"""
        rospy.init_node('cam_tracker_visualizer', anonymous=True)
        
        rospy.loginfo("=" * 70)
        rospy.loginfo("🎥 Cam Tracker可视化节点启动中...")
        rospy.loginfo("=" * 70)
        
        # ========== 状态变量 ==========
        self.latest_image = None
        self.cam_track_data = None
        
        self.paused = False
        self.show_help = True
        self.show_details = True
        
        # 性能监控
        self.fps_buffer = deque(maxlen=FPS_BUFFER_SIZE)
        self.last_frame_time = time.time()
        self.frame_count = 0
        
        # ========== ROS参数 ==========
        self.camera_topic = rospy.get_param('~camera_topic', '/usb_cam/image_raw')
        
        rospy.loginfo(f"📷 相机话题: {self.camera_topic}")
        
        # ========== 初始化ROS通信 ==========
        self.bridge = CvBridge()
        
        # 订阅器
        self.image_sub = rospy.Subscriber(
            self.camera_topic,
            Image,
            self._image_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        self.info_sub = rospy.Subscriber(
            '/cam_tracker/info',
            CamTrack,
            self._info_callback,
            queue_size=10
        )
        
        rospy.loginfo("=" * 70)
        rospy.loginfo("✅ 节点初始化完成")
        rospy.loginfo("📡 订阅话题:")
        rospy.loginfo(f"   {self.camera_topic}")
        rospy.loginfo(f"   /cam_tracker/info")
        rospy.loginfo("=" * 70)
        rospy.loginfo("⌨️  键盘控制:")
        rospy.loginfo("   'q': 退出")
        rospy.loginfo("   'p': 暂停/继续")
        rospy.loginfo("   's': 保存截图")
        rospy.loginfo("   'h': 显示/隐藏帮助")
        rospy.loginfo("   'd': 显示/隐藏详细信息")
        rospy.loginfo("=" * 70)
    
    # ==================== 回调函数 ====================
    
    def _image_callback(self, msg: Image):
        """图像回调"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"图像转换错误: {e}")
    
    def _info_callback(self, msg: CamTrack):
        """检测信息回调"""
        self.cam_track_data = msg
    
    # ==================== 绘制方法 ====================
    
    def _draw_balls(self, img):
        """绘制ball检测结果"""
        if self.cam_track_data is None or self.cam_track_data.ball_num == 0:
            return
        
        # 绘制目标ball（最适合抓取的）
        x = int(self.cam_track_data.ball_x)
        y = int(self.cam_track_data.ball_y)
        
        # 绘制圆圈标记
        cv2.circle(img, (x, y), CIRCLE_RADIUS, COLOR_TARGET_BALL, -1)
        cv2.circle(img, (x, y), CIRCLE_RADIUS + 5, COLOR_TARGET_BALL, 2)
        
        # 绘制十字准星
        cross_size = 20
        cv2.line(img, (x - cross_size, y), (x + cross_size, y), COLOR_TARGET_BALL, 2)
        cv2.line(img, (x, y - cross_size), (x, y + cross_size), COLOR_TARGET_BALL, 2)
        
        # 绘制标签
        label = f"TARGET BALL ({x}, {y})"
        draw_text_with_background(
            img, label, (x + 25, y - 10),
            font_scale=0.6, thickness=2, text_color=COLOR_TARGET_BALL
        )
        
        # 如果有多个ball，显示数量
        if self.cam_track_data.ball_num > 1:
            count_label = f"Total: {self.cam_track_data.ball_num} balls"
            draw_text_with_background(
                img, count_label, (x + 25, y + 15),
                font_scale=0.5, thickness=1, text_color=COLOR_BALL
            )
    
    def _draw_cars(self, img):
        """绘制car检测结果"""
        if self.cam_track_data is None or self.cam_track_data.car_num == 0:
            return
        
        # 绘制所有car
        for i in range(self.cam_track_data.car_num):
            x = int(self.cam_track_data.car_x[i])
            y = int(self.cam_track_data.car_y[i])
            
            # 绘制圆圈标记
            cv2.circle(img, (x, y), CIRCLE_RADIUS, COLOR_CAR, -1)
            cv2.circle(img, (x, y), CIRCLE_RADIUS + 5, COLOR_CAR, 2)
            
            # 绘制标签
            label = f"CAR #{i} ({x}, {y})"
            draw_text_with_background(
                img, label, (x + 20, y - 10),
                font_scale=0.5, thickness=1, text_color=COLOR_CAR
            )
    
    def _draw_info_panel(self, img):
        """绘制信息面板"""
        # 计算FPS
        current_time = time.time()
        frame_time = current_time - self.last_frame_time
        self.last_frame_time = current_time
        self.fps_buffer.append(1.0 / frame_time if frame_time > 0 else 0)
        avg_fps = np.mean(self.fps_buffer) if len(self.fps_buffer) > 0 else 0
        
        # 系统状态
        if self.cam_track_data:
            system_status = "OK" if self.cam_track_data.system_ok else "ERROR"
            status_color = COLOR_OK if self.cam_track_data.system_ok else COLOR_ERROR
        else:
            system_status = "WAITING"
            status_color = (255, 255, 0)  # 黄色
        
        # 准备信息文本
        info_lines = [
            f"FPS: {avg_fps:.1f}",
            f"Frame: {self.frame_count}",
            f"System: {system_status}",
        ]
        
        if self.show_details and self.cam_track_data:
            info_lines.append(f"Balls: {self.cam_track_data.ball_num}")
            info_lines.append(f"Cars: {self.cam_track_data.car_num}")
            
            if self.cam_track_data.ball_num > 0:
                info_lines.append(f"Target Ball: ({self.cam_track_data.ball_x:.1f}, {self.cam_track_data.ball_y:.1f})")
        
        # 绘制信息面板
        panel_x = 10
        panel_y = 30
        
        for line in info_lines:
            if "System:" in line:
                panel_y += draw_text_with_background(
                    img, line, (panel_x, panel_y),
                    text_color=status_color
                )
            else:
                panel_y += draw_text_with_background(
                    img, line, (panel_x, panel_y)
                )
    
    def _draw_help(self, img):
        """绘制帮助信息"""
        if not self.show_help:
            return
        
        height, width = img.shape[:2]
        
        help_lines = [
            "Keyboard Controls:",
            "  Q - Quit",
            "  P - Pause/Resume",
            "  S - Save Screenshot",
            "  H - Toggle Help",
            "  D - Toggle Details",
        ]
        
        # 在右上角绘制
        help_x = width - 250
        help_y = 30
        
        for line in help_lines:
            help_y += draw_text_with_background(
                img, line, (help_x, help_y),
                font_scale=0.5, thickness=1
            )
    
    def _draw_paused_overlay(self, img):
        """绘制暂停提示"""
        if not self.paused:
            return
        
        height, width = img.shape[:2]
        
        # 半透明黑色遮罩
        overlay = img.copy()
        cv2.rectangle(overlay, (0, 0), (width, height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.3, img, 0.7, 0, img)
        
        # 暂停文字
        text = "PAUSED - Press 'P' to Resume"
        (text_width, text_height), _ = cv2.getTextSize(
            text, TEXT_FONT, 1.5, 3
        )
        x = (width - text_width) // 2
        y = (height + text_height) // 2
        
        draw_text_with_background(
            img, text, (x, y),
            font_scale=1.5, thickness=3,
            text_color=(0, 255, 255)
        )
    
    # ==================== 主循环 ====================
    
    def run(self):
        """主循环"""
        rospy.loginfo("🚀 可视化节点运行中...")
        
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, WINDOW_WIDTH, WINDOW_HEIGHT)
        
        rate = rospy.Rate(50)  # 50 Hz
        
        while not rospy.is_shutdown():
            if self.latest_image is not None:
                self.frame_count += 1
                
                # 创建显示图像
                display_img = self.latest_image.copy()
                
                if not self.paused:
                    # 绘制检测结果
                    self._draw_balls(display_img)
                    self._draw_cars(display_img)
                
                # 绘制信息
                self._draw_info_panel(display_img)
                self._draw_help(display_img)
                self._draw_paused_overlay(display_img)
                
                # 显示图像
                cv2.imshow(WINDOW_NAME, display_img)
            
            # 处理键盘输入
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                rospy.loginfo("⌨️  按下'q'，退出程序")
                break
            elif key == ord('p'):
                self.paused = not self.paused
                status = "暂停" if self.paused else "继续"
                rospy.loginfo(f"⏸️  {status}")
            elif key == ord('s'):
                self._save_screenshot()
            elif key == ord('h'):
                self.show_help = not self.show_help
                status = "显示" if self.show_help else "隐藏"
                rospy.loginfo(f"ℹ️  {status}帮助信息")
            elif key == ord('d'):
                self.show_details = not self.show_details
                status = "显示" if self.show_details else "隐藏"
                rospy.loginfo(f"📊 {status}详细信息")
            
            rate.sleep()
        
        self.shutdown()
    
    def _save_screenshot(self):
        """保存当前帧为图片"""
        if self.latest_image is None:
            rospy.logwarn("⚠️  没有可用的图像")
            return
        
        from datetime import datetime
        
        # 创建截图目录
        screenshot_dir = os.path.expanduser("~/tracker_screenshots")
        os.makedirs(screenshot_dir, exist_ok=True)
        
        # 生成文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"screenshot_{timestamp}.png"
        filepath = os.path.join(screenshot_dir, filename)
        
        # 保存图像
        display_img = self.latest_image.copy()
        self._draw_balls(display_img)
        self._draw_cars(display_img)
        self._draw_info_panel(display_img)
        
        cv2.imwrite(filepath, display_img)
        rospy.loginfo(f"📸 截图已保存: {filepath}")
    
    def shutdown(self):
        """关闭节点"""
        rospy.loginfo("🛑 关闭可视化节点...")
        cv2.destroyAllWindows()
        rospy.loginfo("✅ 节点已关闭")


# ==================== 主函数 ====================

def main():
    """主函数"""
    try:
        visualizer = CamTrackerVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 ROS中断")
    except Exception as e:
        rospy.logerr(f"❌ 节点启动失败: {e}")
        import traceback
        rospy.logerr(traceback.format_exc())
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
