#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Ball Tracker Visualizer Node - 小球追踪可视化节点
================================================

功能说明：
1. 实时显示追踪目标位置和状态
2. 显示夹爪区域分界线
3. 显示所有检测到的目标（边界框、ID、置信度）
4. 显示追踪器状态信息
5. 显示性能指标（FPS、延迟）
6. 支持键盘控制（暂停、保存截图等）

话题订阅：
- /usb_cam/image_raw: 摄像头图像
- /detection/target: 追踪目标位置
- /detection/all_targets: 所有检测目标
- /tracker_status: 追踪器状态

键盘控制：
- 'q': 退出
- 'p': 暂停/继续
- 's': 保存当前帧
- 'h': 显示/隐藏帮助信息
- 'd': 显示/隐藏详细信息
- 'b': 显示/隐藏所有检测框

作者: ROS Developer
日期: 2025-11-01
版本: 2.0.0
"""

import rospy
import cv2
import numpy as np
import time
from typing import Optional
from collections import deque

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from cam_tracker.msg import Detection, DetectionArray

# ==================== 配置参数 ====================

# 显示参数
WINDOW_NAME = "Ball Tracker Visualizer"
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720

# 颜色定义 (BGR格式)
COLOR_TARGET = (0, 255, 0)          # 绿色 - 追踪目标
COLOR_GRIPPER_LINE = (0, 165, 255)  # 橙色 - 夹爪分界线
COLOR_IN_ZONE = (0, 255, 0)         # 绿色 - 在夹取区域
COLOR_OUT_ZONE = (0, 0, 255)        # 红色 - 不在夹取区域
COLOR_ALL_TARGETS = (255, 0, 255)   # 紫色 - 其他目标
COLOR_TEXT = (255, 255, 255)        # 白色 - 文字
COLOR_BG = (0, 0, 0)                # 黑色 - 背景

# 绘制参数
TARGET_CIRCLE_RADIUS = 15
TARGET_CIRCLE_THICKNESS = 3
LINE_THICKNESS = 2
TEXT_FONT = cv2.FONT_HERSHEY_SIMPLEX
TEXT_SCALE = 0.6
TEXT_THICKNESS = 2

# 性能监控
FPS_BUFFER_SIZE = 30


# ==================== 工具函数 ====================

def draw_text_with_background(img, text, pos, font_scale=TEXT_SCALE, thickness=TEXT_THICKNESS,
                               text_color=COLOR_TEXT, bg_color=COLOR_BG, padding=5):
    """
    绘制带背景的文字
    
    Args:
        img: 图像
        text: 文字内容
        pos: 位置 (x, y)
        font_scale: 字体大小
        thickness: 字体粗细
        text_color: 文字颜色
        bg_color: 背景颜色
        padding: 内边距
    """
    x, y = pos
    (text_width, text_height), baseline = cv2.getTextSize(
        text, TEXT_FONT, font_scale, thickness
    )
    
    # 绘制背景矩形
    cv2.rectangle(
        img,
        (x - padding, y - text_height - padding),
        (x + text_width + padding, y + baseline + padding),
        bg_color,
        -1
    )
    
    # 绘制文字
    cv2.putText(
        img, text, (x, y),
        TEXT_FONT, font_scale, text_color, thickness, cv2.LINE_AA
    )
    
    return text_height + baseline + 2 * padding


# ==================== 主节点类 ====================

class BallTrackerVisualizer:
    """小球追踪可视化节点"""
    
    def __init__(self):
        """初始化节点"""
        rospy.init_node('ball_tracker_visualizer', anonymous=True)
        
        rospy.loginfo("=" * 70)
        rospy.loginfo("🎥 小球追踪可视化节点启动中...")
        rospy.loginfo("=" * 70)
        
        # ========== 状态变量 ==========
        self.latest_image = None
        self.target_data = None
        self.all_targets_data = None
        self.tracker_status = "等待数据..."
        
        self.paused = False
        self.show_help = True
        self.show_details = True
        self.show_all_boxes = True
        
        # 性能监控
        self.fps_buffer = deque(maxlen=FPS_BUFFER_SIZE)
        self.last_frame_time = time.time()
        self.frame_count = 0
        
        # ========== ROS参数 ==========
        self.camera_topic = rospy.get_param('~camera_topic', '/usb_cam/image_raw')
        self.gripper_zone_y_ratio = rospy.get_param('~gripper_zone_y_ratio', 0.6)
        
        rospy.loginfo(f"📷 相机话题: {self.camera_topic}")
        rospy.loginfo(f"✂️  夹爪分界线: {self.gripper_zone_y_ratio * 100:.0f}%")
        
        # ========== 初始化ROS通信 ==========
        self.bridge = CvBridge()
        
        # 订阅器
        self.image_sub = rospy.Subscriber(
            self.camera_topic,
            Image,
            self._image_callback,
            queue_size=1,
            buff_size=2**24  # 16MB buffer
        )
        
        self.target_sub = rospy.Subscriber(
            '/detection/target',
            Detection,
            self._target_callback,
            queue_size=10
        )
        
        self.all_targets_sub = rospy.Subscriber(
            '/detection/all_targets',
            DetectionArray,
            self._all_targets_callback,
            queue_size=10
        )
        
        self.status_sub = rospy.Subscriber(
            '/tracker_status',
            String,
            self._status_callback,
            queue_size=10
        )
        
        rospy.loginfo("=" * 70)
        rospy.loginfo("✅ 节点初始化完成")
        rospy.loginfo("📡 订阅话题:")
        rospy.loginfo(f"   {self.camera_topic}")
        rospy.loginfo(f"   /detection/target")
        rospy.loginfo(f"   /detection/all_targets")
        rospy.loginfo(f"   /tracker_status")
        rospy.loginfo("=" * 70)
        rospy.loginfo("⌨️  键盘控制:")
        rospy.loginfo("   'q': 退出")
        rospy.loginfo("   'p': 暂停/继续")
        rospy.loginfo("   's': 保存截图")
        rospy.loginfo("   'h': 显示/隐藏帮助")
        rospy.loginfo("   'd': 显示/隐藏详细信息")
        rospy.loginfo("   'b': 显示/隐藏检测框")
        rospy.loginfo("=" * 70)
    
    # ==================== 回调函数 ====================
    
    def _image_callback(self, msg: Image):
        """图像回调"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"图像转换错误: {e}")
    
    def _target_callback(self, msg: Detection):
        """目标数据回调"""
        self.target_data = msg
    
    def _all_targets_callback(self, msg: DetectionArray):
        """所有目标数据回调"""
        self.all_targets_data = msg
    
    def _status_callback(self, msg: String):
        """状态回调"""
        self.tracker_status = msg.data
    
    # ==================== 绘制方法 ====================
    
    def _draw_gripper_zone_line(self, img):
        """绘制夹爪区域分界线"""
        height, width = img.shape[:2]
        y_line = int(height * self.gripper_zone_y_ratio)
        
        # 绘制水平线
        cv2.line(
            img,
            (0, y_line),
            (width, y_line),
            COLOR_GRIPPER_LINE,
            LINE_THICKNESS
        )
        
        # 绘制标签
        draw_text_with_background(
            img,
            f"Gripper Zone Line ({self.gripper_zone_y_ratio * 100:.0f}%)",
            (10, y_line - 10),
            font_scale=0.5,
            thickness=1,
            text_color=COLOR_GRIPPER_LINE
        )
        
        # 绘制区域标签
        draw_text_with_background(
            img,
            "Out of Zone",
            (width - 150, y_line - 30),
            font_scale=0.5,
            thickness=1,
            text_color=COLOR_OUT_ZONE
        )
        
        draw_text_with_background(
            img,
            "In Gripper Zone",
            (width - 180, y_line + 30),
            font_scale=0.5,
            thickness=1,
            text_color=COLOR_IN_ZONE
        )
    
    def _draw_target(self, img):
        """绘制追踪目标"""
        if self.target_data is None:
            return
        
        x = int(self.target_data.detection_x)
        y = int(self.target_data.detection_y)
        detection_id = self.target_data.detection_id
        in_zone = self.target_data.in_gripper_zone
        
        # 根据状态选择颜色
        if detection_id == 1:
            # 正常追踪
            color = COLOR_IN_ZONE if in_zone else COLOR_OUT_ZONE
            status_text = "TRACKING"
        elif detection_id == 0:
            # 暂时丢失
            color = (0, 255, 255)  # 黄色
            status_text = "TEMP LOST"
        else:  # detection_id == -1
            # 目标丢失
            color = (128, 128, 128)  # 灰色
            status_text = "LOST"
        
        # 绘制目标圆圈
        cv2.circle(
            img,
            (x, y),
            TARGET_CIRCLE_RADIUS,
            color,
            TARGET_CIRCLE_THICKNESS
        )
        
        # 绘制十字准星
        cross_size = 25
        cv2.line(img, (x - cross_size, y), (x + cross_size, y), color, 2)
        cv2.line(img, (x, y - cross_size), (x, y + cross_size), color, 2)
        
        # 绘制状态标签
        zone_text = "✓ IN ZONE" if in_zone else "✗ OUT ZONE"
        label = f"{status_text} | {zone_text}"
        draw_text_with_background(
            img,
            label,
            (x + 20, y - 20),
            font_scale=0.6,
            thickness=2,
            text_color=color
        )
        
        # 绘制坐标
        coord_text = f"({x}, {y})"
        draw_text_with_background(
            img,
            coord_text,
            (x + 20, y + 5),
            font_scale=0.5,
            thickness=1,
            text_color=color
        )
    
    def _draw_all_targets(self, img):
        """绘制所有检测目标"""
        if not self.show_all_boxes or self.all_targets_data is None:
            return
        
        for det in self.all_targets_data.detections:
            # 跳过当前追踪目标（已单独绘制）
            if (self.target_data and 
                abs(det.center_x - self.target_data.detection_x) < 5 and
                abs(det.center_y - self.target_data.detection_y) < 5):
                continue
            
            # 绘制边界框
            x1, y1, x2, y2 = [int(v) for v in det.xyxy]
            cv2.rectangle(img, (x1, y1), (x2, y2), COLOR_ALL_TARGETS, 2)
            
            # 绘制标签
            label = f"{det.class_name} ID:{det.id} {det.confidence:.2f}"
            draw_text_with_background(
                img,
                label,
                (x1, y1 - 10),
                font_scale=0.4,
                thickness=1,
                text_color=COLOR_ALL_TARGETS
            )
    
    def _draw_info_panel(self, img):
        """绘制信息面板"""
        height, width = img.shape[:2]
        
        # 计算FPS
        current_time = time.time()
        frame_time = current_time - self.last_frame_time
        self.last_frame_time = current_time
        self.fps_buffer.append(1.0 / frame_time if frame_time > 0 else 0)
        avg_fps = np.mean(self.fps_buffer) if len(self.fps_buffer) > 0 else 0
        
        # 准备信息文本
        info_lines = [
            f"FPS: {avg_fps:.1f}",
            f"Frame: {self.frame_count}",
            f"Status: {self.tracker_status}",
        ]
        
        if self.show_details and self.all_targets_data:
            info_lines.extend([
                f"Detections: {self.all_targets_data.total_objects}",
                f"Process Time: {self.all_targets_data.processing_time*1000:.1f}ms",
            ])
        
        if self.target_data:
            info_lines.append(f"Target ID: {self.target_data.detection_id}")
        
        # 绘制信息面板
        panel_x = 10
        panel_y = 30
        
        for line in info_lines:
            panel_y += draw_text_with_background(
                img, line, (panel_x, panel_y),
                font_scale=0.5, thickness=1
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
            "  B - Toggle Boxes",
        ]
        
        # 在右上角绘制
        help_x = width - 250
        help_y = 30
        
        for line in help_lines:
            help_y += draw_text_with_background(
                img, line, (help_x, help_y),
                font_scale=0.4, thickness=1,
                text_color=(200, 200, 200)
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
            if self.latest_image is not None and not self.paused:
                # 复制图像用于绘制
                display_img = self.latest_image.copy()
                
                # 绘制各个元素
                self._draw_gripper_zone_line(display_img)
                self._draw_all_targets(display_img)
                self._draw_target(display_img)
                self._draw_info_panel(display_img)
                self._draw_help(display_img)
                
                # 显示图像
                cv2.imshow(WINDOW_NAME, display_img)
                self.frame_count += 1
            
            elif self.latest_image is not None and self.paused:
                # 暂停状态：显示最后一帧
                display_img = self.latest_image.copy()
                self._draw_gripper_zone_line(display_img)
                self._draw_all_targets(display_img)
                self._draw_target(display_img)
                self._draw_info_panel(display_img)
                self._draw_help(display_img)
                self._draw_paused_overlay(display_img)
                cv2.imshow(WINDOW_NAME, display_img)
            
            # 处理键盘输入
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                rospy.loginfo("🛑 用户退出")
                break
            elif key == ord('p'):
                self.paused = not self.paused
                status = "暂停" if self.paused else "继续"
                rospy.loginfo(f"⏸️  {status}")
            elif key == ord('s'):
                self._save_screenshot()
            elif key == ord('h'):
                self.show_help = not self.show_help
                rospy.loginfo(f"📖 帮助: {'显示' if self.show_help else '隐藏'}")
            elif key == ord('d'):
                self.show_details = not self.show_details
                rospy.loginfo(f"📊 详细信息: {'显示' if self.show_details else '隐藏'}")
            elif key == ord('b'):
                self.show_all_boxes = not self.show_all_boxes
                rospy.loginfo(f"🔲 检测框: {'显示' if self.show_all_boxes else '隐藏'}")
            
            rate.sleep()
        
        self.shutdown()
    
    def _save_screenshot(self):
        """保存当前帧为图片"""
        if self.latest_image is None:
            rospy.logwarn("⚠️  无图像可保存")
            return
        
        import os
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
        self._draw_gripper_zone_line(display_img)
        self._draw_all_targets(display_img)
        self._draw_target(display_img)
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
        visualizer = BallTrackerVisualizer()
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
