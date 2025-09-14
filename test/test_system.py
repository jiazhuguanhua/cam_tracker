#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
简单的摄像头测试脚本
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def test_camera():
    """测试摄像头基本功能"""
    # 尝试打开摄像头
    for device_id in [0, 1, 2]:
        cap = cv2.VideoCapture(device_id)
        if cap.isOpened():
            print(f"摄像头 /dev/video{device_id} 可用")
            
            # 测试读取一帧
            ret, frame = cap.read()
            if ret:
                print(f"成功读取帧，尺寸: {frame.shape}")
                cv2.imshow(f"Camera {device_id}", frame)
                cv2.waitKey(1000)  # 显示1秒
                cv2.destroyAllWindows()
            else:
                print(f"无法读取摄像头 {device_id} 的图像")
            
            cap.release()
        else:
            print(f"摄像头 /dev/video{device_id} 不可用")

def test_yolo():
    """测试YOLO模型"""
    try:
        from ultralytics import YOLO
        model = YOLO('yolo11n.pt')
        print("YOLO模型加载成功")
        
        # 创建测试图像
        test_img = np.zeros((480, 640, 3), dtype=np.uint8)
        results = model(test_img, verbose=False)
        print("YOLO推理测试成功")
        return True
    except Exception as e:
        print(f"YOLO测试失败: {e}")
        return False

if __name__ == '__main__':
    print("=== 摄像头和YOLO测试 ===")
    test_camera()
    test_yolo()
    print("=== 测试完成 ===")