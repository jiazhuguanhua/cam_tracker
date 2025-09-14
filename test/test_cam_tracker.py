#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
测试脚本：验证更新后的cam_tracker_node是否能正常启动
"""

import sys
import os

# 添加路径以便导入模块
sys.path.append('/home/micoair/drone_vision_ws/src/cam_tracker/scripts')

def test_imports():
    """测试所有必要的import是否正常"""
    try:
        print("测试导入模块...")
        
        import rospy
        print("✓ rospy 导入成功")
        
        import cv2
        print("✓ cv2 导入成功")
        
        import numpy as np
        print("✓ numpy 导入成功")
        
        from collections import defaultdict
        print("✓ defaultdict 导入成功")
        
        try:
            from ultralytics import YOLO
            print("✓ ultralytics YOLO 导入成功")
        except ImportError as e:
            print(f"✗ ultralytics YOLO 导入失败: {e}")
            return False
        
        print("\n所有必要模块导入成功！")
        return True
        
    except Exception as e:
        print(f"✗ 导入失败: {e}")
        return False

def test_model_loading():
    """测试YOLO模型加载"""
    try:
        from ultralytics import YOLO
        
        # 查找模型文件
        model_paths = [
            '/home/micoair/UAVGP25/yolo11n.pt',
            '/home/micoair/yolo11n.pt', 
            'yolo11n.pt'
        ]
        
        model_path = None
        for path in model_paths:
            if os.path.exists(path):
                model_path = path
                break
        
        if model_path:
            print(f"找到模型文件: {model_path}")
            print("测试YOLO模型加载...")
            model = YOLO(model_path)
            print("✓ YOLO模型加载成功")
            return True
        else:
            print("✗ 未找到YOLO模型文件")
            print("请确保以下位置之一存在yolo11n.pt文件:")
            for path in model_paths:
                print(f"  - {path}")
            return False
            
    except Exception as e:
        print(f"✗ YOLO模型加载失败: {e}")
        return False

def main():
    print("=== CAM TRACKER NODE 测试 ===\n")
    
    # 测试导入
    if not test_imports():
        print("\n测试失败：模块导入有问题")
        return
    
    print("\n" + "="*40)
    
    # 测试模型加载
    if not test_model_loading():
        print("\n测试失败：YOLO模型加载有问题")
        return
    
    print("\n" + "="*40)
    print("✓ 所有测试通过！cam_tracker_node应该能正常运行")
    print("\n使用方法:")
    print("1. 启动roscore")
    print("2. 启动USB摄像头节点: rosrun usb_cam usb_cam_node")
    print("3. 启动追踪节点: rosrun cam_tracker cam_tracker_node.py")

if __name__ == '__main__':
    main()