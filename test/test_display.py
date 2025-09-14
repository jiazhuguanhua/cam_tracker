#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
测试OpenCV图形显示功能
"""

import cv2
import numpy as np

def test_opencv_display():
    """测试OpenCV图形显示"""
    try:
        # 创建一个测试图像
        img = np.zeros((400, 600, 3), dtype=np.uint8)
        
        # 绘制一些测试内容
        cv2.rectangle(img, (50, 50), (200, 150), (0, 255, 0), 2)
        cv2.circle(img, (125, 100), 5, (0, 255, 0), -1)
        cv2.putText(img, "ID: 1 person 0.85", (50, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        cv2.rectangle(img, (250, 100), (400, 200), (255, 0, 0), 2)
        cv2.circle(img, (325, 150), 5, (255, 0, 0), -1)
        cv2.putText(img, "ID: 2 car 0.92", (250, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        
        # 添加标题和信息
        cv2.putText(img, "YOLO11 + ByteTrack - Test Display", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(img, "FPS: 15", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(img, "Targets: 2", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        # 显示图像
        print("正在显示测试窗口...")
        print("如果您看到了窗口，说明图形显示正常工作")
        print("按任意键关闭窗口")
        
        cv2.imshow("Test Display", img)
        cv2.waitKey(0)  # 等待按键
        cv2.destroyAllWindows()
        
        print("图形显示测试完成！")
        return True
        
    except Exception as e:
        print(f"图形显示测试失败: {e}")
        return False

if __name__ == "__main__":
    test_opencv_display()