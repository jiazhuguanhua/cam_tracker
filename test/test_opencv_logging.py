#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
OpenCV日志级别兼容性测试脚本
"""

import cv2

def test_opencv_logging():
    """测试OpenCV日志设置的兼容性"""
    print("=== OpenCV日志级别兼容性测试 ===")
    print(f"OpenCV版本: {cv2.__version__}")
    print()
    
    # 测试可用的属性
    print("可用的OpenCV属性:")
    
    # 检查setLogLevel方法
    if hasattr(cv2, 'setLogLevel'):
        print("✓ cv2.setLogLevel 可用")
        
        # 检查日志级别常量
        log_levels = [
            'LOG_LEVEL_SILENT',
            'LOG_LEVEL_FATAL', 
            'LOG_LEVEL_ERROR',
            'LOG_LEVEL_WARNING',
            'LOG_LEVEL_INFO',
            'LOG_LEVEL_DEBUG',
            'LOG_LEVEL_VERBOSE'
        ]
        
        print("  可用的日志级别常量:")
        for level in log_levels:
            if hasattr(cv2, level):
                value = getattr(cv2, level)
                print(f"  ✓ cv2.{level} = {value}")
            else:
                print(f"  ✗ cv2.{level} 不可用")
    else:
        print("✗ cv2.setLogLevel 不可用")
    
    # 检查utils.logging
    if hasattr(cv2, 'utils'):
        print("✓ cv2.utils 可用")
        if hasattr(cv2.utils, 'logging'):
            print("✓ cv2.utils.logging 可用")
        else:
            print("✗ cv2.utils.logging 不可用")
    else:
        print("✗ cv2.utils 不可用")
    
    print()
    print("=== 测试日志级别设置 ===")
    
    # 尝试设置日志级别
    methods = [
        ("数值方法 (ERROR=3)", lambda: cv2.setLogLevel(3)),
        ("常量方法", lambda: cv2.setLogLevel(cv2.LOG_LEVEL_ERROR) if hasattr(cv2, 'LOG_LEVEL_ERROR') else None),
        ("utils方法", lambda: cv2.utils.logging.setLogLevel("ERROR") if hasattr(cv2, 'utils') and hasattr(cv2.utils, 'logging') else None)
    ]
    
    for name, method in methods:
        try:
            if method() is not None:
                print(f"✓ {name}: 设置成功")
            else:
                print(f"⚠ {name}: 方法不可用")
        except Exception as e:
            print(f"✗ {name}: 设置失败 - {e}")
    
    print()
    print("推荐的设置方法:")
    if hasattr(cv2, 'setLogLevel'):
        if hasattr(cv2, 'LOG_LEVEL_ERROR'):
            print("cv2.setLogLevel(cv2.LOG_LEVEL_ERROR)")
        else:
            print("cv2.setLogLevel(3)  # 3 代表 ERROR 级别")
    elif hasattr(cv2, 'utils') and hasattr(cv2.utils, 'logging'):
        print('cv2.utils.logging.setLogLevel("ERROR")')
    else:
        print("该OpenCV版本可能不支持日志级别设置")

if __name__ == "__main__":
    test_opencv_logging()