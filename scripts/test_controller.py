#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
测试脚本：发送控制信号到cam_tracker
"""

import rospy
from std_msgs.msg import UInt8

def main():
    rospy.init_node('cam_tracker_controller', anonymous=True)
    
    pub = rospy.Publisher('/cam_tracker/tracker_control', UInt8, queue_size=1)
    
    rospy.sleep(1.0)  # 等待发布器初始化
    
    print("=" * 60)
    print("Cam Tracker 控制器")
    print("=" * 60)
    print("命令:")
    print("  0 - 停止检测")
    print("  1 - 抓取区模式（只检测ball）")
    print("  2 - 释放区模式（只检测car）")
    print("  q - 退出")
    print("=" * 60)
    
    while not rospy.is_shutdown():
        try:
            cmd = input("\n输入命令 (0/1/2/q): ").strip()
            
            if cmd == 'q':
                print("退出...")
                break
            elif cmd in ['0', '1', '2']:
                mode = int(cmd)
                msg = UInt8()
                msg.data = mode
                pub.publish(msg)
                
                if mode == 0:
                    print("✅ 已发送: 停止检测")
                elif mode == 1:
                    print("✅ 已发送: 抓取区模式（检测ball）")
                elif mode == 2:
                    print("✅ 已发送: 释放区模式（检测car）")
            else:
                print("⚠️  无效命令，请输入 0/1/2/q")
                
        except KeyboardInterrupt:
            print("\n退出...")
            break
        except Exception as e:
            print(f"错误: {e}")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
