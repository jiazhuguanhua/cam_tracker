#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS节点: 使用YOLO进行ball和car检测
支持两种模式：
- control=1: 抓取区模式，只检测ball
- control=2: 释放区模式，只检测car
- control=0: 停止检测
"""

import rospy
import cv2
import os
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8, Header
from cv_bridge import CvBridge, CvBridgeError
from cam_tracker.msg import CamTrack
from ultralytics import YOLO

# 抑制OpenCV和FFmpeg警告
os.environ['OPENCV_LOG_LEVEL'] = 'ERROR'
os.environ['FFMPEG_HIDE_BANNER'] = '1'

CLASS_NAME_BALL = 'tennis-ball'
CLASS_NAME_CAR = 'car'

class CamTrackerNode:
    def __init__(self):
        """初始化Cam Tracker节点"""
        rospy.init_node('cam_tracker_node', anonymous=True)
        rospy.loginfo("=== Cam Tracker Node 启动 ===")
        
        # 控制模式: 0=停止, 1=抓取区(ball), 2=释放区(car)
        self.control_mode = 0
        
        # 加载YOLO模型
        model_path = self.find_model_path()
        rospy.loginfo(f"加载YOLO模型: {model_path}")
        try:
            self.model = YOLO(model_path)
            rospy.loginfo("✓ 模型加载成功")
        except Exception as e:
            rospy.logerr(f"✗ 模型加载失败: {e}")
            raise
        
        # 初始化CvBridge
        self.bridge = CvBridge()
        
        # 检测参数
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        rospy.loginfo(f"置信度阈值: {self.confidence_threshold}")
        
        # ROS订阅和发布
        camera_topic = rospy.get_param('~camera_topic', '/usb_cam/image_raw')
        rospy.loginfo(f"订阅摄像头话题: {camera_topic}")
        self.image_sub = rospy.Subscriber(camera_topic, Image, self.image_callback, queue_size=1)
        
        rospy.loginfo("订阅控制话题: /cam_tracker/tracker_control")
        self.control_sub = rospy.Subscriber('/cam_tracker/tracker_control', UInt8, self.control_callback, queue_size=1)
        
        rospy.loginfo("发布检测话题: /cam_tracker/info")
        self.info_pub = rospy.Publisher('/cam_tracker/info', CamTrack, queue_size=10)
        
        rospy.loginfo("=== 节点初始化完成，等待控制信号 ===")
    
    def find_model_path(self):
        """查找YOLO模型文件"""
        import rospkg
        try:
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('cam_tracker')
            
            # 尝试多个可能的模型路径
            possible_paths = [
                os.path.join(package_path, 'models', 'tennis_best.pt'),
                os.path.join(package_path, 'models', 'nuaa_brick_best.pt'),
                os.path.join(package_path, 'models', 'yolo11n.pt')
            ]
            
            for path in possible_paths:
                if os.path.exists(path):
                    return path
            
            rospy.logwarn("未找到模型文件，使用默认模型")
            return 'yolo11n.pt'
        except Exception as e:
            rospy.logwarn(f"查找模型出错: {e}")
            return 'yolo11n.pt'
    
    def control_callback(self, msg):
        """处理控制消息"""
        mode = msg.data
        if mode == self.control_mode:
            return
        
        self.control_mode = mode
        if mode == 0:
            rospy.loginfo("[控制] 停止检测")
        elif mode == 1:
            rospy.loginfo("[控制] 启动抓取区模式 - 只检测ball")
        elif mode == 2:
            rospy.loginfo("[控制] 启动释放区模式 - 只检测car")
        else:
            rospy.logwarn(f"[控制] 未知模式: {mode}")
    
    def image_callback(self, msg):
        """处理图像并进行检测"""
        # 如果是停止模式，不处理
        if self.control_mode == 0:
            return
        
        try:
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # YOLO检测
            results = self.model(
                cv_image,
                conf=self.confidence_threshold,
                verbose=False
            )
            
            # 解析结果并发布
            self.process_and_publish(results, msg.header)
            
        except CvBridgeError as e:
            rospy.logerr(f"图像转换错误: {e}")
        except Exception as e:
            rospy.logerr(f"检测错误: {e}")
    
    def process_and_publish(self, results, header):
        """处理检测结果并发布"""
        # 创建消息
        cam_track = CamTrack()
        cam_track.header = Header()
        cam_track.header.stamp = rospy.Time.now()
        cam_track.header.frame_id = "camera_link"
        cam_track.system_ok = True
        
        # 初始化所有字段
        cam_track.ball_num = 0
        cam_track.ball_x = 0.0
        cam_track.ball_y = 0.0
        cam_track.ball_dis = 0.0
        
        cam_track.car_num = 0
        cam_track.car_x = []
        cam_track.car_y = []
        cam_track.car_dis = []
        
        cam_track.in_gripper = False
        
        # 解析YOLO结果
        if results and len(results) > 0 and results[0].boxes is not None:
            boxes = results[0].boxes
            
            ball_detections = []
            car_detections = []
            
            for i in range(len(boxes)):
                xyxy = boxes.xyxy[i].cpu().numpy()
                conf = float(boxes.conf[i].cpu().numpy())
                cls_id = int(boxes.cls[i].cpu().numpy())
                class_name = self.model.names.get(cls_id, "unknown")
                
                # 计算中心点
                x1, y1, x2, y2 = xyxy
                center_x = float((x1 + x2) / 2.0)
                center_y = float((y1 + y2) / 2.0)
                
                detection = {
                    'class': class_name,
                    'x': center_x,
                    'y': center_y,
                    'conf': conf
                }
                
                if class_name == CLASS_NAME_BALL:
                    ball_detections.append(detection)
                elif class_name == CLASS_NAME_CAR:
                    car_detections.append(detection)
            
            # 根据模式填充数据
            if self.control_mode == 1:  # 抓取区 - 只检测ball
                if ball_detections:
                    # 选择置信度最高的ball
                    best_ball = max(ball_detections, key=lambda x: x['conf'])
                    cam_track.ball_num = len(ball_detections)
                    cam_track.ball_x = best_ball['x']
                    cam_track.ball_y = best_ball['y']
                    cam_track.ball_dis = 0.0
                    
                    rospy.logdebug(f"[Ball] 检测到 {len(ball_detections)} 个球，"
                                  f"最佳位置: ({best_ball['x']:.1f}, {best_ball['y']:.1f})")
            
            elif self.control_mode == 2:  # 释放区 - 只检测car
                if car_detections:
                    cam_track.car_num = len(car_detections)
                    cam_track.car_x = [d['x'] for d in car_detections]
                    cam_track.car_y = [d['y'] for d in car_detections]
                    cam_track.car_dis = [0.0] * len(car_detections)
                    
                    rospy.logdebug(f"[Car] 检测到 {len(car_detections)} 辆车")
        
        # 发布消息
        self.info_pub.publish(cam_track)


def main():
    try:
        node = CamTrackerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点正常退出")
    except Exception as e:
        rospy.logerr(f"节点异常: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
