#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS节点: 使用YOLO11进行实时目标检测和追踪
专门追踪person类型目标，双话题输出：
- /detection/single_target: 当前追踪的person目标简化信息
- /detection/multi_target: 所有检测目标的完整信息
支持通过 /tracker_action 话题控制追踪器的启动和停止
"""

import rospy
import cv2
import numpy as np
import time
import os
from collections import defaultdict
from sensor_msgs.msg import Image
from std_msgs.msg import Header, Bool
from cv_bridge import CvBridge, CvBridgeError
from cam_tracker.msg import Detection, CompleteDetection, DetectionArray

from ultralytics import YOLO

# 设置环境变量来抑制FFmpeg警告
os.environ['FFMPEG_HIDE_BANNER'] = '1'
os.environ['AV_LOG_FORCE_NOCOLOR'] = '1'
os.environ['OPENCV_LOG_LEVEL'] = 'ERROR'

# 目标追踪配置
TARGET_CLASS = "person"  # 追踪目标类型


def setup_opencv_logging():
    """安全地设置OpenCV日志级别，兼容不同版本"""
    try:
        cv_version = cv2.__version__
        rospy.logdebug(f"OpenCV版本: {cv_version}")
        
        if hasattr(cv2, 'setLogLevel'):
            cv2.setLogLevel(2)  # ERROR级别
            rospy.logdebug("使用数值2设置OpenCV日志级别为ERROR")
        else:
            rospy.logwarn("cv2.setLogLevel方法不可用")
        
        rospy.logdebug("OpenCV日志级别配置完成")
        return True
        
    except Exception as e:
        rospy.logwarn(f"OpenCV日志级别设置失败: {e}")
        rospy.logdebug("这不会影响主要功能，但可能会看到OpenCV警告信息")
        return False


class PersonTrackerNode:
    def __init__(self):
        """初始化Person追踪节点"""
        rospy.init_node('cam_tracker_node', anonymous=True)
        rospy.loginfo("初始化Person追踪节点...")
        
        # 追踪状态
        self.tracker_enabled = False
        self.tracked_person_id = None  # 当前追踪的person ID
        self.last_seen_frame = 0  # 最后看到目标的帧数
        self.current_frame = 0
        self.max_missing_frames = 30  # 最大丢失帧数，超过则切换目标
        
        # 初始化OpenCV日志
        setup_opencv_logging()
        
        # 初始化YOLO模型
        model_path = rospy.get_param('~model_path', '/home/micoair/drone_vision_ws/yolo11n.pt')
        rospy.loginfo(f"加载YOLO模型: {model_path}")
        
        try:
            self.model = YOLO(model_path)
            rospy.loginfo("YOLO模型加载成功")
        except Exception as e:
            rospy.logerr(f"YOLO模型加载失败: {e}")
            return
        
        # 初始化CvBridge
        self.bridge = CvBridge()
        
        # 追踪器参数
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.iou_threshold = rospy.get_param('~iou_threshold', 0.45)
        self.max_det = rospy.get_param('~max_det', 300)
        self.tracker_type = rospy.get_param('~tracker_type', 'bytetrack.yaml')
        
        rospy.loginfo(f"追踪参数: conf={self.confidence_threshold}, iou={self.iou_threshold}")
        rospy.loginfo(f"目标追踪类型: {TARGET_CLASS}")
        
        # ROS话题初始化
        rospy.loginfo("初始化ROS话题...")
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback, queue_size=1)
        
        # 双话题发布者
        self.single_target_pub = rospy.Publisher('/detection/single_target', Detection, queue_size=10)
        self.multi_target_pub = rospy.Publisher('/detection/multi_target', DetectionArray, queue_size=10)
        
        # 追踪器控制话题
        self.action_sub = rospy.Subscriber('/tracker_action', Bool, self.tracker_action_callback, queue_size=1)
        
        rospy.loginfo("话题配置完成:")
        rospy.loginfo("  订阅: /usb_cam/image_raw")
        rospy.loginfo("  订阅: /tracker_action")
        rospy.loginfo("  发布: /detection/single_target (简化person信息)")
        rospy.loginfo("  发布: /detection/multi_target (所有目标完整信息)")
        
        rospy.loginfo("Person追踪器待机中，发送控制信号启动")

    def tracker_action_callback(self, msg):
        """处理追踪器控制消息"""
        action = msg.data
        
        if action and not self.tracker_enabled:
            # 启动追踪器
            self.tracker_enabled = True
            self.tracked_person_id = None  # 重置追踪目标
            rospy.loginfo(f"追踪器已启动，开始寻找{TARGET_CLASS}目标")
            
        elif not action and self.tracker_enabled:
            # 停止追踪器
            self.tracker_enabled = False
            self.tracked_person_id = None
            rospy.loginfo("追踪器已停止")
            
        elif action and self.tracker_enabled:
            rospy.logdebug("追踪器已经在运行中")
        elif not action and not self.tracker_enabled:
            rospy.logdebug("追踪器已经是停止状态")

    def image_callback(self, msg):
        """处理图像回调"""
        if not self.tracker_enabled:
            return
            
        try:
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width = cv_image.shape[:2]
            self.current_frame += 1
            
            # YOLO检测和追踪
            start_time = time.time()
            results = self.model.track(
                cv_image,
                conf=self.confidence_threshold,
                iou=self.iou_threshold,
                max_det=self.max_det,
                tracker=self.tracker_type,
                persist=True,
                verbose=False
            )
            processing_time = time.time() - start_time
            
            # 解析检测结果
            tracked_objects = self.parse_detection_results(results)
            
            # 发布消息
            self.publish_detections(tracked_objects, msg.header, width, height, processing_time)
            
        except CvBridgeError as e:
            rospy.logerr(f"图像转换错误: {e}")
        except Exception as e:
            rospy.logerr(f"图像处理错误: {e}")

    def parse_detection_results(self, results):
        """解析YOLO检测结果"""
        tracked_objects = []
        
        if results and len(results) > 0 and results[0].boxes is not None:
            boxes = results[0].boxes
            
            for i in range(len(boxes)):
                # 获取边界框坐标
                xyxy = boxes.xyxy[i].cpu().numpy()
                conf = float(boxes.conf[i].cpu().numpy())
                cls_id = int(boxes.cls[i].cpu().numpy())
                
                # 获取类别名称
                class_name = self.model.names.get(cls_id, f"class_{cls_id}")
                
                # 获取追踪ID
                track_id = None
                if hasattr(boxes, 'id') and boxes.id is not None and i < len(boxes.id):
                    track_id = int(boxes.id[i].cpu().numpy())
                else:
                    track_id = i  # 如果没有追踪ID，使用索引
                
                # 计算中心点
                x1, y1, x2, y2 = xyxy
                center_x = (x1 + x2) / 2.0
                center_y = (y1 + y2) / 2.0
                
                obj = {
                    'track_id': track_id,
                    'class_name': class_name,
                    'confidence': conf,
                    'bbox': [float(x1), float(y1), float(x2), float(y2)],
                    'center_x': float(center_x),
                    'center_y': float(center_y),
                    'width': float(x2 - x1),
                    'height': float(y2 - y1)
                }
                
                tracked_objects.append(obj)
                
        return tracked_objects

    def select_target_person(self, tracked_objects):
        """选择要追踪的person目标"""
        # 过滤出person类型的目标
        person_objects = [obj for obj in tracked_objects if obj['class_name'] == TARGET_CLASS]
        
        if not person_objects:
            # 如果没有检测到person，检查是否需要重置追踪
            if self.tracked_person_id is not None:
                missing_frames = self.current_frame - self.last_seen_frame
                if missing_frames > self.max_missing_frames:
                    rospy.loginfo(f"目标丢失超过{self.max_missing_frames}帧，重置追踪目标")
                    self.tracked_person_id = None
            return None
        
        # 如果当前有追踪目标，检查是否还存在
        if self.tracked_person_id is not None:
            for person in person_objects:
                if person['track_id'] == self.tracked_person_id:
                    self.last_seen_frame = self.current_frame
                    return person
            
            # 当前追踪目标丢失，选择置信度最高的新目标
            rospy.loginfo(f"当前追踪目标(ID:{self.tracked_person_id})丢失，选择新目标")
        
        # 选择置信度最高的person作为新的追踪目标
        best_person = max(person_objects, key=lambda x: x['confidence'])
        self.tracked_person_id = best_person['track_id']
        self.last_seen_frame = self.current_frame
        
        rospy.loginfo(f"开始追踪新的{TARGET_CLASS}目标: ID={self.tracked_person_id}, "
                      f"置信度={best_person['confidence']:.3f}")
        
        return best_person

    def publish_detections(self, tracked_objects, header, width, height, processing_time):
        """发布检测结果到两个话题"""
        try:
            # 1. 发布所有目标的完整信息到 /detection/multi_target
            self.publish_multi_target(tracked_objects, header, width, height, processing_time)
            
            # 2. 发布追踪的person目标到 /detection/single_target
            target_person = self.select_target_person(tracked_objects)
            if target_person:
                self.publish_single_target(target_person)
                
        except Exception as e:
            rospy.logerr(f"发布检测结果时发生错误: {e}")

    def publish_multi_target(self, tracked_objects, header, width, height, processing_time):
        """发布所有目标的完整信息"""
        detection_array = DetectionArray()
        detection_array.header = header
        detection_array.header.frame_id = "camera"
        detection_array.image_width = width
        detection_array.image_height = height
        detection_array.total_objects = len(tracked_objects)
        detection_array.processing_time = processing_time
        
        for obj in tracked_objects:
            detection = CompleteDetection()
            detection.header = header
            detection.id = obj['track_id']
            detection.class_name = obj['class_name']
            detection.confidence = obj['confidence']
            detection.xyxy = obj['bbox']
            detection.center_x = obj['center_x']
            detection.center_y = obj['center_y']
            detection.width = obj['width']
            detection.height = obj['height']
            
            detection_array.detections.append(detection)
        
        self.multi_target_pub.publish(detection_array)
        rospy.logdebug(f"发布多目标信息: {len(tracked_objects)}个目标")

    def publish_single_target(self, target_person):
        """发布当前追踪的person目标简化信息"""
        detection = Detection()
        detection.detection_id = target_person['track_id']
        detection.detection_x = target_person['center_x']
        detection.detection_y = target_person['center_y']
        
        self.single_target_pub.publish(detection)
        rospy.logdebug(f"发布追踪目标: ID={detection.detection_id}, "
                      f"位置=({detection.detection_x:.1f},{detection.detection_y:.1f})")


def main():
    try:
        rospy.loginfo("启动追踪节点...")
        
        # 创建节点实例
        node_start_time = time.time()
        node = PersonTrackerNode()
        node_init_time = time.time() - node_start_time
        
        rospy.loginfo(f"节点初始化完成，耗时: {node_init_time:.2f}秒")
        rospy.loginfo("开始监听图像话题，进入主循环...")
        
        # 进入ROS主循环
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("键盘中断")
        
    except rospy.ROSInterruptException:
        rospy.loginfo("收到ROS中断信号，节点正常退出")
    except rospy.ROSException as e:
        rospy.logerr(f"ROS异常: {e}")
    except Exception as e:
        rospy.logerr(f"节点启动失败: {e}")
        import traceback
        rospy.logdebug(f"错误详情:\n{traceback.format_exc()}")
    finally:
        rospy.loginfo("追踪节点已停止")


if __name__ == '__main__':
    main()