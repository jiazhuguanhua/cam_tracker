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
TARGET_CLASS = "red_brick"  # 追踪目标类型


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
        rospy.loginfo("初始化追踪节点...")
        
        # 追踪状态
        self.tracker_enabled = False
        self.tracked_person_id = None  # 当前追踪的person ID
        self.last_seen_frame = 0  # 最后看到目标的帧数
        self.current_frame = 0
        self.max_missing_frames = 30  # 最大丢失帧数，超过则切换目标
        
        # 最近的目标数据存储（用于没有目标时输出）
        self.last_target_data = None
        
        # 输出频率控制
        self.min_publish_rate = 20.0  # 最小输出频率20Hz
        self.last_publish_time = 0.0
        
        # 初始化OpenCV日志
        setup_opencv_logging()
        
        # 初始化YOLO模型
        # 自动查找模型文件路径，优先级：ROS参数 > 包内models目录 > 默认下载
        default_model_path = self.find_model_path()
        model_path = rospy.get_param('~model_path', default_model_path)
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
        self.single_target_pub = rospy.Publisher('/detection/data', Detection, queue_size=10)
        self.multi_target_pub = rospy.Publisher('/detection/multi_target', DetectionArray, queue_size=10)
        
        # 追踪器控制话题
        self.action_sub = rospy.Subscriber('/tracker_action', Bool, self.tracker_action_callback, queue_size=1)
        
        # 定时器确保最小输出频率
        self.publish_timer = rospy.Timer(rospy.Duration(1.0/self.min_publish_rate), self.timer_publish_callback)
        
        rospy.loginfo("话题配置完成:")
        rospy.loginfo("  订阅: /usb_cam/image_raw")
        rospy.loginfo("  订阅: /tracker_action")
        rospy.loginfo("  发布: /detection/data (简化person信息)")
        rospy.loginfo("  发布: /detection/multi_target (所有目标完整信息)")
        rospy.loginfo(f"  输出频率: >= {self.min_publish_rate}Hz")
        
        rospy.loginfo("追踪器待机中，发送控制信号启动")

    def find_model_path(self):
        """自动查找YOLO模型文件路径"""
        import rospkg
        
        try:
            # 获取当前包的路径
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('cam_tracker')
            
            # 定义可能的模型路径（按优先级排序）
            possible_paths = [
                os.path.join(package_path, 'models', 'nuaa_brick_best.pt'),
                os.path.join(package_path, 'models', 'yolo11n.pt'),
                os.path.join(package_path, 'yolo11n.pt'),
                os.path.expanduser('~/models/yolo11n.pt'),
                os.path.expanduser('~/yolo11n.pt'),
                'yolo11n.pt'  # 这会让ultralytics自动下载
            ]
            
            # 检查文件是否存在
            for path in possible_paths:
                if os.path.exists(path):
                    rospy.loginfo(f"找到YOLO模型文件: {path}")
                    return path
            
            # 如果没有找到现有文件，返回默认路径（ultralytics会自动下载）
            rospy.logwarn("未找到本地YOLO模型文件，将使用默认模型（自动下载）")
            return 'yolo11n.pt'
            
        except Exception as e:
            rospy.logwarn(f"查找模型路径时出错: {e}")
            return 'yolo11n.pt'

    def tracker_action_callback(self, msg):
        """处理追踪器控制消息"""
        action = msg.data
        
        if action and not self.tracker_enabled:
            # 启动追踪器
            self.tracker_enabled = True
            self.tracked_person_id = None  # 重置追踪目标
            self.last_publish_time = time.time()  # 重置发布时间
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

    def timer_publish_callback(self, event):
        """定时器回调，确保最小发布频率"""
        if not self.tracker_enabled:
            return
            
        current_time = time.time()
        time_since_last_publish = current_time - self.last_publish_time
        min_interval = 1.0 / self.min_publish_rate
        
        # 如果距离上次发布时间已经超过最小间隔，强制发布一次
        if time_since_last_publish >= min_interval:
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "camera"
            
            # 发布最近的目标数据（tracker_id=-1表示没有当前目标）
            self.publish_single_target(None, header)
            self.last_publish_time = current_time

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
            self.publish_single_target(target_person, header)
            
            # 更新发布时间
            self.last_publish_time = time.time()
                
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

    def publish_single_target(self, target_person, header=None):
        """发布当前追踪的person目标简化信息"""
        detection = Detection()
        
        if target_person is not None:
            # 有目标时，使用正常的tracker_id和位置
            detection.detection_id = 1
            detection.detection_x = target_person['center_x']
            detection.detection_y = target_person['center_y']
            
            # 保存最新的目标数据
            self.last_target_data = {
                'center_x': target_person['center_x'],
                'center_y': target_person['center_y']
            }
            
            rospy.logdebug(f"发布追踪目标: ID={detection.detection_id}, "
                          f"位置=({detection.detection_x:.1f},{detection.detection_y:.1f})")
        else:
            # 没有目标时，使用tracker_id=0，但输出最近的位置数据
            detection.detection_id = 0
            
            if self.last_target_data is not None:
                detection.detection_x = self.last_target_data['center_x']
                detection.detection_y = self.last_target_data['center_y']
                rospy.logdebug(f"发布最近目标数据: ID=-1, "
                              f"位置=({detection.detection_x:.1f},{detection.detection_y:.1f})")
            else:
                # 如果没有历史数据，输出默认值
                detection.detection_x = 0.0
                detection.detection_y = 0.0
                rospy.logdebug("发布默认目标数据: ID=-1, 位置=(0.0,0.0)")
        
        self.single_target_pub.publish(detection)


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