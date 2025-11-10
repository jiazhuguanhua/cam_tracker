#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS节点: 使用YOLO+ByteTracker进行ball和car检测与追踪
支持两种模式：
- control=1: 抓取区模式，只检测ball（带追踪，保持ID一致性）
- control=2: 释放区模式，只检测car（带追踪）
- control=0: 停止检测
"""

import rospy
import cv2
import os
import numpy as np
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
        rospy.loginfo("=== Cam Tracker Node with ByteTracker ===")
        
        # 控制模式: 0=停止, 1=抓取区(ball), 2=释放区(car)
        self.control_mode = 0
        
        # 加载YOLO模型
        model_path = self.find_model_path()
        rospy.loginfo(f"Loading YOLO model: {model_path}")
        try:
            self.model = YOLO(model_path)
            rospy.loginfo("✓ Model loaded successfully")
        except Exception as e:
            rospy.logerr(f"✗ Model loading failed: {e}")
            raise
        
        # 初始化CvBridge
        self.bridge = CvBridge()
        
        # 检测参数
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        rospy.loginfo(f"Confidence threshold: {self.confidence_threshold}")
        
        # 追踪参数
        self.tracker_type = rospy.get_param('~tracker_type', 'bytetrack')  # 'bytetrack' or 'botsort'
        rospy.loginfo(f"Tracker type: {self.tracker_type}")
        
        # Ball追踪状态
        self.current_ball_id = None  # 当前追踪的ball ID
        self.last_ball_position = None  # 上一次ball的位置 (x, y)
        self.ball_id_lost_frames = 0  # ID丢失的帧数
        self.max_lost_frames = 10  # 最大允许丢失帧数
        
        # ROS订阅和发布
        camera_topic = rospy.get_param('~camera_topic', '/usb_cam/image_raw')
        rospy.loginfo(f"Subscribe to camera: {camera_topic}")
        self.image_sub = rospy.Subscriber(camera_topic, Image, self.image_callback, queue_size=1)
        
        rospy.loginfo("Subscribe to control: /cam_tracker/tracker_control")
        self.control_sub = rospy.Subscriber('/cam_tracker/tracker_control', UInt8, self.control_callback, queue_size=1)
        
        rospy.loginfo("Publish to: /cam_tracker/info")
        self.info_pub = rospy.Publisher('/cam_tracker/info', CamTrack, queue_size=10)
        
        rospy.loginfo("=== Node initialized, waiting for control signal ===")
    
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
            
            rospy.logwarn("Model file not found, using default model")
            return 'yolo11n.pt'
        except Exception as e:
            rospy.logwarn(f"Error finding model: {e}")
            return 'yolo11n.pt'
    
    def control_callback(self, msg):
        """处理控制消息"""
        mode = msg.data
        if mode == self.control_mode:
            return
        
        old_mode = self.control_mode
        self.control_mode = mode
        
        # 模式切换时重置追踪状态
        if old_mode != mode:
            self.reset_tracking()
        
        if mode == 0:
            rospy.loginfo("[Control] Stop detection")
        elif mode == 1:
            rospy.loginfo("[Control] Pickup mode - Track ball with ByteTracker")
        elif mode == 2:
            rospy.loginfo("[Control] Release mode - Track car with ByteTracker")
        else:
            rospy.logwarn(f"[Control] Unknown mode: {mode}")
    
    def reset_tracking(self):
        """重置追踪状态"""
        self.current_ball_id = None
        self.last_ball_position = None
        self.ball_id_lost_frames = 0
        rospy.logdebug("Tracking state reset")
    
    def image_callback(self, msg):
        """处理图像并进行检测+追踪"""
        # 如果是停止模式，不处理
        if self.control_mode == 0:
            return
        
        try:
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # YOLO检测 + 追踪
            # persist=True 保持追踪器状态在帧之间
            results = self.model.track(
                cv_image,
                conf=self.confidence_threshold,
                tracker=self.tracker_type,
                persist=True,
                verbose=False
            )
            
            # 解析结果并发布
            self.process_and_publish(results, msg.header)
            
        except CvBridgeError as e:
            rospy.logerr(f"Image conversion error: {e}")
        except Exception as e:
            rospy.logerr(f"Detection error: {e}")
            import traceback
            traceback.print_exc()
    
    def calculate_distance(self, pos1, pos2):
        """计算两个位置之间的欧氏距离"""
        if pos1 is None or pos2 is None:
            return float('inf')
        return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    def select_best_ball(self, ball_detections):
        """
        选择最佳ball进行追踪
        策略：
        1. 如果当前有追踪ID，优先选择相同ID的ball
        2. 如果当前ID丢失，选择距离上次位置最近的ball作为新目标
        3. 如果没有历史信息，选择置信度最高的ball
        """
        if not ball_detections:
            return None
        
        # 情况1: 有当前追踪ID，查找相同ID的ball
        if self.current_ball_id is not None:
            for ball in ball_detections:
                if ball['id'] == self.current_ball_id:
                    # 找到相同ID，重置丢失计数
                    self.ball_id_lost_frames = 0
                    self.last_ball_position = (ball['x'], ball['y'])
                    rospy.logdebug(f"[Ball Tracking] Continue tracking ID={self.current_ball_id}")
                    return ball
            
            # ID未找到，增加丢失计数
            self.ball_id_lost_frames += 1
            rospy.logdebug(f"[Ball Tracking] ID={self.current_ball_id} lost for {self.ball_id_lost_frames} frames")
        
        # 情况2: ID丢失或首次检测，根据位置或置信度选择
        if self.last_ball_position is not None and self.ball_id_lost_frames < self.max_lost_frames:
            # 选择距离最近的ball（ID切换策略）
            closest_ball = min(ball_detections, 
                             key=lambda b: self.calculate_distance(
                                 self.last_ball_position, (b['x'], b['y'])
                             ))
            
            distance = self.calculate_distance(
                self.last_ball_position, (closest_ball['x'], closest_ball['y'])
            )
            
            # 如果距离合理（小于阈值），认为是同一个球
            if distance < 100:  # 像素距离阈值
                old_id = self.current_ball_id
                self.current_ball_id = closest_ball['id']
                self.last_ball_position = (closest_ball['x'], closest_ball['y'])
                self.ball_id_lost_frames = 0
                rospy.loginfo(f"[Ball Tracking] ID switch: {old_id} -> {self.current_ball_id} (distance={distance:.1f})")
                return closest_ball
        
        # 情况3: 完全丢失或首次检测，选择置信度最高的ball
        best_ball = max(ball_detections, key=lambda x: x['conf'])
        old_id = self.current_ball_id
        self.current_ball_id = best_ball['id']
        self.last_ball_position = (best_ball['x'], best_ball['y'])
        self.ball_id_lost_frames = 0
        
        if old_id is None:
            rospy.loginfo(f"[Ball Tracking] Start tracking ID={self.current_ball_id}")
        else:
            rospy.loginfo(f"[Ball Tracking] Lost too long, new target ID={self.current_ball_id}")
        
        return best_ball
    
    def process_and_publish(self, results, header):
        """处理检测+追踪结果并发布"""
        # 创建消息
        cam_track = CamTrack()
        cam_track.header = Header()
        cam_track.header.stamp = rospy.Time.now()
        cam_track.header.frame_id = "camera_link"
        cam_track.system_ok = True
        
        # 初始化所有字段
        cam_track.ball_num = 0
        cam_track.ball_id = -1
        cam_track.ball_x = 0.0
        cam_track.ball_y = 0.0
        cam_track.ball_dis = 0.0
        
        cam_track.car_num = 0
        cam_track.car_ids = []
        cam_track.car_x = []
        cam_track.car_y = []
        cam_track.car_dis = []
        
        cam_track.in_gripper = False
        
        # 解析YOLO+追踪结果
        if results and len(results) > 0 and results[0].boxes is not None:
            boxes = results[0].boxes
            
            # 检查是否有追踪ID
            has_tracking = boxes.id is not None
            
            ball_detections = []
            car_detections = []
            
            for i in range(len(boxes)):
                xyxy = boxes.xyxy[i].cpu().numpy()
                conf = float(boxes.conf[i].cpu().numpy())
                cls_id = int(boxes.cls[i].cpu().numpy())
                class_name = self.model.names.get(cls_id, "unknown")
                
                # 获取追踪ID
                track_id = int(boxes.id[i].cpu().numpy()) if has_tracking else -1
                
                # 计算中心点
                x1, y1, x2, y2 = xyxy
                center_x = float((x1 + x2) / 2.0)
                center_y = float((y1 + y2) / 2.0)
                
                detection = {
                    'class': class_name,
                    'id': track_id,
                    'x': center_x,
                    'y': center_y,
                    'conf': conf
                }
                
                if class_name == CLASS_NAME_BALL:
                    ball_detections.append(detection)
                elif class_name == CLASS_NAME_CAR:
                    car_detections.append(detection)
            
            # 根据模式填充数据
            if self.control_mode == 1:  # 抓取区 - 追踪ball
                if ball_detections:
                    # 使用智能选择策略
                    best_ball = self.select_best_ball(ball_detections)
                    
                    if best_ball:
                        cam_track.ball_num = len(ball_detections)
                        cam_track.ball_id = best_ball['id']
                        cam_track.ball_x = best_ball['x']
                        cam_track.ball_y = best_ball['y']
                        cam_track.ball_dis = 0.0
                        
                        rospy.logdebug(f"[Ball] Detected {len(ball_detections)} balls, "
                                      f"Tracking ID={best_ball['id']}, "
                                      f"Position=({best_ball['x']:.1f}, {best_ball['y']:.1f}), "
                                      f"Conf={best_ball['conf']:.2f}")
                else:
                    # 没有检测到ball
                    if self.current_ball_id is not None:
                        self.ball_id_lost_frames += 1
                        if self.ball_id_lost_frames >= self.max_lost_frames:
                            rospy.logwarn(f"[Ball Tracking] Lost ID={self.current_ball_id} for too long, reset")
                            self.reset_tracking()
            
            elif self.control_mode == 2:  # 释放区 - 追踪car
                if car_detections:
                    # 按ID排序以保持一致性
                    car_detections.sort(key=lambda x: x['id'])
                    
                    cam_track.car_num = len(car_detections)
                    cam_track.car_ids = [d['id'] for d in car_detections]
                    cam_track.car_x = [d['x'] for d in car_detections]
                    cam_track.car_y = [d['y'] for d in car_detections]
                    cam_track.car_dis = [0.0] * len(car_detections)
                    
                    rospy.logdebug(f"[Car] Detected {len(car_detections)} cars, "
                                  f"IDs={cam_track.car_ids}")
        
        # 发布消息
        self.info_pub.publish(cam_track)


def main():
    try:
        node = CamTrackerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node shutdown normally")
    except Exception as e:
        rospy.logerr(f"Node exception: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
