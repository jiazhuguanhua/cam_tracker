#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Ball Tracker Node - 无人机小球抓取视觉追踪节点
========================================

功能说明：
1. 接收启动信号后，拍摄一张照片进行目标检测
2. 自动选择图像边缘位置的目标（便于无人机抓取）
3. 持续追踪选定目标，发布位置信息
4. 接收停止信号后优雅关闭（保持模型加载在GPU中）

话题接口：
- 订阅: /camera/image_raw (图像输入)
- 订阅: /tracker_control (启动/停止控制)
- 发布: /detection/target (目标位置)
- 发布: /detection/all_targets (所有检测目标)
- 发布: /tracker_status (追踪器状态)

作者: ROS Developer
日期: 2025-10-31
版本: 2.0.0
"""

import rospy
import cv2
import numpy as np
import time
import os
from typing import Optional, List, Dict
from enum import Enum

from sensor_msgs.msg import Image
from std_msgs.msg import Header, String, Bool
from cv_bridge import CvBridge, CvBridgeError
from cam_tracker.msg import Detection, CompleteDetection, DetectionArray

from ultralytics import YOLO

# ==================== 配置参数 ====================

# 目标类型配置

TARGET_CLASS = "tennis-ball"  # 追踪目标类型 red_brick tennis-ball
MODEL_NAME = "tennis_best.pt"

# 边缘检测参数
EDGE_MARGIN_RATIO = 0.25  # 边缘区域占图像的比例 (0.25 = 图像四周25%区域)

# 追踪参数
MAX_MISSING_FRAMES = 30  # 目标丢失超过此帧数后重新选择
MIN_PUBLISH_RATE = 20.0  # 最小发布频率 (Hz)

# 夹爪区域参数
GRIPPER_ZONE_Y_RATIO = 0.6  # 分界线Y坐标占图像高度的比例（0.6表示图像60%位置的水平线）

# 环境变量设置
os.environ['FFMPEG_HIDE_BANNER'] = '1'
os.environ['AV_LOG_FORCE_NOCOLOR'] = '1'
os.environ['OPENCV_LOG_LEVEL'] = 'ERROR'


# ==================== 枚举类 ====================

class TrackerState(Enum):
    """追踪器状态"""
    IDLE = "idle"  # 空闲状态
    INITIALIZING = "initializing"  # 初始化拍照识别
    TRACKING = "tracking"  # 正在追踪
    TARGET_LOST = "target_lost"  # 目标丢失
    STOPPED = "stopped"  # 已停止


class TrackerPhase(Enum):
    """追踪阶段（用于Action反馈）"""
    WAITING = "waiting"  # 等待启动
    SNAPSHOT = "snapshot"  # 拍照识别
    TARGET_SELECTED = "target_selected"  # 目标已选择
    TRACKING_ACTIVE = "tracking_active"  # 追踪进行中
    COMPLETED = "completed"  # 完成


# ==================== 工具函数 ====================

def setup_opencv_logging():
    """设置OpenCV日志级别"""
    try:
        if hasattr(cv2, 'setLogLevel'):
            cv2.setLogLevel(2)  # ERROR级别
        return True
    except Exception as e:
        rospy.logwarn(f"OpenCV日志设置失败: {e}")
        return False


def is_on_edge(bbox: List[float], img_width: int, img_height: int, margin_ratio: float = EDGE_MARGIN_RATIO) -> bool:
    """
    判断目标是否在图像边缘区域
    
    Args:
        bbox: [x1, y1, x2, y2] 边界框坐标
        img_width: 图像宽度
        img_height: 图像高度
        margin_ratio: 边缘区域占比
        
    Returns:
        是否在边缘
    """
    x1, y1, x2, y2 = bbox
    center_x = (x1 + x2) / 2.0
    center_y = (y1 + y2) / 2.0
    
    # 计算边缘区域阈值
    margin_x = img_width * margin_ratio
    margin_y = img_height * margin_ratio
    
    # 判断中心点是否在边缘区域
    on_left = center_x < margin_x
    on_right = center_x > (img_width - margin_x)
    on_top = center_y < margin_y
    on_bottom = center_y > (img_height - margin_y)
    
    return on_left or on_right or on_top or on_bottom


def calculate_edge_distance(bbox: List[float], img_width: int, img_height: int) -> float:
    """
    计算目标中心到最近边缘的距离
    
    Args:
        bbox: [x1, y1, x2, y2] 边界框坐标
        img_width: 图像宽度
        img_height: 图像高度
        
    Returns:
        到边缘的最小距离（像素）
    """
    x1, y1, x2, y2 = bbox
    center_x = (x1 + x2) / 2.0
    center_y = (y1 + y2) / 2.0
    
    # 计算到四个边的距离
    dist_left = center_x
    dist_right = img_width - center_x
    dist_top = center_y
    dist_bottom = img_height - center_y
    
    # 返回最小距离
    return min(dist_left, dist_right, dist_top, dist_bottom)


# ==================== 主节点类 ====================

class BallTrackerNode:
    """
    小球追踪节点
    """
    
    def __init__(self):
        """初始化节点"""
        rospy.init_node('ball_tracker_node', anonymous=True)
        
        rospy.loginfo("=" * 70)
        rospy.loginfo("🎾 小球追踪节点启动中...")
        rospy.loginfo("=" * 70)
        
        # ========== 状态变量 ==========
        self.state = TrackerState.IDLE
        self.phase = TrackerPhase.WAITING
        self.tracked_target_id = None
        self.last_seen_frame = 0
        self.current_frame = 0
        self.last_target_data = None
        self.last_publish_time = 0.0
        
        # 图像相关
        self.latest_image = None
        self.image_width = 0
        self.image_height = 0
        
        # ========== ROS参数 ==========
        self.camera_topic = rospy.get_param('~camera_topic', '/usb_cam/image_raw')
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.iou_threshold = rospy.get_param('~iou_threshold', 0.45)
        self.max_det = rospy.get_param('~max_det', 100)
        self.tracker_type = rospy.get_param('~tracker_type', 'bytetrack.yaml')
        self.edge_margin_ratio = rospy.get_param('~edge_margin_ratio', EDGE_MARGIN_RATIO)
        self.min_publish_rate = rospy.get_param('~min_publish_rate', MIN_PUBLISH_RATE)
        self.auto_start = rospy.get_param('~auto_start', False)
        self.gripper_zone_y_ratio = rospy.get_param('~gripper_zone_y_ratio', GRIPPER_ZONE_Y_RATIO)
        
        rospy.loginfo(f"📷 相机话题: {self.camera_topic}")
        rospy.loginfo(f"🎯 目标类型: {TARGET_CLASS}")
        rospy.loginfo(f"📊 置信度阈值: {self.confidence_threshold}")
        rospy.loginfo(f"🔲 边缘区域比例: {self.edge_margin_ratio}")
        rospy.loginfo(f"✂️  夹爪区域分界线: {self.gripper_zone_y_ratio * 100:.0f}%")
        
        # ========== 初始化YOLO模型 ==========
        setup_opencv_logging()
        model_path = self._find_model_path()
        
        rospy.loginfo(f"🤖 加载YOLO模型: {model_path}")
        try:
            self.model = YOLO(model_path)
            rospy.loginfo("✅ YOLO模型加载成功 (已预热GPU)")
        except Exception as e:
            rospy.logerr(f"❌ YOLO模型加载失败: {e}")
            raise
        
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
        
        self.control_sub = rospy.Subscriber(
            '/tracker_control',
            Bool,
            self._control_callback,
            queue_size=1
        )
        
        # 发布器
        self.target_pub = rospy.Publisher(
            '/detection/target',
            Detection,
            queue_size=10
        )
        
        self.all_targets_pub = rospy.Publisher(
            '/detection/all_targets',
            DetectionArray,
            queue_size=10
        )
        
        self.status_pub = rospy.Publisher(
            '/tracker_status',
            String,
            queue_size=10
        )
        
        # 定时器 - 确保最小发布频率
        self.publish_timer = rospy.Timer(
            rospy.Duration(1.0 / self.min_publish_rate),
            self._timer_callback
        )
        
        rospy.loginfo("=" * 70)
        rospy.loginfo("✅ 节点初始化完成")
        rospy.loginfo("📡 话题:")
        rospy.loginfo(f"   订阅: {self.camera_topic}")
        rospy.loginfo(f"   订阅: /tracker_control")
        rospy.loginfo(f"   发布: /detection/target")
        rospy.loginfo(f"   发布: /detection/all_targets")
        rospy.loginfo(f"   发布: /tracker_status")
        rospy.loginfo("=" * 70)
        
        if self.auto_start:
            rospy.loginfo("🚀 自动启动追踪...")
            self._start_tracking()
        else:
            rospy.loginfo("⏸️  等待启动信号...")
            self._publish_status("等待启动信号")
    
    # ==================== 私有方法 ====================
    
    def _find_model_path(self) -> str:
        """查找YOLO模型文件路径"""
        import rospkg
        
        try:
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('cam_tracker')
            
            possible_paths = [
                os.path.join(package_path, 'models', MODEL_NAME),
                os.path.join(package_path, 'models', 'yolo11n.pt'),
                os.path.expanduser(f'~/models/{MODEL_NAME}'),
                MODEL_NAME
            ]
            
            for path in possible_paths:
                if os.path.exists(path):
                    return path
            
            rospy.logwarn("未找到本地模型，将使用在线模型")
            return 'yolo11n.pt'
            
        except Exception as e:
            rospy.logwarn(f"查找模型路径失败: {e}")
            return 'yolo11n.pt'
    
    def _control_callback(self, msg: Bool):
        """
        处理控制信号
        
        Args:
            msg: True=启动, False=停止
        """
        if msg.data:
            # 启动追踪
            if self.state == TrackerState.IDLE or self.state == TrackerState.STOPPED:
                rospy.loginfo("📨 收到启动信号")
                self._start_tracking()
            else:
                rospy.logdebug("追踪器已在运行")
        else:
            # 停止追踪
            if self.state != TrackerState.IDLE and self.state != TrackerState.STOPPED:
                rospy.loginfo("📨 收到停止信号")
                self._stop_tracking()
            else:
                rospy.logdebug("追踪器已停止")
    
    def _start_tracking(self):
        """启动追踪流程"""
        rospy.loginfo("=" * 70)
        rospy.loginfo("🚀 启动追踪流程")
        rospy.loginfo("=" * 70)
        
        self.state = TrackerState.INITIALIZING
        self.phase = TrackerPhase.SNAPSHOT
        self.tracked_target_id = None
        self.last_target_data = None
        self.current_frame = 0
        self.last_publish_time = time.time()
        
        self._publish_status("初始化中: 等待拍照...")
        
        rospy.loginfo("📸 阶段1: 等待拍照并识别目标...")
    
    def _stop_tracking(self):
        """停止追踪（优雅关闭，保持模型加载）"""
        rospy.loginfo("=" * 70)
        rospy.loginfo("🛑 停止追踪")
        rospy.loginfo("=" * 70)
        
        self.state = TrackerState.STOPPED
        self.phase = TrackerPhase.COMPLETED
        self.tracked_target_id = None
        
        self._publish_status("已停止 (模型保持加载)")
        
        rospy.loginfo("✅ 追踪已停止，模型保持在GPU中")
        rospy.loginfo("💡 发送True信号可重新启动追踪")
    
    def _image_callback(self, msg: Image):
        """
        图像回调函数
        
        Args:
            msg: ROS图像消息
        """
        # 保存最新图像
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_height, self.image_width = self.latest_image.shape[:2]
        except CvBridgeError as e:
            rospy.logerr(f"图像转换错误: {e}")
            return
        
        # 根据状态处理
        if self.state == TrackerState.INITIALIZING:
            self._process_initialization()
        elif self.state == TrackerState.TRACKING:
            self._process_tracking(msg.header)
    
    def _process_initialization(self):
        """处理初始化阶段（拍照识别并选择目标）"""
        if self.latest_image is None:
            return
        
        rospy.loginfo("📸 拍摄快照并进行目标检测...")
        self._publish_status("正在拍照识别...")
        
        # 执行检测（不带追踪）
        start_time = time.time()
        results = self.model(
            self.latest_image,
            conf=self.confidence_threshold,
            iou=self.iou_threshold,
            max_det=self.max_det,
            verbose=False
        )
        detection_time = time.time() - start_time
        
        # 解析检测结果
        detected_objects = self._parse_detection_results(results, use_tracker=False)
        
        rospy.loginfo(f"🔍 检测到 {len(detected_objects)} 个目标 (耗时: {detection_time:.3f}s)")
        
        # 过滤目标类型
        target_objects = [obj for obj in detected_objects if obj['class_name'] == TARGET_CLASS]
        
        if not target_objects:
            rospy.logwarn(f"⚠️  未检测到 {TARGET_CLASS} 类型目标，等待下一帧...")
            return
        
        rospy.loginfo(f"✅ 找到 {len(target_objects)} 个 {TARGET_CLASS} 目标")
        
        # 选择边缘目标
        selected_target = self._select_edge_target(target_objects)
        
        if selected_target:
            self.tracked_target_id = selected_target['track_id']
            self.last_seen_frame = self.current_frame
            self.state = TrackerState.TRACKING
            self.phase = TrackerPhase.TARGET_SELECTED
            
            rospy.loginfo("=" * 70)
            rospy.loginfo(f"🎯 已选择目标:")
            rospy.loginfo(f"   ID: {self.tracked_target_id}")
            rospy.loginfo(f"   置信度: {selected_target['confidence']:.3f}")
            rospy.loginfo(f"   位置: ({selected_target['center_x']:.1f}, {selected_target['center_y']:.1f})")
            rospy.loginfo(f"   距离边缘: {selected_target.get('edge_distance', 0):.1f} 像素")
            rospy.loginfo("=" * 70)
            rospy.loginfo("📡 阶段2: 开始持续追踪...")
            
            self._publish_status(f"追踪目标 ID:{self.tracked_target_id}")
            self.phase = TrackerPhase.TRACKING_ACTIVE
        else:
            rospy.logwarn("⚠️  未找到合适的边缘目标，等待下一帧...")
    
    def _select_edge_target(self, targets: List[Dict]) -> Optional[Dict]:
        """
        从目标列表中选择最适合抓取的边缘目标
        
        优先级:
        1. 在边缘区域的目标
        2. 距离边缘最近的目标
        3. 置信度最高的目标
        
        Args:
            targets: 目标列表
            
        Returns:
            选中的目标，如果没有合适目标返回None
        """
        if not targets:
            return None
        
        # 计算每个目标的边缘距离
        for target in targets:
            target['edge_distance'] = calculate_edge_distance(
                target['bbox'],
                self.image_width,
                self.image_height
            )
            target['is_on_edge'] = is_on_edge(
                target['bbox'],
                self.image_width,
                self.image_height,
                self.edge_margin_ratio
            )
        
        # 优先选择在边缘区域的目标
        edge_targets = [t for t in targets if t['is_on_edge']]
        
        if edge_targets:
            # 在边缘目标中选择置信度最高的
            selected = max(edge_targets, key=lambda x: x['confidence'])
            rospy.loginfo(f"✅ 选择边缘区域目标 (置信度: {selected['confidence']:.3f})")
        else:
            # 如果没有边缘目标，选择距离边缘最近的
            selected = min(targets, key=lambda x: x['edge_distance'])
            rospy.loginfo(f"⚠️  无边缘目标，选择最靠近边缘的目标 (距离: {selected['edge_distance']:.1f}px)")
        
        return selected
    
    def _process_tracking(self, header: Header):
        """
        处理追踪阶段
        
        Args:
            header: ROS消息头
        """
        if self.latest_image is None:
            return
        
        self.current_frame += 1
        
        # 执行追踪
        start_time = time.time()
        results = self.model.track(
            self.latest_image,
            conf=self.confidence_threshold,
            iou=self.iou_threshold,
            max_det=self.max_det,
            tracker=self.tracker_type,
            persist=True,
            verbose=False
        )
        processing_time = time.time() - start_time
        
        # 解析结果
        tracked_objects = self._parse_detection_results(results, use_tracker=True)
        
        # 发布所有目标信息
        self._publish_all_targets(tracked_objects, header, processing_time)
        
        # 查找当前追踪的目标
        current_target = None
        for obj in tracked_objects:
            if obj['class_name'] == TARGET_CLASS and obj['track_id'] == self.tracked_target_id:
                current_target = obj
                self.last_seen_frame = self.current_frame
                break
        
        # 检查目标是否丢失
        if current_target is None:
            missing_frames = self.current_frame - self.last_seen_frame
            
            if missing_frames > MAX_MISSING_FRAMES:
                # 目标丢失超过阈值
                if self.state != TrackerState.TARGET_LOST:
                    rospy.logwarn(f"⚠️  目标丢失超过 {MAX_MISSING_FRAMES} 帧，进入搜索模式")
                    self.state = TrackerState.TARGET_LOST
                    self._publish_status("目标丢失 - 积极搜索中")
                
                # 发布ID为-1，表示目标丢失（但继续输出最近位置）
                self._publish_target(None, header, force_invalid=True)
                
                # 积极检测 - 尝试从当前检测到的目标中重新选择
                self._try_reacquire_target(tracked_objects)
            else:
                # 目标暂时丢失（未超过阈值），继续发布最近数据
                self._publish_target(None, header)
        else:
            # 找到目标，正常发布
            if self.state == TrackerState.TARGET_LOST:
                rospy.loginfo("✅ 目标已恢复追踪")
                self.state = TrackerState.TRACKING
                self._publish_status(f"追踪目标 ID:{self.tracked_target_id}")
            
            self._publish_target(current_target, header)
    
    def _try_reacquire_target(self, tracked_objects: List[Dict]):
        """
        尝试重新获取目标（积极检测模式）
        
        Args:
            tracked_objects: 当前检测到的所有目标
        """
        target_objects = [obj for obj in tracked_objects if obj['class_name'] == TARGET_CLASS]
        
        if target_objects:
            # 找到目标类型的物体，重新选择
            new_target = self._select_edge_target(target_objects)
            if new_target:
                old_id = self.tracked_target_id
                self.tracked_target_id = new_target['track_id']
                self.last_seen_frame = self.current_frame
                self.state = TrackerState.TRACKING
                
                rospy.loginfo("=" * 70)
                rospy.loginfo(f"🎯 重新捕获目标:")
                rospy.loginfo(f"   旧ID: {old_id} → 新ID: {self.tracked_target_id}")
                rospy.loginfo(f"   置信度: {new_target['confidence']:.3f}")
                rospy.loginfo(f"   位置: ({new_target['center_x']:.1f}, {new_target['center_y']:.1f})")
                rospy.loginfo("=" * 70)
                
                self._publish_status(f"已恢复追踪 ID:{self.tracked_target_id}")
            else:
                rospy.logdebug("未找到合适的边缘目标，继续搜索...")
        else:
            rospy.logdebug(f"未检测到 {TARGET_CLASS} 类型目标，继续搜索...")
    
    def _reselect_target(self, tracked_objects: List[Dict]):
        """
        重新选择目标
        
        Args:
            tracked_objects: 当前检测到的所有目标
        """
        rospy.loginfo("🔄 尝试重新选择目标...")
        
        target_objects = [obj for obj in tracked_objects if obj['class_name'] == TARGET_CLASS]
        
        if target_objects:
            new_target = self._select_edge_target(target_objects)
            if new_target:
                old_id = self.tracked_target_id
                self.tracked_target_id = new_target['track_id']
                self.last_seen_frame = self.current_frame
                self.state = TrackerState.TRACKING
                
                rospy.loginfo(f"✅ 重新选择目标: {old_id} → {self.tracked_target_id}")
                self._publish_status(f"重新追踪 ID:{self.tracked_target_id}")
            else:
                rospy.logwarn("⚠️  未找到合适的替代目标")
        else:
            rospy.logwarn(f"⚠️  未检测到 {TARGET_CLASS} 类型目标")
    
    def _parse_detection_results(self, results, use_tracker: bool = True) -> List[Dict]:
        """
        解析YOLO检测结果
        
        Args:
            results: YOLO检测结果
            use_tracker: 是否使用追踪ID
            
        Returns:
            目标列表
        """
        objects = []
        
        if results and len(results) > 0 and results[0].boxes is not None:
            boxes = results[0].boxes
            
            for i in range(len(boxes)):
                # 提取数据
                xyxy = boxes.xyxy[i].cpu().numpy()
                conf = float(boxes.conf[i].cpu().numpy())
                cls_id = int(boxes.cls[i].cpu().numpy())
                class_name = self.model.names.get(cls_id, f"class_{cls_id}")
                
                # 获取追踪ID
                if use_tracker and hasattr(boxes, 'id') and boxes.id is not None and i < len(boxes.id):
                    track_id = int(boxes.id[i].cpu().numpy())
                else:
                    track_id = i
                
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
                
                objects.append(obj)
        
        return objects
    
    def _publish_target(self, target: Optional[Dict], header: Optional[Header] = None, force_invalid: bool = False):
        """
        发布当前追踪目标的位置信息
        
        Args:
            target: 目标对象，None表示无目标
            header: ROS消息头
            force_invalid: 强制设置为无效目标（ID=-1），用于目标丢失情况
        """
        detection = Detection()
        
        # 计算夹爪区域分界线的Y坐标
        gripper_zone_y = self.image_height * self.gripper_zone_y_ratio
        
        if target is not None:
            # 有目标：发布当前位置
            detection.detection_id = 1  # 有效目标
            detection.detection_x = target['center_x']
            detection.detection_y = target['center_y']
            
            # 判断是否在夹取区域（中心在分界线下方）
            detection.in_gripper_zone = (target['center_y'] > gripper_zone_y)
            
            # 保存最新数据
            self.last_target_data = {
                'center_x': target['center_x'],
                'center_y': target['center_y'],
                'in_gripper_zone': detection.in_gripper_zone
            }
            
            zone_status = "✓ 在夹取区域" if detection.in_gripper_zone else "✗ 不在夹取区域"
            rospy.logdebug(
                f"📡 发布目标位置: ID=1, ({detection.detection_x:.1f}, {detection.detection_y:.1f}) {zone_status}"
            )
        else:
            # 无目标：根据force_invalid参数决定ID
            if force_invalid:
                # 目标已丢失超过阈值，使用ID=-1
                detection.detection_id = -1
                rospy.logdebug("📡 发布丢失状态: ID=-1 (积极搜索中)")
            else:
                # 目标暂时丢失，使用ID=0
                detection.detection_id = 0
                rospy.logdebug("📡 发布暂时丢失: ID=0 (使用最近数据)")
            
            if self.last_target_data:
                detection.detection_x = self.last_target_data['center_x']
                detection.detection_y = self.last_target_data['center_y']
                detection.in_gripper_zone = self.last_target_data.get('in_gripper_zone', False)
            else:
                detection.detection_x = 0.0
                detection.detection_y = 0.0
                detection.in_gripper_zone = False
        
        self.target_pub.publish(detection)
        self.last_publish_time = time.time()
    
    def _publish_all_targets(self, targets: List[Dict], header: Header, processing_time: float):
        """
        发布所有检测目标的完整信息
        
        Args:
            targets: 目标列表
            header: ROS消息头
            processing_time: 处理时间
        """
        array = DetectionArray()
        array.header = header
        array.header.frame_id = "camera"
        array.image_width = self.image_width
        array.image_height = self.image_height
        array.total_objects = len(targets)
        array.processing_time = processing_time
        
        for obj in targets:
            det = CompleteDetection()
            det.header = header
            det.id = obj['track_id']
            det.class_name = obj['class_name']
            det.confidence = obj['confidence']
            det.xyxy = obj['bbox']
            det.center_x = obj['center_x']
            det.center_y = obj['center_y']
            det.width = obj['width']
            det.height = obj['height']
            
            array.detections.append(det)
        
        self.all_targets_pub.publish(array)
        rospy.logdebug(f"📡 发布完整信息: {len(targets)} 个目标")
    
    def _publish_status(self, status: str):
        """
        发布追踪器状态
        
        Args:
            status: 状态描述
        """
        msg = String()
        msg.data = f"{self.state.value}:{self.phase.value}:{status}"
        self.status_pub.publish(msg)
    
    def _timer_callback(self, event):
        """
        定时器回调，确保最小发布频率
        
        Args:
            event: 定时器事件
        """
        if self.state != TrackerState.TRACKING:
            return
        
        current_time = time.time()
        time_since_last_publish = current_time - self.last_publish_time
        min_interval = 1.0 / self.min_publish_rate
        
        if time_since_last_publish >= min_interval:
            # 强制发布一次（使用最近的数据）
            self._publish_target(None)
    
    # ==================== 公共方法 ====================
    
    def run(self):
        """主循环"""
        rospy.loginfo("🚀 节点运行中...")
        rospy.loginfo("💡 控制方法:")
        rospy.loginfo("   rostopic pub /tracker_control std_msgs/Bool \"data: true\"  # 启动")
        rospy.loginfo("   rostopic pub /tracker_control std_msgs/Bool \"data: false\" # 停止")
        rospy.loginfo("=" * 70)
        
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("🛑 用户中断")
        finally:
            self.shutdown()
    
    def shutdown(self):
        """关闭节点"""
        rospy.loginfo("=" * 70)
        rospy.loginfo("🛑 关闭节点...")
        rospy.loginfo("=" * 70)
        
        # 停止定时器
        if hasattr(self, 'publish_timer'):
            self.publish_timer.shutdown()
        
        rospy.loginfo("✅ 节点已关闭 (模型已从GPU卸载)")


# ==================== 主函数 ====================

def main():
    """主函数"""
    try:
        node = BallTrackerNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 ROS中断")
    except Exception as e:
        rospy.logerr(f"❌ 节点启动失败: {e}")
        import traceback
        rospy.logerr(traceback.format_exc())


if __name__ == '__main__':
    main()
