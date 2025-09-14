#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS节点: 使用YOLO11进行实时目标检测和追踪
订阅USB摄像头图像，发布检测和追踪结果
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
from cam_tracker.msg import Detection, DetectionArray

from ultralytics import YOLO

# 设置环境变量来抑制FFmpeg警告
os.environ['FFMPEG_HIDE_BANNER'] = '1'
os.environ['AV_LOG_FORCE_NOCOLOR'] = '1'
os.environ['OPENCV_LOG_LEVEL'] = 'ERROR'

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


class CamTrackerNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('cam_tracker_node', anonymous=True)
        
        # 设置日志级别
        self.debug_mode = rospy.get_param('~debug_mode', False)
        if self.debug_mode:
            rospy.loginfo("启用调试模式 - 详细日志输出")
        
        rospy.loginfo("YOLO11 多目标追踪节点启动")
        
        # 安全地设置OpenCV日志级别
        setup_opencv_logging()
        
        # 参数设置
        self.model_path = rospy.get_param('~model_path', 'yolo11n.pt')
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.25)
        self.device = rospy.get_param('~device', 'cpu')
        self.show_image = rospy.get_param('~show_image', False)
        
        if self.debug_mode:
            rospy.loginfo("节点参数配置:")
            rospy.loginfo(f"  模型路径: {self.model_path}")
            rospy.loginfo(f"  置信度阈值: {self.confidence_threshold}")
            rospy.loginfo(f"  计算设备: {self.device}")
            rospy.loginfo(f"  显示图像: {self.show_image}")
        else:
            rospy.logdebug("节点参数配置:")
            rospy.logdebug(f"  模型路径: {self.model_path}")
            rospy.logdebug(f"  置信度阈值: {self.confidence_threshold}")
            rospy.logdebug(f"  计算设备: {self.device}")
            rospy.logdebug(f"  显示图像: {self.show_image}")
        
        # OpenCV bridge
        self.bridge = CvBridge()
        rospy.logdebug("CvBridge初始化完成")
        
        # 追踪器控制状态
        self.tracker_enabled = False  # 默认关闭追踪器
        self.model_loaded = False
        
        # 初始化模型
        self.model = None
        self.init_model()
        
        # 初始化追踪变量
        if self.debug_mode:
            rospy.loginfo("初始化追踪系统...")
        else:
            rospy.logdebug("初始化追踪系统...")
        
        self.frame_count = 0
        self.last_time = time.time()
        self.track_history = defaultdict(lambda: [])
        self.target_id = None
        self.first_frame = True
        self.show_other_detections = True
        self.fps_list = []
        self.total_processing_time = 0.0
        self.max_fps = 0.0
        self.min_fps = float('inf')
        
        rospy.logdebug("追踪变量初始化完成")
        rospy.logdebug(f"轨迹历史最大长度: 30点")
        rospy.logdebug(f"显示所有检测框: {self.show_other_detections}")
        
        # ROS话题初始化
        rospy.logdebug("初始化ROS话题...")
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback, queue_size=1)
        self.detection_pub = rospy.Publisher('/yolo_identify', DetectionArray, queue_size=10)
        
        # 添加追踪器控制话题
        self.action_sub = rospy.Subscriber('/tracker_action', Bool, self.tracker_action_callback, queue_size=1)
        
        rospy.logdebug("订阅话题: /usb_cam/image_raw")
        rospy.logdebug("订阅话题: /tracker_action (Bool)")
        rospy.logdebug("发布话题: /yolo_identify")
        rospy.logdebug(f"图像订阅队列大小: 1")
        rospy.logdebug(f"检测发布队列大小: 10")
        
        rospy.loginfo("追踪器待机中，发送控制信号启动")
    
    def init_model(self):
        """初始化YOLO模型"""
        rospy.loginfo("加载YOLO模型...")
        
        try:
            if not os.path.exists(self.model_path):
                rospy.logdebug(f"当前路径: {self.model_path}")
                rospy.logfatal(f"模型文件不存在: {self.model_path}")
                rospy.signal_shutdown("模型文件不存在")
                return
            
            rospy.logdebug(f"模型文件路径: {self.model_path}")
            rospy.logdebug(f"模型文件大小: {os.path.getsize(self.model_path) / (1024*1024):.1f} MB")
            
            start_time = time.time()
            self.model = YOLO(self.model_path)
            load_time = time.time() - start_time
            
            rospy.loginfo(f"模型加载完成，耗时: {load_time:.2f}秒")
            rospy.logdebug(f"模型类别数量: {len(self.model.names)}")
            rospy.logdebug(f"支持的类别: {list(self.model.names.values())}")
            
            self.model_loaded = True
            
        except Exception as e:
            rospy.logerr(f"YOLO模型加载失败: {e}")
            rospy.logfatal("无法加载YOLO模型，节点即将关闭")
            import traceback
            rospy.logerr(f"详细错误信息:\n{traceback.format_exc()}")
            rospy.signal_shutdown("YOLO模型加载失败")
            return
    
    def tracker_action_callback(self, msg):
        """处理追踪器控制命令"""
        action = msg.data
        
        if not self.model_loaded:
            rospy.logwarn("⚠️ 模型尚未加载完成，无法启动追踪器")
            return
            
        if action and not self.tracker_enabled:
            # 启动追踪器
            self.tracker_enabled = True
            self.first_frame = True
            self.frame_count = 0
            self.track_history.clear()
            self.fps_list.clear()
            self.total_processing_time = 0.0
            self.max_fps = 0.0
            self.min_fps = float('inf')
            
            rospy.loginfo("追踪器已启动")
            
        elif not action and self.tracker_enabled:
            # 停止追踪器
            self.tracker_enabled = False
            rospy.loginfo("追踪器已停止")
            
            # 发布空的检测结果
            empty_array = DetectionArray()
            empty_array.header.stamp = rospy.Time.now()
            empty_array.header.frame_id = "camera"
            empty_array.total_objects = 0
            empty_array.processing_time = 0.0
            self.detection_pub.publish(empty_array)
            
        elif action and self.tracker_enabled:
            rospy.logdebug("追踪器已经在运行中")
        elif not action and not self.tracker_enabled:
            rospy.logdebug("追踪器已经处于停止状态")
        
    def image_callback(self, msg):
        """图像回调函数"""
        try:
            # 检查追踪器是否启用
            if not self.tracker_enabled:
                rospy.logdebug("追踪器未启用，跳过图像处理")
                return
            
            # 计算FPS
            timer = cv2.getTickCount()
            
            # 第一帧特殊处理
            if self.frame_count == 0:
                rospy.logdebug("收到第一帧图像数据，开始追踪处理")
                rospy.logdebug(f"图像尺寸: {msg.width}x{msg.height}")
                rospy.logdebug(f"图像编码: {msg.encoding}")
                rospy.logdebug(f"图像步长: {msg.step}")
            
            # 将ROS图像转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rospy.logdebug(f"图像转换成功，形状: {cv_image.shape}")
            
            # 获取图像尺寸
            height, width = cv_image.shape[:2]
            
            # 进行目标检测和追踪
            start_time = time.time()
            tracked_objects = self.detect_and_track(cv_image)
            processing_time = time.time() - start_time
            self.total_processing_time += processing_time
            
            # 计算当前帧FPS
            current_fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
            self.fps_list.append(current_fps)
            
            # 更新FPS统计
            if current_fps > self.max_fps:
                self.max_fps = current_fps
            if current_fps < self.min_fps:
                self.min_fps = current_fps
            
            # 发布结果
            self.publish_detections(tracked_objects, msg.header, width, height, processing_time)
            
            # 可选：显示图像
            if self.show_image:
                self.display_image(cv_image, tracked_objects, current_fps)
                
            # 统计帧率和追踪状态
            self.frame_count += 1
            
            # 每10帧输出简要信息
            if self.frame_count % 10 == 0:
                rospy.logdebug(f"处理第{self.frame_count}帧, 当前FPS: {int(current_fps)}, "
                              f"检测目标: {len(tracked_objects)}, "
                              f"处理时间: {processing_time*1000:.1f}ms")
            
            # 每30帧输出详细统计
            if self.frame_count % 30 == 0:
                avg_fps = sum(self.fps_list[-30:]) / min(30, len(self.fps_list))
                avg_processing = self.total_processing_time / self.frame_count
                
                rospy.logdebug("="*50)
                rospy.logdebug(f"帧数统计 - 第{self.frame_count}帧")
                rospy.logdebug(f"  平均FPS: {avg_fps:.1f}")
                rospy.logdebug(f"  最高FPS: {self.max_fps:.1f}")
                rospy.logdebug(f"  最低FPS: {self.min_fps:.1f}")
                rospy.logdebug(f"  平均处理时间: {avg_processing*1000:.1f}ms")
                rospy.logdebug(f"  当前检测目标数: {len(tracked_objects)}")
                
                # 统计各类别目标数量
                class_counts = {}
                for obj in tracked_objects:
                    class_name = obj.get('class_name', 'unknown')
                    class_counts[class_name] = class_counts.get(class_name, 0) + 1
                
                if class_counts:
                    count_str = ", ".join([f"{cls}: {count}" for cls, count in class_counts.items()])
                    rospy.logdebug(f"  目标类别统计: {count_str}")
                
                rospy.logdebug("="*50)
            
        except CvBridgeError as e:
            rospy.logerr(f"图像转换错误: {e}")
            rospy.logdebug(f"图像消息详情: 宽度={msg.width}, 高度={msg.height}, 编码={msg.encoding}")
        except Exception as e:
            rospy.logerr(f"处理图像时发生错误: {e}")
            import traceback
            rospy.logerr(f"错误详情:\n{traceback.format_exc()}")
            rospy.logwarn("跳过当前帧，继续处理下一帧")
    
    def detect_and_track(self, image):
        """使用YOLO进行目标检测和追踪"""
        if self.model is None:
            rospy.logerr("YOLO模型未加载，跳过检测")
            return []
        
        try:
            # 使用YOLO的track方法进行检测和追踪 (使用bytetrack追踪器)
            inference_start = time.time()
            results = self.model.track(image, persist=True, tracker="bytetrack.yaml")
            inference_time = time.time() - inference_start
            
            rospy.logdebug(f"YOLO推理耗时: {inference_time*1000:.1f}ms")
            
            # 第一帧日志
            if self.first_frame and results[0].boxes is not None:
                boxes = results[0].boxes
                rospy.logdebug("第一帧检测结果:")
                rospy.logdebug(f"  检测到 {len(boxes)} 个目标，推理耗时: {inference_time*1000:.1f}ms")
                rospy.logdebug("开始追踪所有目标...")
                self.first_frame = False
            
            tracked_objects = []
            
            # 处理所有检测结果
            if results[0].boxes is not None:
                boxes = results[0].boxes.xywh.cpu()
                xyxy_boxes = results[0].boxes.xyxy.cpu()
                confidences = results[0].boxes.conf.cpu().numpy()
                classes = results[0].boxes.cls.cpu().numpy()
                track_ids = results[0].boxes.id
                
                rospy.logdebug(f"检测结果处理: {len(boxes)}个边界框")
                
                if track_ids is not None:
                    track_ids = track_ids.int().cpu().tolist()
                    rospy.logdebug(f"追踪ID列表: {track_ids}")
                    
                    for i in range(len(boxes)):
                        track_id = track_ids[i]
                        confidence = confidences[i]
                        class_id = int(classes[i])
                        class_name = self.model.names.get(class_id, f'class_{class_id}')
                        
                        # 更新所有目标的轨迹历史
                        x, y, w, h = boxes[i]
                        track = self.track_history[track_id]
                        track.append((float(x), float(y)))
                        if len(track) > 30:  # 只保留最近30个点
                            track.pop(0)
                        
                        # 准备返回的目标信息
                        x1, y1, x2, y2 = xyxy_boxes[i]
                        tracked_objects.append({
                            'track_id': track_id,
                            'bbox': (float(x1), float(y1), float(x2), float(y2)),
                            'class_name': class_name,
                            'confidence': float(confidence),
                            'center_x': float(x),
                            'center_y': float(y)
                        })
                        
                        rospy.logdebug(f"目标ID {track_id}: {class_name} 置信度:{confidence:.3f} "
                                      f"位置:({x:.0f},{y:.0f}) 轨迹点数:{len(track)}")
                else:
                    rospy.logdebug("检测结果中没有追踪ID - 可能是追踪器初始化问题")
            else:
                rospy.logdebug("当前帧未检测到任何目标")
            
            rospy.logdebug(f"本帧处理完成: {len(tracked_objects)}个追踪目标")
            return tracked_objects
            
        except Exception as e:
            rospy.logerr(f"YOLO检测错误: {e}")
            import traceback
            rospy.logdebug(f"检测错误详情:\n{traceback.format_exc()}")
            rospy.logwarn("检测失败，返回空结果")
            return []
    
    def display_image(self, image, tracked_objects, current_fps):
        """显示带有检测框、ID、中心点和轨迹的图像"""
        annotated_frame = image.copy()
        
        # 显示所有检测到的目标
        for obj in tracked_objects:
            x1, y1, x2, y2 = obj['bbox']
            track_id = obj['track_id']
            class_name = obj['class_name']
            confidence = obj['confidence']
            center_x = obj.get('center_x', (x1 + x2) / 2)
            center_y = obj.get('center_y', (y1 + y2) / 2)
            
            # 根据类别设置不同颜色
            if class_name == 'person':
                color = (0, 255, 0)  # 绿色
            elif class_name == 'car':
                color = (255, 0, 0)  # 蓝色
            elif class_name == 'bicycle':
                color = (0, 255, 255)  # 黄色
            elif class_name == 'motorcycle':
                color = (255, 0, 255)  # 紫色
            else:
                color = (255, 255, 255)  # 白色
            
            # 绘制边界框
            cv2.rectangle(annotated_frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            
            # 绘制中心点
            cv2.circle(annotated_frame, (int(center_x), int(center_y)), 5, color, -1)
            cv2.circle(annotated_frame, (int(center_x), int(center_y)), 8, color, 2)
            
            # 绘制ID和类别标签
            label = f"ID:{track_id} {class_name} {confidence:.2f}"
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            
            # 绘制标签背景
            cv2.rectangle(annotated_frame, 
                         (int(x1), int(y1) - label_size[1] - 10), 
                         (int(x1) + label_size[0], int(y1)), 
                         color, -1)
            
            # 绘制标签文字
            cv2.putText(annotated_frame, label, (int(x1), int(y1-5)), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            
            # 绘制轨迹历史
            if track_id in self.track_history and len(self.track_history[track_id]) > 1:
                points = np.hstack(self.track_history[track_id]).astype(np.int32).reshape((-1, 1, 2))
                cv2.polylines(annotated_frame, [points], isClosed=False, color=color, thickness=2)
        
        # 显示追踪器信息和FPS
        cv2.putText(annotated_frame, "YOLO11 + ByteTrack - All Targets", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (50, 170, 50), 2)
        cv2.putText(annotated_frame, f"FPS: {int(current_fps)}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (50, 170, 50), 2)
        cv2.putText(annotated_frame, f"Targets: {len(tracked_objects)}", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (50, 170, 50), 2)
        
        # 显示图例
        legend_y = 120
        legend_items = [
            ("Person", (0, 255, 0)),
            ("Car", (255, 0, 0)),
            ("Bicycle", (0, 255, 255)),
            ("Motorcycle", (255, 0, 255)),
            ("Others", (255, 255, 255))
        ]
        
        for i, (name, color) in enumerate(legend_items):
            y_pos = legend_y + i * 25
            cv2.rectangle(annotated_frame, (10, y_pos - 10), (30, y_pos + 10), color, -1)
            cv2.putText(annotated_frame, name, (35, y_pos + 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        cv2.imshow("YOLO11 Tracking - All Targets", annotated_frame)
        cv2.waitKey(1)
    
    def publish_detections(self, tracked_objects, header, width, height, processing_time):
        """发布检测结果"""
        try:
            detection_array = DetectionArray()
            detection_array.header = header
            detection_array.header.frame_id = "camera"
            detection_array.image_width = width
            detection_array.image_height = height
            detection_array.total_objects = len(tracked_objects)
            detection_array.processing_time = processing_time
            
            rospy.logdebug(f"准备发布检测数组: 图像尺寸({width}x{height}), "
                          f"目标数量: {len(tracked_objects)}, "
                          f"处理时间: {processing_time*1000:.1f}ms")
            
            for i, obj in enumerate(tracked_objects):
                detection = Detection()
                detection.header = header
                detection.id = obj['track_id']
                detection.class_name = obj['class_name']
                detection.confidence = obj['confidence']
                
                x1, y1, x2, y2 = obj['bbox']
                detection.xyxy = [x1, y1, x2, y2]
                detection.center_x = obj.get('center_x', (x1 + x2) / 2.0)
                detection.center_y = obj.get('center_y', (y1 + y2) / 2.0)
                detection.width = x2 - x1
                detection.height = y2 - y1
                
                detection_array.detections.append(detection)
                
                rospy.logdebug(f"检测对象{i+1}: ID={detection.id}, "
                              f"类别={detection.class_name}, "
                              f"置信度={detection.confidence:.3f}, "
                              f"中心=({detection.center_x:.0f},{detection.center_y:.0f})")
            
            # 发布消息
            publish_start = time.time()
            self.detection_pub.publish(detection_array)
            publish_time = time.time() - publish_start
            
            rospy.logdebug(f"消息发布完成，耗时: {publish_time*1000:.2f}ms")
            
            # 详细日志输出
            if len(tracked_objects) > 0:
                rospy.logdebug(f"成功发布 {len(tracked_objects)} 个追踪目标到话题 /yolo_identify")
            else:
                rospy.logdebug("发布空检测结果（当前帧未检测到目标）")
                
        except Exception as e:
            rospy.logerr(f"发布检测结果时发生错误: {e}")
            import traceback
            rospy.logerr(f"发布错误详情:\n{traceback.format_exc()}")
            rospy.logwarn("检测结果发布失败，但节点继续运行")


def main():
    try:
        rospy.logdebug("启动YOLO11追踪节点...")
        rospy.logdebug(f"ROS节点名称: {rospy.get_name()}")
        rospy.logdebug(f"ROS命名空间: {rospy.get_namespace()}")
        
        # 创建节点实例
        node_start_time = time.time()
        node = CamTrackerNode()
        node_init_time = time.time() - node_start_time
        
        rospy.logdebug(f"节点初始化完成，耗时: {node_init_time:.2f}秒")
        rospy.logdebug("开始监听图像话题，进入主循环...")
        
        # 进入ROS主循环
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.logdebug("键盘中断")
        
    except rospy.ROSInterruptException:
        rospy.logdebug("收到ROS中断信号，节点正常退出")
    except rospy.ROSException as e:
        rospy.logerr(f"ROS异常: {e}")
        rospy.logfatal("节点因ROS异常而退出")
    except Exception as e:
        rospy.logerr(f"节点启动失败: {e}")
        import traceback
        rospy.logdebug(f"致命错误详情:\n{traceback.format_exc()}")
    finally:
        rospy.logdebug("开始清理资源...")
        
        try:
            cv2.destroyAllWindows()
            rospy.logdebug("OpenCV窗口已关闭")
        except:
            pass
            
        rospy.logdebug("资源清理完成")
        rospy.logdebug("cam_tracker node 已停止")


if __name__ == '__main__':
    main()