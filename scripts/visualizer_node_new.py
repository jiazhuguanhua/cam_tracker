#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Cam Tracker Visualizer Node (Optimized UI with Tracking ID)
Features:
1. Subscribe to /usb_cam/image_raw for camera image
2. Subscribe to /cam_tracker/info for detection info (ball and car positions with IDs)
3. Display real-time visualization with OpenCV window
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cam_tracker.msg import CamTrack


class CamTrackerVisualizer:
    def __init__(self):
        """Initialize visualizer node"""
        rospy.init_node('cam_tracker_visualizer', anonymous=True)
        
        # OpenCV bridge
        self.bridge = CvBridge()
        
        # Store latest data
        self.latest_image = None
        self.latest_detection = None
        
        # Create OpenCV window
        self.window_name = "Cam Tracker Visualizer"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1280, 720)
        
        # Subscribe to topics
        rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback, queue_size=1)
        rospy.Subscriber('/cam_tracker/info', CamTrack, self.detection_callback, queue_size=1)
        
        rospy.loginfo("=== Cam Tracker Visualizer with Tracking ID ===")
        rospy.loginfo("Subscribe: /usb_cam/image_raw")
        rospy.loginfo("Subscribe: /cam_tracker/info")
        rospy.loginfo("Press 'q' to quit, 's' to save screenshot")
    
    def image_callback(self, msg):
        """Receive and convert image"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Image conversion failed: {e}")
    
    def detection_callback(self, msg):
        """Receive detection info"""
        self.latest_detection = msg
    
    def draw_detections(self, image):
        """Draw detection results on image with enhanced UI"""
        if self.latest_detection is None:
            cv2.putText(image, "Waiting for detection data...", (20, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            return image
        
        det = self.latest_detection
        h, w = image.shape[:2]
        
        # === Draw top status bar with semi-transparent background ===
        overlay = image.copy()
        cv2.rectangle(overlay, (0, 0), (w, 120), (40, 40, 40), -1)
        cv2.addWeighted(overlay, 0.7, image, 0.3, 0, image)
        
        # Draw separator line
        cv2.line(image, (0, 120), (w, 120), (100, 100, 100), 2)
        
        # System status with icon
        status_color = (0, 255, 0) if det.system_ok else (0, 0, 255)
        status_text = "System: OK" if det.system_ok else "System: ERROR"
        cv2.circle(image, (30, 35), 8, status_color, -1)
        cv2.putText(image, status_text, (50, 45), 
                   cv2.FONT_HERSHEY_DUPLEX, 0.9, status_color, 2)
        
        # Ball count with icon and tracking ID
        cv2.circle(image, (30, 85), 12, (0, 255, 255), 2)
        ball_text = f"Ball: {det.ball_num}"
        if det.ball_id >= 0:
            ball_text += f" [ID:{det.ball_id}]"
        cv2.putText(image, ball_text, (50, 95), 
                   cv2.FONT_HERSHEY_DUPLEX, 0.9, (0, 255, 255), 2)
        
        # Car count with icon
        cv2.rectangle(image, (330, 73), (354, 97), (255, 0, 255), 2)
        cv2.putText(image, f"Car: {det.car_num}", (365, 95), 
                   cv2.FONT_HERSHEY_DUPLEX, 0.9, (255, 0, 255), 2)
        
        # Gripper status with icon
        gripper_text = "Gripper: CLOSED" if det.in_gripper else "Gripper: OPEN"
        gripper_color = (0, 200, 255) if det.in_gripper else (150, 150, 150)
        gripper_x = 570
        # Draw simple gripper icon
        if det.in_gripper:
            cv2.rectangle(image, (gripper_x, 80), (gripper_x+10, 92), gripper_color, -1)
        else:
            cv2.rectangle(image, (gripper_x, 80), (gripper_x+4, 92), gripper_color, -1)
            cv2.rectangle(image, (gripper_x+16, 80), (gripper_x+20, 92), gripper_color, -1)
        cv2.putText(image, gripper_text, (gripper_x+30, 95), 
                   cv2.FONT_HERSHEY_DUPLEX, 0.9, gripper_color, 2)
        
        # === Draw Ball detection (Yellow) with Tracking ID ===
        if det.ball_num > 0:
            bx, by = int(det.ball_x), int(det.ball_y)
            
            # Outer circle with glow effect
            cv2.circle(image, (bx, by), 45, (0, 200, 200), 1)
            cv2.circle(image, (bx, by), 38, (0, 255, 255), 3)
            cv2.circle(image, (bx, by), 6, (0, 255, 255), -1)
            
            # Crosshair
            line_len = 25
            cv2.line(image, (bx - line_len, by), (bx + line_len, by), (0, 255, 255), 2)
            cv2.line(image, (bx, by - line_len), (bx, by + line_len), (0, 255, 255), 2)
            
            # Label with background - include ID
            if det.ball_id >= 0:
                label = f"Ball ID:{det.ball_id} ({bx}, {by})"
            else:
                label = f"Ball ({bx}, {by})"
            
            (label_w, label_h), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_DUPLEX, 0.8, 2)
            label_x, label_y = bx + 55, by - 15
            # Draw background with border
            cv2.rectangle(image, (label_x - 5, label_y - label_h - 5), 
                         (label_x + label_w + 5, label_y + baseline + 5), (0, 0, 0), -1)
            cv2.rectangle(image, (label_x - 5, label_y - label_h - 5), 
                         (label_x + label_w + 5, label_y + baseline + 5), (0, 255, 255), 2)
            cv2.putText(image, label, (label_x, label_y), 
                       cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 255, 255), 2)
        
        # === Draw Car detection (Magenta) with Tracking IDs ===
        if det.car_num > 0:
            for i in range(det.car_num):
                cx, cy = int(det.car_x[i]), int(det.car_y[i])
                car_id = det.car_ids[i] if i < len(det.car_ids) else -1
                
                # Double rectangle with glow effect
                cv2.rectangle(image, (cx - 55, cy - 55), (cx + 55, cy + 55), (200, 0, 200), 1)
                cv2.rectangle(image, (cx - 48, cy - 48), (cx + 48, cy + 48), (255, 0, 255), 3)
                cv2.circle(image, (cx, cy), 6, (255, 0, 255), -1)
                
                # Corner markers
                corner_len = 15
                # Top-left
                cv2.line(image, (cx - 48, cy - 48), (cx - 48 + corner_len, cy - 48), (255, 0, 255), 3)
                cv2.line(image, (cx - 48, cy - 48), (cx - 48, cy - 48 + corner_len), (255, 0, 255), 3)
                # Top-right
                cv2.line(image, (cx + 48, cy - 48), (cx + 48 - corner_len, cy - 48), (255, 0, 255), 3)
                cv2.line(image, (cx + 48, cy - 48), (cx + 48, cy - 48 + corner_len), (255, 0, 255), 3)
                # Bottom-left
                cv2.line(image, (cx - 48, cy + 48), (cx - 48 + corner_len, cy + 48), (255, 0, 255), 3)
                cv2.line(image, (cx - 48, cy + 48), (cx - 48, cy + 48 - corner_len), (255, 0, 255), 3)
                # Bottom-right
                cv2.line(image, (cx + 48, cy + 48), (cx + 48 - corner_len, cy + 48), (255, 0, 255), 3)
                cv2.line(image, (cx + 48, cy + 48), (cx + 48, cy + 48 - corner_len), (255, 0, 255), 3)
                
                # Label with background - include ID
                if car_id >= 0:
                    label = f"Car ID:{car_id} ({cx}, {cy})"
                else:
                    label = f"Car #{i+1} ({cx}, {cy})"
                
                (label_w, label_h), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_DUPLEX, 0.8, 2)
                label_x, label_y = cx + 65, cy - 15
                # Draw background with border
                cv2.rectangle(image, (label_x - 5, label_y - label_h - 5), 
                             (label_x + label_w + 5, label_y + baseline + 5), (0, 0, 0), -1)
                cv2.rectangle(image, (label_x - 5, label_y - label_h - 5), 
                             (label_x + label_w + 5, label_y + baseline + 5), (255, 0, 255), 2)
                cv2.putText(image, label, (label_x, label_y), 
                           cv2.FONT_HERSHEY_DUPLEX, 0.8, (255, 0, 255), 2)
        
        return image
    
    def run(self):
        """Main loop: display images"""
        rospy.loginfo("Visualizer node is running...")
        
        rate = rospy.Rate(30)  # 30Hz refresh rate
        
        while not rospy.is_shutdown():
            if self.latest_image is not None:
                # Copy image for drawing
                display_image = self.latest_image.copy()
                
                # Draw detection results
                display_image = self.draw_detections(display_image)
                
                # Display image
                cv2.imshow(self.window_name, display_image)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    rospy.loginfo("User pressed 'q', exiting node")
                    break
                elif key == ord('s'):
                    # Save screenshot
                    filename = f"/tmp/cam_tracker_{rospy.Time.now().to_sec():.0f}.jpg"
                    cv2.imwrite(filename, display_image)
                    rospy.loginfo(f"Screenshot saved: {filename}")
            else:
                # Show waiting screen when no image data
                waiting_image = 50 * np.ones((480, 640, 3), dtype=np.uint8)
                cv2.putText(waiting_image, "Waiting for camera data...", (120, 220), 
                           cv2.FONT_HERSHEY_DUPLEX, 1.0, (0, 255, 255), 2)
                cv2.putText(waiting_image, "Subscribing to /usb_cam/image_raw", (80, 260), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (150, 150, 150), 1)
                cv2.imshow(self.window_name, waiting_image)
                cv2.waitKey(1)
            
            rate.sleep()
        
        # Cleanup
        cv2.destroyAllWindows()


def main():
    try:
        visualizer = CamTrackerVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted")
    except Exception as e:
        rospy.logerr(f"Node exception: {e}")
        import traceback
        traceback.print_exc()
    finally:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
