#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import numpy as np
from cam_tracker.msg import Detection
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import message_filters


def rs_callback(data1,data2):
    print(1)
    color_image = bridge.imgmsg_to_cv2(data1, 'bgr8')
    depth_image = bridge.imgmsg_to_cv2(data2, '16UC1')
    c_x = 320
    c_y = 240
    real_z = depth_image[c_y, c_x] * 0.001  
    real_x = (c_x- ppx) / fx * real_z
    real_y = (c_y - ppy) / fy * real_z
    rospy.loginfo("potion:x=%f,y=%f,z=%f",real_x,real_y,real_z) 

def video_callback(msg):
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    height, width = cv_image.shape[:2]
    

def data_callback(msg):
    global detect_data
    detect_data = msg


def main():
    '''
    global fx, fy, ppx, ppy #相机内参
    fx = 609.134765 
    fy = 608.647949
    ppx = 312.763214
    ppy = 240.882049
    '''

    global bridge
    bridge = CvBridge()

    rospy.init_node('visualizer_node', anonymous=True)

    sub = rospy.Subscriber(
        name="/detection/data",
        data_class=Detection,
        callback=data_callback,
        queue_size=10
    )
    sub_video = rospy.Subscriber(
        name="/camera/color/image_raw",
        data_class=Image,
        callback=video_callback,
        queue_size=1
    )
    '''
    sub_dvideo = rospy.Subscriber(
        name="/camera/aligned_depth_to_color/image_raw",
        data_class=Image,
        callback=
    )
    '''

    color = message_filters.Subscriber("/camera/color/image_raw", Image)
    depth = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
    color_depth = message_filters.ApproximateTimeSynchronizer([color, depth], 10, 1, allow_headerless=True)
    color_depth.registerCallback(rs_callback)  # 不触发回调？

    rospy.loginfo("Visualizer node running...")
    rospy.on_shutdown(lambda: cv2.destroyAllWindows())
    cv2.namedWindow("Visulizer Node", cv2.WINDOW_NORMAL)
    
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if cv_image is not None:
            point_coords = (int(detect_data.detection_x), int(detect_data.detection_y))
            cv2.circle(cv_image, point_coords, 8, (0, 255, 255), -1)
            cv2.imshow("Visulizer Node", cv_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                return
        else:
            cv2.waitKey(1)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Visualizer node exited successfully!")