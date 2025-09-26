#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import numpy as np
from cam_tracker.msg import Detection
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def video_callback(msg):
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    height, width = cv_image.shape[:2]
    

def data_callback(msg):
    global detect_data
    detect_data = msg


def main():
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