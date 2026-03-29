#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from ros_opencv import ROS2OPENCV
from std_msgs.msg import String

class RedBallDetector(ROS2OPENCV):
    def __init__(self, node_name):
        super(RedBallDetector, self).__init__(node_name)
        self.detect_box = None
        self.initRange()
    
    def process_image(self, frame):
        src = frame.copy()
        ###convert rgb to hsv###
        hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
        
        ###create inrange mask(yellow and red)###
        res = src.copy()
        mask_red1 = cv2.inRange(hsv, self.red_min, self.red_max)
        mask_red2 = cv2.inRange(hsv, self.red2_min, self.red2_max)
        mask = cv2.bitwise_or(mask_red1, mask_red2)

        res = cv2.bitwise_and(src, src, mask=mask)
        h,w = res.shape[:2]
        blured = cv2.blur(res,(5,5))
        ret, bright = cv2.threshold(blured,10,255,cv2.THRESH_BINARY)
        ###open and close calculate###
        gray = cv2.cvtColor(bright,cv2.COLOR_BGR2GRAY)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        opened = cv2.morphologyEx(gray, cv2.MORPH_OPEN, kernel)
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)
        cv2.imshow("gray", closed)
        circles = cv2.HoughCircles(closed, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=0, maxRadius=0)
        if circles is not None:
            circles = np.uint16(np.around(circles)) 
            for i in circles[0, : ]:
                cv2.circle(frame, (i[0], i[1]), i[2], (0, 0, 255), 2)
                cv2.circle(frame, (i[0], i[1]), 2, (0, 0, 255), 2)
        processed_image = frame.copy() 
        return processed_image
    
    def initRange(self):
        self.red_min = np.array([0, 128, 46])
        self.red_max = np.array([5, 255,  255])
        self.red2_min = np.array([156, 128,  46])
        self.red2_max = np.array([180, 255,  255])

if __name__ == '__main__':
    try:
        node_name = "red_ball_detector"
        RedBallDetector(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down face detector node."
cv2.destroyAllWindows()
