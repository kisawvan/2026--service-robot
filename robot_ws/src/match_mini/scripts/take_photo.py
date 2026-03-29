#!/usr/bin/env python

import rospy
import cv2
from ros_opencv import ROS2OPENCV
import sys, select, os


class TakePhoto(ROS2OPENCV):
    def __init__(self, node_name):
        super(TakePhoto, self).__init__(node_name)
        self.detect_box = None
        self.result = None
        self.count = 0
        self.person_name = rospy.get_param('~person_name', 'name_one')
        self.face_cascade = cv2.CascadeClassifier('/home/bcsh/robot_ws/src/match_mini/scripts/cascades/haarcascade_frontalface_default.xml')
        self.dirname = "/home/bcsh/robot_ws/src/match_mini/scripts/p1/" + self.person_name + "/"
        self.X = None
        self.Y = None
        
        
    
    def process_image(self, frame):
        print("sss")
        src = frame.copy()
        gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
        result = src.copy()
        self.result = result
        for (x, y, w, h) in faces:
            result = cv2.rectangle(result, (x, y), (x+w, y+h), (255, 0, 0), 2)
            f = cv2.resize(gray[y:y+h, x:x+w], (200, 200))
            if self.count<20:
                if key == 'p' :
                    cv2.imwrite(self.dirname + '%s.pgm' % str(self.count), f)
                    self.count += 1
        return result
    
        
if __name__ == '__main__':
    try:
        node_name = "take_photo_rec"
        TakePhoto(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down face detector node."
cv2.destroyAllWindows()
