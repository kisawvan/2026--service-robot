#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist


class linefollow:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=1)
        self.pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        self.twist = Twist()
        self.logmark = True

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")     
            frame = np.array(cv_image, dtype=np.uint8)
        except CvBridgeError, e:
            print e
        
        frame1 = cv2.resize(frame,(160,120),interpolation=cv2.INTER_CUBIC)

        gray = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)

        ret,binary  = cv2.threshold(gray,60,255,cv2.THRESH_BINARY)
        kernel = np.ones((5,5),np.uint8)
        opening = cv2.morphologyEx(binary,cv2.MORPH_OPEN,kernel)
        binary = cv2.morphologyEx(binary,cv2.MORPH_CLOSE,kernel)
        counter=0
        sumvalue=0
        for i in range(1,160):
            if binary[20,i] == 0:
                counter+=1
                sumvalue+=i
            else:
                pass
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0
        if(counter != 0):
            if (counter < 100):
                self.twist.linear.x = 0.1
                self.twist.angular.z = (80-(sumvalue/counter))/80.0*1.0
                self.pub_cmd.publish(self.twist)
            else:
                if self.logmark:
                    self.logmark = False
                    rospy.loginfo("Unrecognized Line in vision")
            self.logmark = True
        else:
            if self.logmark:
                self.logmark = False
                rospy.loginfo("No Line in vision")
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(binary , encoding="passthrough"))

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        rospy.init_node("line follow")
        linefollow()
        rospy.loginfo("line follow is started..")
        rospy.loginfo("Please subscribe the ROS image.")
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down line follow node."
        cv2.destroyAllWindows()
