#!/usr/bin/env python
#coding:utf-8

import rospy
import string
import sys
from respeaker_interface import RespeakerInterface
from respeaker_audio import RespeakerAudio
from std_msgs.msg import String
from geometry_msgs.msg import Twist

reload(sys)
sys.setdefaultencoding( "utf-8" )
       	
class SnowManNode(object):
    def __init__(self, node_name):
        self.node_name = node_name
        rospy.init_node(node_name)
        rospy.on_shutdown(self.shutdown)
        self.respeaker_interface = RespeakerInterface()
        self.respeaker_audio = RespeakerAudio()
        self.ask_pub = rospy.Publisher('/snowman/ask', String, queue_size=5)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        self.rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            text = self.respeaker_audio.record()
            if text.find("前进") >= 0:
                print("forward")
                cmd_msg = Twist()
                cmd_msg.linear.x = 0.3
                cmd_msg.angular.z = 0.0
                self.cmd_pub.publish(cmd_msg)     
            elif text.find("后退") >= 0:
                print("backward")
                cmd_msg = Twist()
                cmd_msg.linear.x = -0.3
                cmd_msg.angular.z = 0.0
                self.cmd_pub.publish(cmd_msg)
            elif text.find("左转") >= 0:
                print("left")
                cmd_msg = Twist()
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.3
                self.cmd_pub.publish(cmd_msg)
            elif text.find("右转") >= 0:
                print("right")
                cmd_msg = Twist()
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = -0.3
                self.cmd_pub.publish(cmd_msg)            
            print(text)
        self.rate.sleep()
   
    def shutdown(self):
        self.cmd_pub.publish(Twist())
        self.respeaker_interface.close()
        self.respeaker_audio.stop()
        
if __name__ == '__main__':
    snowmanNode = SnowManNode("snowman_node")

        
        
        
        
        
