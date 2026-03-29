#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Quaternion
import tf
from math import radians, copysign
from transform_utils import quat_to_angle, normalize_angle

from respeaker_interface import RespeakerInterface
from respeaker_audio import RespeakerAudio
from std_msgs.msg import String


        	
class SnowManNode(object):
    def __init__(self, node_name):
        self.node_name = node_name
        rospy.init_node(node_name)
        rospy.on_shutdown(self.shutdown)
        self.rate = rospy.get_param('~rate', 20)
        r = rospy.Rate(self.rate)
        self.test_angle = 0
        self.speed = rospy.get_param('~speed', 1.0) 
        self.tolerance = radians(rospy.get_param('tolerance', 1)) 
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.base_frame = rospy.get_param('~base_frame', '/base_link')
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        self.tf_listener = tf.TransformListener()
        self.respeaker_interface = RespeakerInterface()
        self.respeaker_audio = RespeakerAudio()
        self.ask_pub = rospy.Publisher('/snowman/ask', String, queue_size=5)
        self.start = False
        rospy.sleep(2)
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))
        while not rospy.is_shutdown():
            self.odom_angle = self.get_odom_angle()
            text = self.respeaker_audio.record()
            self.ask_pub.publish(str(text))
            direction = self.respeaker_interface.direction
            if text.lower().find("hello") >= 0:
                print("nin hao") 
                self.start = True
                self.test_angle = direction                           
            if self.start == True:
                last_angle = self.odom_angle
                turn_angle = 0
                error = self.test_angle - turn_angle
                while abs(error) > self.tolerance and self.start:
                    move_cmd = Twist()
                    move_cmd.angular.z = copysign(self.speed, error)
                    self.cmd_vel.publish(move_cmd)
                    r.sleep()
                    self.odom_angle = self.get_odom_angle()
                    delta_angle = normalize_angle(self.odom_angle - last_angle)
                    turn_angle += delta_angle
                    error = self.test_angle - turn_angle
                    last_angle = self.odom_angle
                self.cmd_vel.publish(Twist())
                self.start = False
            rospy.sleep(0.5)
        self.cmd_vel.publish(Twist())
                
    def get_odom_angle(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except:
            rospy.loginfo("TF Exception")
            return       
        # Convert the rotation from a quaternion to an Euler angle
        return quat_to_angle(Quaternion(*rot))            
   
    def shutdown(self):
        self.respeaker_interface.close()
        self.respeaker_audio.stop()
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
        
if __name__ == '__main__':
    try:
        SnowManNode()
    except:
        rospy.loginfo("Calibration terminated.")
        
        
        
        
        
