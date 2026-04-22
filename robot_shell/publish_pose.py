#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Point
import tf

def publish_pose():
    rospy.init_node('pose_publisher_node', anonymous=True)
    pub = rospy.Publisher('/robot_xy_pose', Point, queue_size=10)
    listener = tf.TransformListener()
    rate = rospy.Rate(1.0) # 1Hz

    rospy.loginfo("Starting XY pose publisher...")
    while not rospy.is_shutdown():
        try:
            # 获取map到base_footprint或base_link的坐标变换
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            p = Point()
            p.x = trans[0]
            p.y = trans[1]
            p.z = trans[2]
            pub.publish(p)
            rospy.loginfo("Published Pose -> x: {:.3f}, y: {:.3f}".format(p.x, p.y))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_pose()
    except rospy.ROSInterruptException:
        pass
