#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import apriltag
import numpy as np
from ros_opencv import ROS2OPENCV
from robot_vision.msg import AprilTagDetection

class AprilTagDetector(ROS2OPENCV):
    def __init__(self, node_name):
        super(AprilTagDetector, self).__init__(node_name)
        
        # 初始化检测器
        self.detector = apriltag.Detector(
            apriltag.DetectorOptions(families='tag36h11'))
        
        # 参数配置
        self.rotate_image = rospy.get_param('~rotate_image', False)
        self.show_markers = rospy.get_param('~show_markers', True)
        
        # 初始化发布器
        self.detection_pub = rospy.Publisher("/apriltag_detector/detections", 
                                          AprilTagDetection, 
                                          queue_size=10)

    def process_image(self, frame):
        try:
            # 图像预处理
            if self.rotate_image:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            
            # 核心检测流程
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            tags = self.detector.detect(gray)
            
            # 处理检测结果
            for tag in tags:
                if self.show_markers:
                    self._draw_tag_markers(frame, tag)
                self._publish_detection(tag, frame.shape)
            
            return frame
        except Exception as e:
            rospy.logerr("AprilTag检测错误: %s" % str(e))
            return frame

    def _draw_tag_markers(self, frame, tag):
        """绘制标记"""
        corners = tag.corners.astype(int)
        center = tag.center.astype(int)
        
        cv2.polylines(frame, [corners], True, (0, 255, 0), 2)
        cv2.putText(frame, "ID: {}".format(tag.tag_id), 
                   (corners[0][0], corners[0][1]-10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.circle(frame, tuple(center), 5, (0, 0, 255), -1)
        
        for i, corner in enumerate(corners):
            cv2.circle(frame, tuple(corner), 4, (255, 0, 0), 2)
            cv2.putText(frame, str(i), tuple(corner),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

    def _publish_detection(self, tag, image_shape):
        """发布检测结果"""
        detection = AprilTagDetection()
        detection.header.stamp = rospy.Time.now()
        detection.tag_id = tag.tag_id
        
        height, width = image_shape[:2]
        detection.x = tag.center[0] / width
        detection.y = tag.center[1] / height
        
        self.detection_pub.publish(detection)

if __name__ == '__main__':
    try:
        node_name = "apriltag_detector"
        AprilTagDetector(node_name)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

