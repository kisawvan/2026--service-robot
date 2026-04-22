#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import mediapipe as mp
from sensor_msgs.msg import Image
from std_msgs.msg import String

def imgmsg_to_cv2(img_msg):
    # 抛弃 cv_bridge，直接解析二进制数据解决 ROS Melodic 下的 Python3 兼容问题
    return np.ndarray(shape=(img_msg.height, img_msg.width, 3),
                      dtype=np.uint8, buffer=img_msg.data)

def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height, img_msg.width, _ = cv_image.shape
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tobytes()
    img_msg.step = len(img_msg.data) // img_msg.height
    return img_msg

class GestureRecognitionCNN:
    def __init__(self):
        rospy.init_node('gesture_recognition_cnn_node', anonymous=True)
        
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.result_pub = rospy.Publisher("/gesture_recognition/result", String, queue_size=10)
        self.image_pub = rospy.Publisher("/gesture_recognition/image", Image, queue_size=10)
        
        # 初始化 MediaPipe 姿势/手部识别 (底层使用轻量级 CNN 网络)
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils
        
        # 指尖关键点索引 (大拇指、食指、中指、无名指、小指)
        self.tip_ids = [4, 8, 12, 16, 20]

        rospy.loginfo("CNN-based Gesture Recognition Node Started. Waiting for images...")

    def image_callback(self, data):
        try:
            cv_image = imgmsg_to_cv2(data)
        except Exception as e:
            rospy.logerr("Image Convert Error: {0}".format(e))
            return

        # 转换为 RGB 供 MediaPipe CNN 处理
        img_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        results = self.hands.process(img_rgb)
        
        gesture_result = "None"
        
        if results.multi_hand_landmarks:
            for hand_lms in results.multi_hand_landmarks:
                # 绘制手部关键点连线
                self.mp_draw.draw_landmarks(cv_image, hand_lms, self.mp_hands.HAND_CONNECTIONS)
                
                lm_list = []
                for id, lm in enumerate(hand_lms.landmark):
                    h, w, c = cv_image.shape
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    lm_list.append([id, cx, cy])
                
                if len(lm_list) != 0:
                    fingers = []
                    
                    # 大拇指：判断指尖与关节的左右相对位置
                    if lm_list[self.tip_ids[0]][1] > lm_list[self.tip_ids[0] - 1][1]:
                        fingers.append(1)
                    else:
                        fingers.append(0)
                        
                    # 其余四指：判断指尖是否高于第二关节
                    for id in range(1, 5):
                        if lm_list[self.tip_ids[id]][2] < lm_list[self.tip_ids[id] - 2][2]:
                            fingers.append(1)
                        else:
                            fingers.append(0)
                            
                    total_fingers = fingers.count(1)
                    
                    # 根据张开的手指数量映射手势
                    if total_fingers == 0:
                        gesture_result = "Fist / Rock"
                    elif total_fingers == 2 and fingers[1]==1 and fingers[2]==1:
                        gesture_result = "Scissors / Peace"
                    elif total_fingers == 5:
                        gesture_result = "Paper / Full Hand"
                    else:
                        gesture_result = "{} Fingers".format(total_fingers)

                self.result_pub.publish(gesture_result)
        
        # 画面显示及发布
        cv2.putText(cv_image, "Gesture: " + gesture_result, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
        
        try:
            self.image_pub.publish(cv2_to_imgmsg(cv_image))
        except Exception as e:
            rospy.logerr(e)

if __name__ == '__main__':
    try:
        gr_cnn = GestureRecognitionCNN()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass