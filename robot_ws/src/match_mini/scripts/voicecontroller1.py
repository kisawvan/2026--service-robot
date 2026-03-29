#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import sys
reload(sys)
sys.setdefaultencoding('utf8')
import os
import rospy

import time
from respeaker_interface import RespeakerInterface
from respeaker_audio import RespeakerAudio
from std_msgs.msg import String


class VoiceController(object):
    def __init__(self, node_name):
        self.node_name = node_name
        rospy.init_node(node_name)
        rospy.on_shutdown(self.shutdown)
        self.respeaker_interface = RespeakerInterface()
        self.respeaker_audio = RespeakerAudio()
        self.ask_pub = rospy.Publisher('cmd_msg', String, queue_size=5)

    def shutdown(self):
        self.respeaker_interface.close()
        self.respeaker_audio.stop()

def callback(msg):
    os.system("mpg123 /home/bcsh/robot_ws/src/match_mini/voice/zhuabu.mp3")
 
if __name__ == '__main__':
    voice_controller = VoiceController("voice_controller")
    auto_slam = rospy.Publisher('auto_slam', String, queue_size=10)
    rospy.Subscriber("get_pos", String, callback, queue_size=10)
    
    print("等待键盘输入 'g' 启动自动导航...")
    
    while not rospy.is_shutdown():
        # 获取用户输入
        user_input = raw_input("请输入指令: ").strip().lower()
        
        if user_input == 'g':
            print("发送导航启动指令...")
            # 创建并发布 "start" 消息
            start_msg = String()
            start_msg.data = "start"
            auto_slam.publish(start_msg)
            rospy.loginfo("已通过 auto_slam 话题发布启动指令")
        elif user_input == 'q':
            print("退出程序")
            break
        else:
            print("未知指令，请按 'g' 启动导航或 'q' 退出")
